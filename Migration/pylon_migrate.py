#!/usr/bin/env python3
"""One-shot migration of project-owned KSP identifiers; no runtime compatibility layer."""
from __future__ import annotations
import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

MAPPING = json.loads(Path(__file__).with_name('identifiers.json').read_text())
MODULES = MAPPING['modules']; PARTS = MAPPING['parts']; CONFIGS = MAPPING['configs']
OLD_PACKAGES = ('ksp_lidar_bridge', 'ksp_ros2_interfaces', 'ksp_vehicle_control',
                'ksp_nav2_bringup', 'debris_orbit', 'position_estimator', 'mun_rover_demo')

@dataclass
class Entry:
    key: str
    value: str
    start: int
    end: int
    value_start: int
    value_end: int

@dataclass
class Node:
    name: str
    start: int = 0
    end: int = 0
    entries: list[Entry] = field(default_factory=list)
    children: list[Node] = field(default_factory=list)
    parent: Node | None = field(default=None, repr=False)
    def get(self, key):
        values = [e.value for e in self.entries if e.key == key]
        if len(set(values)) > 1: raise ValueError(f'Conflicting {self.name}.{key}')
        return values[0] if values else ''

def parse(text):
    root = Node('ROOT'); stack = [root]; pending = None
    # Quoted braces are values; comments and whitespace remain byte-for-byte intact.
    token = re.compile(r'"(?:\\.|[^"\\])*"|//[^\n]*|[{}\n]|[^{}\n"/]+|/(?!/)', re.M)
    pieces=[]; begin=0
    def flush(end):
        nonlocal pending, pieces, begin
        raw=''.join(pieces); pieces=[]
        stripped=raw.strip()
        if not stripped:return
        if '=' in raw:
            key,value=raw.split('=',1); left=begin+raw.index('=')+1
            leading=len(value)-len(value.lstrip()); trailing=len(value.rstrip())
            stack[-1].entries.append(Entry(key.strip(),value.strip(),begin,end,left+leading,left+trailing))
        else:pending=(stripped,begin+len(raw)-len(raw.lstrip()))
    for m in token.finditer(text):
        value=m.group()
        if value.startswith('//'):
            flush(m.start()); continue
        if value in ('\n','{','}'):
            flush(m.start())
            if value=='{':
                if pending is None:raise ValueError('Unnamed configuration node')
                node=Node(pending[0],pending[1],parent=stack[-1]);stack[-1].children.append(node);stack.append(node);pending=None
            elif value=='}':
                if len(stack)==1:raise ValueError('Unbalanced closing brace')
                stack.pop().end=m.start();pending=None
            begin=m.end()
        else:
            if not pieces:begin=m.start()
            pieces.append(value)
    flush(len(text))
    if len(stack)!=1:raise ValueError('Unclosed configuration node')
    return root

def walk(node):
    yield node
    for child in node.children:yield from walk(child)

def part_identifier(value):
    for old,new in PARTS.items():
        for a,b in ((old,new),(old.replace('_','.'),new.replace('_','.'))):
            if value==a:return b
            if value.startswith(a+'_') and value[len(a)+1:].isdigit():return b+value[len(a):]
    return value

def transform(text):
    root=parse(text);edits=[];settings={};model={}
    def setting(target,key,value):
        if key in target and target[key]!=value:raise ValueError(f'Conflicting shared setting {key}: {target[key]} / {value}')
        target[key]=value
    def replace(entry,value):
        if entry.value!=value:edits.append((entry.value_start,entry.value_end,value))
    sensor_modules={'ModuleKerbalLidar','ModulePyLoNLidar','ModuleKerbalRgbCamera','ModulePyLoNRgbCamera'}
    for node in walk(root):
        if node.name in CONFIGS:edits.append((node.start,node.start+len(node.name),CONFIGS[node.name]))
        module=node.get('name') if node.name in ('MODULE','SCENARIO') else ''
        owned=module in MODULES or module in MODULES.values()
        for entry in node.entries:
            if node.name=='PART' and entry.key in ('name','part'):
                replace(entry,part_identifier(entry.value))
            elif node.name=='PART' and entry.key in ('link','sym','parent'):
                replace(entry,part_identifier(entry.value))
            elif node.name=='PART' and entry.key in ('attN','srfN'):
                # Attachment node label and whitespace belong to the craft. Only
                # its referenced part token changes; numeric save indices stay.
                replace(entry,re.sub(r'(?<=,)(\s*)([^,\s]+)',lambda m:m[1]+part_identifier(m[2]),entry.value))
            elif owned and entry.key=='name':replace(entry,MODULES.get(entry.value,entry.value))
            elif node.name=='MODEL' and entry.key in ('model','texture') and entry.value.startswith('KerbalLiDAR/'):
                replace(entry,'PyLoN/'+entry.value[len('KerbalLiDAR/'):])
        if not owned:continue
        for entry in node.entries:
            if module in ('ModuleKerbalLidar','ModulePyLoNLidar') and entry.key.startswith('visualModel'):
                if entry.key=='visualModelObjPath' and entry.value and not entry.value.startswith(('KerbalLiDAR/','PyLoN/','GameData/KerbalLiDAR/','GameData/PyLoN/')):
                    raise ValueError('Custom OBJ model needs conversion to a native MODEL node before migration')
                edits.append((entry.start,entry.end,''))
                continue
            endpoint={'udpHost':'stateHost','udpPort':'statePort','stateUdpHost':'stateHost','stateUdpPort':'statePort','commandUdpPort':'commandPort'}.get(entry.key)
            model_key={'activeVesselUrdfEnabled':'enabled','activeVesselUrdfRefreshSeconds':'refreshSeconds','activeVesselUrdfChunkBytes':'chunkBytes','maxActiveVesselUrdfChunks':'maxChunks','allowRemoteUrdf':'allowRemoteUrdf'}.get(entry.key)
            if endpoint:
                setting(settings,endpoint,entry.value);edits.append((entry.start,entry.end,''))
            if model_key:
                setting(model,model_key,entry.value);edits.append((entry.start,entry.end,''))
        if module in sensor_modules and node.parent is not None:
            old_id=node.get('partName') or node.get('lidarName')
            siblings=[n for n in node.parent.children if n.name=='MODULE' and n.get('name') in ('ModuleKerbalRosSensorId','ModulePyLoNSensorId')]
            if old_id:
                if len(siblings)>1:raise ValueError('Multiple Sensor ID modules on one part')
                if siblings:
                    sibling=siblings[0];current=sibling.get('sensorId')
                    # The previous runtime already preferred the canonical sensorId.
                    # Obsolete partName/lidarName values must not overwrite it.
                    if not current:
                        entries=[e for e in sibling.entries if e.key=='sensorId']
                        if entries:replace(entries[0],old_id)
                        else:edits.append((sibling.end,sibling.end,'\n sensorId = '+old_id+'\n'))
                else:edits.append((node.parent.end,node.parent.end,'\n MODULE\n {\n  name = ModulePyLoNSensorId\n  sensorId = '+old_id+'\n }\n'))
            for entry in node.entries:
                if entry.key in ('partName','lidarName'):edits.append((entry.start,entry.end,''))
    for start,end,value in sorted(set(edits),reverse=True):text=text[:start]+value+text[end:]
    return text,settings,model

def runtime_config(settings,model):
    settings={'stateHost':'127.0.0.1','statePort':'49010','commandPort':'49011',**settings}
    model={'enabled':'true','allowRemoteUrdf':'false','refreshSeconds':'2','chunkBytes':'12000','maxChunks':'256',**model}
    return ''.join(name+'\n{\n'+''.join('    '+k+' = '+v+'\n' for k,v in values.items())+'}\n' for name,values in [('PYLON_TRANSPORT',settings),('PYLON_MODEL',model)])

def backup_path(path):
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    return path.with_name(path.name+'.pre-pylon-'+stamp)

def migrate(source,output=None,in_place=False):
    source=Path(source).resolve()
    paths=sorted(p for p in source.rglob('*') if p.suffix.lower() in ('.craft','.sfs','.cfg')) if source.is_dir() else [source]
    changes=[];settings={};model={}
    for path in paths:
        raw=path.read_bytes();text=raw.decode('utf-8-sig');converted,a,b=transform(text)
        for target,values in ((settings,a),(model,b)):
            for key,value in values.items():
                if key in target and target[key]!=value:raise ValueError(f'Conflicting {key} across input files')
                target[key]=value
        data=(b'\xef\xbb\xbf' if raw.startswith(b'\xef\xbb\xbf') else b'')+converted.encode('utf-8')
        changes.append((path,raw,data))
    # Preflight every input before writing anything.
    destinations=[]
    if output:
        output=Path(output).resolve()
        if output==source or (source.is_dir() and output.is_relative_to(source)):raise ValueError('Output must be outside the input tree')
    for path,raw,data in changes:
        print(('CHANGE ' if raw!=data else 'KEEP   ')+str(path))
        if output:
            destination=output/(path.relative_to(source) if source.is_dir() else path.name)
            if destination.exists() and destination.read_bytes()!=data:raise ValueError(f'Output exists with different content: {destination}')
            destinations.append((destination,data))
    if output and (settings or model):
        destination=output/'Runtime.cfg';data=runtime_config(settings,model).encode()
        if destination.exists() and destination.read_bytes()!=data:raise ValueError('Conflicting output Runtime.cfg')
        destinations.append((destination,data))
    unique_destinations={}
    for destination,data in destinations:
        if destination in unique_destinations and unique_destinations[destination]!=data:
            raise ValueError(f'Conflicting generated configuration: {destination}')
        unique_destinations[destination]=data
    if in_place:
        if settings or model:raise ValueError('Use --output first: per-part settings must be installed as Config/Runtime.cfg')
        for path,raw,data in changes:
            if raw!=data:
                shutil.copy2(path,backup_path(path));path.write_bytes(data)
    for path,data in destinations:
        path.parent.mkdir(parents=True,exist_ok=True)
        if not path.exists():path.write_bytes(data)
    return sum(raw!=data for _,raw,data in changes)

def retire_install(ksp_dir,ros2_ws,apply=False):
    candidates=[]
    if ksp_dir:
        old=Path(ksp_dir)/'GameData/KerbalLiDAR'
        if old.exists():
            marker=old/'Parts/Lidar2D/part.cfg'
            if not marker.is_file() or 'kerbal_lidar_2d' not in marker.read_text():raise ValueError('Old KSP installation is not recognized; left untouched')
            candidates.append(old)
    if ros2_ws:
        ws=Path(ros2_ws)
        for name in OLD_PACKAGES:
            package=ws/'src'/name
            xml=package/'package.xml'
            installed=ws/'install'/name/'share'/name/'package.xml'
            owned=package.exists() or installed.is_file()
            if package.exists():
                if not xml.is_file() or ET.parse(xml).getroot().findtext('name')!=name:raise ValueError(f'Unrecognized ROS source: {package}')
                candidates.append(package)
            if installed.is_file() and ET.parse(installed).getroot().findtext('name')!=name:
                raise ValueError(f'Unrecognized installed ROS package: {installed}')
            merged=ws/'install/share'/name/'package.xml'
            if merged.exists():raise ValueError(f'Merged install requires a separate clean install prefix: {merged}')
            for area in ('build','install'):
                artifact=ws/area/name
                if artifact.exists():
                    if not owned:raise ValueError(f'Cannot identify orphaned ROS artifact: {artifact}')
                    candidates.append(artifact)
    # Retired directories must leave src/GameData entirely, not just change name.
    for path in candidates:
        print(('RETIRE ' if apply else 'WOULD RETIRE ')+str(path))
        if apply:
            if 'GameData'==path.parent.name:store=path.parent.parent/'PyLoN-migration-backups'
            else:store=path.parent.parent/'pylon-migration-backups'/path.parent.name
            store.mkdir(parents=True,exist_ok=True)
            target=store/backup_path(path).name;shutil.move(str(path),str(target))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',nargs='?');mode=parser.add_mutually_exclusive_group();mode.add_argument('--output');mode.add_argument('--in-place',action='store_true')
    parser.add_argument('--retire-install',action='store_true');parser.add_argument('--apply',action='store_true');parser.add_argument('--ksp-dir');parser.add_argument('--ros2-ws')
    args=parser.parse_args()
    try:
        if args.retire_install:retire_install(args.ksp_dir,args.ros2_ws,args.apply)
        elif args.source:migrate(args.source,args.output,args.in_place)
        else:parser.error('source or --retire-install is required')
    except (ValueError,OSError,ET.ParseError) as exc:parser.exit(2,str(exc)+'\n')
if __name__=='__main__':main()
