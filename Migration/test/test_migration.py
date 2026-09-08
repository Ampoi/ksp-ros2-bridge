import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('pylon_migration',Path(__file__).parents[1]/'pylon_migrate.py')
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)

CRAFT='''ship = KerbalLiDAR is my ship name
PART
{
 part = kerbal.lidar.3d_42
 persistentId = 42
 MODULE
 {
  name = ModuleKerbalLidar
  partName = front_sensor
  udpPort = 49010
 }
 MODULE
 {
  name = AnotherMod
  partName = do not change
  text = "KerbalLiDAR/{value}"
 }
}
'''

class MigrationTests(unittest.TestCase):
 def test_structure_ids_and_other_mod_preserved(self):
  out,settings,_=m.transform(CRAFT)
  self.assertIn('part = pylon.lidar.3d_42',out)
  self.assertIn('name = ModulePyLoNLidar',out)
  self.assertIn('sensorId = front_sensor',out)
  self.assertIn('persistentId = 42',out)
  self.assertIn('ship = KerbalLiDAR is my ship name',out)
  self.assertIn('partName = do not change',out)
  self.assertIn('"KerbalLiDAR/{value}"',out)
  self.assertEqual(settings,{'statePort':'49010'})
  self.assertEqual(m.transform(out)[0],out)
 def test_conflicting_sensor_and_endpoint_values(self):
  second='\n MODULE\n {\n name = ModuleKerbalRosSensorId\n sensorId = other\n }\n'
  conflict=CRAFT[:CRAFT.rfind('}')]+second+'}\n'
  out,_,_=m.transform(conflict)
  self.assertIn('sensorId = other',out)
  self.assertNotIn('sensorId = front_sensor',out)
  duplicate=conflict[:conflict.rfind('}')]+second+'}\n'
  with self.assertRaisesRegex(ValueError,'Multiple Sensor ID'):m.transform(duplicate)
  with self.assertRaisesRegex(ValueError,'setting'):m.transform(CRAFT+CRAFT.replace('49010','49012'))
 def test_malformed_input_is_not_partially_written(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);source=root/'input';source.mkdir();(source/'good.craft').write_text(CRAFT);(source/'bad.sfs').write_text('GAME {')
   with self.assertRaises(ValueError):m.migrate(source,root/'output')
   self.assertFalse((root/'output').exists())
 def test_preview_output_and_idempotent_rerun(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);source=root/'input.craft';source.write_text(CRAFT)
   self.assertEqual(m.migrate(source),1);self.assertEqual(source.read_text(),CRAFT)
   m.migrate(source,root/'output');m.migrate(source,root/'output')
   self.assertTrue((root/'output/Runtime.cfg').exists())
   self.assertEqual(source.read_text(),CRAFT)
 def test_in_place_backup_is_restorable(self):
  with tempfile.TemporaryDirectory() as directory:
   path=Path(directory)/'ship.craft';text=CRAFT.replace('  udpPort = 49010\n','');path.write_text(text)
   m.migrate(path,in_place=True);backups=list(path.parent.glob('*.pre-pylon-*'));self.assertEqual(len(backups),1)
   self.assertEqual(backups[0].read_text(),text)
   m.migrate(path,in_place=True);self.assertEqual(len(list(path.parent.glob('*.pre-pylon-*'))),1)
   path.write_bytes(backups[0].read_bytes());self.assertEqual(path.read_text(),text)
 def test_install_retirement_leaves_other_projects(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);old=root/'src/ksp_lidar_bridge';old.mkdir(parents=True);(old/'package.xml').write_text('<package><name>ksp_lidar_bridge</name></package>')
   other=root/'src/other';other.mkdir();(other/'note').write_text('keep')
   m.retire_install(None,root,False);self.assertTrue(old.exists())
   m.retire_install(None,root,True);self.assertFalse(old.exists());self.assertTrue((other/'note').exists())
   self.assertEqual(len(list((root/'pylon-migration-backups/src').iterdir())),1)

 def test_common_config_conflict_preflight(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);source=root/'input';source.mkdir()
   (source/'ship.craft').write_text(CRAFT)
   (source/'Runtime.cfg').write_text('PYLON_TRANSPORT\n{\n statePort = 1234\n}\n')
   with self.assertRaisesRegex(ValueError,'Conflicting generated'):m.migrate(source,root/'output')
   self.assertFalse((root/'output').exists())
 def test_orphan_install_is_retired_only_with_ownership_evidence(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);installed=root/'install/ksp_lidar_bridge/share/ksp_lidar_bridge';installed.mkdir(parents=True)
   (installed/'package.xml').write_text('<package><name>ksp_lidar_bridge</name></package>')
   m.retire_install(None,root,True)
   self.assertFalse((root/'install/ksp_lidar_bridge').exists())
   orphan=root/'build/ksp_lidar_bridge';orphan.mkdir(parents=True)
   with self.assertRaisesRegex(ValueError,'Cannot identify'):m.retire_install(None,root,True)
   self.assertTrue(orphan.exists())

 def test_craft_attachment_and_symmetry_references_follow_renamed_parts(self):
  text=CRAFT.replace(' persistentId = 42',' link = kerbal.lidar.2d_7\n sym = kerbal.lidar.3d_42\n attN = top, kerbal.lidar.2d_7\n srfN = srfAttach,kerbal.lidar.3d_42\n parent = 5\n persistentId = 42')
  out,_,_=m.transform(text)
  self.assertIn('link = pylon.lidar.2d_7',out)
  self.assertIn('sym = pylon.lidar.3d_42',out)
  self.assertIn('attN = top, pylon.lidar.2d_7',out)
  self.assertIn('srfN = srfAttach,pylon.lidar.3d_42',out)
  self.assertIn('parent = 5',out)

 def test_retired_visual_fields_do_not_damage_adjacent_settings(self):
  text=CRAFT.replace('  partName = front_sensor','  visualModelObjPath = GameData/KerbalLiDAR/Models/Lidar3D/meshy_lidar_3d.obj\n  visualModelTexturePath = KerbalLiDAR/Models/Lidar3D/meshy_lidar_3d.png\n  scanRateHz = 17\n  partName = front_sensor')
  out,_,_=m.transform(text)
  self.assertNotIn('visualModel',out)
  self.assertIn('scanRateHz = 17',out)
  self.assertIn('sensorId = front_sensor',out)
  self.assertIn('partName = do not change',out)
  m.parse(out)
