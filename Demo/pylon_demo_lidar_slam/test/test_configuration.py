"""Check the restored launch assets and their package/frame contracts."""
import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]


class ConfigurationTests(unittest.TestCase):
    def test_sources_parse_and_use_current_public_packages(self):
        for path in ROOT.rglob('*.py'):
            if '__pycache__' in path.parts:
                continue
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    self.assertFalse((node.module or '').startswith('ksp_'), path)

    def test_costmaps_amcl_and_odometry_share_estimated_frames(self):
        config = yaml.safe_load((ROOT / 'params/nav2_params.yaml').read_text())
        odom = config['laser_scan_odometry']['ros__parameters']
        amcl = config['amcl']['ros__parameters']
        controller = config['planar_wrench_controller']['ros__parameters']
        self.assertEqual(amcl['base_frame_id'], odom['base_frame'])
        self.assertEqual(amcl['odom_frame_id'], odom['odom_frame'])
        self.assertEqual(controller['odom_base_frame'], odom['base_frame'])
        self.assertEqual(controller['odom_topic'], odom['odom_topic'])
        for name in ('local_costmap', 'global_costmap'):
            params = config[name][name]['ros__parameters']
            self.assertEqual(params['robot_base_frame'], odom['base_frame'])
            self.assertEqual(params['obstacle_layer']['scan']['topic'], odom['filtered_scan_topic'])

    def test_slam_uses_same_frame_chain_as_amcl(self):
        config = yaml.safe_load((ROOT / 'params/nav2_params.yaml').read_text())
        slam = yaml.safe_load((ROOT / 'params/slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']
        amcl = config['amcl']['ros__parameters']
        self.assertEqual(slam['base_frame'], amcl['base_frame_id'])
        self.assertEqual(slam['odom_frame'], amcl['odom_frame_id'])
        self.assertEqual(slam['map_frame'], amcl['global_frame_id'])

    def test_nav2_uses_twist_and_no_lateral_velocity_for_default_rover(self):
        config = yaml.safe_load((ROOT / 'params/nav2_params.yaml').read_text())
        for name in ('controller_server', 'behavior_server'):
            self.assertFalse(config[name]['ros__parameters']['enable_stamped_cmd_vel'])
        path = config['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(path['min_vel_y'], 0.0)
        self.assertEqual(path['max_vel_y'], 0.0)

    def test_launch_packages_have_runtime_dependencies(self):
        manifest = ET.parse(ROOT / 'package.xml').getroot()
        package = manifest.findtext('name')
        dependencies = {item.text for item in manifest.findall('exec_depend')}
        files = list((ROOT / 'launch').glob('*.py')) + [ROOT / package / 'launch_support.py']
        for path in files:
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.Call):
                    for keyword in node.keywords:
                        if keyword.arg == 'package' and isinstance(keyword.value, ast.Constant):
                            target = keyword.value.value
                            self.assertTrue(target == package or target in dependencies, (path, target))
        self.assertTrue((ROOT / 'resource' / package).exists())
        for dependency in ('pylon_interfaces', 'pylon_vehicle_control', 'slam_toolbox'):
            self.assertIn(dependency, dependencies)
