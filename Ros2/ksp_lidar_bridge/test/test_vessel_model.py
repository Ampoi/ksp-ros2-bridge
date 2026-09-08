import base64
import gzip
import hashlib
import json
import unittest

from ksp_lidar_bridge.vessel_model import UrdfChunkAssembler


SESSION_ID = "a" * 32
VESSEL_ID = "b" * 32
NAME_PREFIX = "ksp_" + VESSEL_ID[:8]


def proxy_urdf(geometry='<box size="1 2 3"/>'):
    return f'''<?xml version="1.0"?>
<robot name="{NAME_PREFIX}_active_vessel">
  <link name="{NAME_PREFIX}_link_0000">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry>{geometry}</geometry></visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="1 2 3"/></geometry>
    </collision>
  </link>
  <link name="{NAME_PREFIX}_link_0001">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="{NAME_PREFIX}_joint_0001" type="fixed">
    <parent link="{NAME_PREFIX}_link_0000"/>
    <child link="{NAME_PREFIX}_link_0001"/>
    <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>
  </joint>
</robot>'''


def chunk_packets(urdf=None, chunk_size=40, include_base_pose=True):
    urdf = proxy_urdf() if urdf is None else urdf
    model_id = hashlib.sha256(urdf.encode()).hexdigest()
    bundle = {
        "type": "ksp_active_vessel_proxy",
        "version": 1,
        "sessionId": SESSION_ID,
        "vesselId": VESSEL_ID,
        "modelId": model_id,
        "geometryPolicy": "primitive_proxy_only",
        "persistencePolicy": "memory_only",
        "rootFrame": f"{NAME_PREFIX}_link_0000",
        "urdf": urdf,
        "partFrames": [
            {"partFlightId": 10, "frame": f"{NAME_PREFIX}_link_0000"},
            {"partFlightId": 11, "frame": f"{NAME_PREFIX}_link_0001"},
        ],
    }
    if include_base_pose:
        bundle["baseToRootPosition"] = [1.25, -2.5, 3.75]
        bundle["baseToRootRotation"] = [0.0, 0.0, 2.0, 2.0]
    compressed = gzip.compress(json.dumps(bundle, separators=(",", ":")).encode())
    digest = hashlib.sha256(compressed).hexdigest()
    chunks = [
        compressed[index:index + chunk_size]
        for index in range(0, len(compressed), chunk_size)
    ]
    return [
        {
            "type": "ksp_vessel_urdf_chunk",
            "version": 1,
            "sessionId": SESSION_ID,
            "modelId": model_id,
            "chunkIndex": index,
            "chunkCount": len(chunks),
            "encoding": "gzip+base64",
            "sha256": digest,
            "expiresAfterSec": 6,
            "data": base64.b64encode(chunk).decode(),
        }
        for index, chunk in enumerate(chunks)
    ]


class UrdfChunkAssemblerTests(unittest.TestCase):
    def test_reassembles_out_of_order_proxy_without_disk_state(self):
        assembler = UrdfChunkAssembler()
        packets = chunk_packets()
        model = None
        for packet in reversed(packets):
            candidate = assembler.consume(packet, received_at=1.0)
            if candidate is not None:
                model = candidate

        self.assertIsNotNone(model)
        self.assertEqual(model.vessel_id, VESSEL_ID)
        self.assertEqual(model.root_frame, f"{NAME_PREFIX}_link_0000")
        self.assertEqual(model.base_to_root_translation, (1.25, -2.5, 3.75))
        self.assertAlmostEqual(model.base_to_root_rotation[2], 2 ** -0.5)
        self.assertAlmostEqual(model.base_to_root_rotation[3], 2 ** -0.5)
        self.assertEqual(model.part_frames[11], f"{NAME_PREFIX}_link_0001")
        self.assertEqual(len(model.transforms), 1)
        transform = model.transforms[0]
        self.assertEqual(transform.translation, (1.0, 2.0, 3.0))
        self.assertAlmostEqual(transform.rotation[2], 2 ** -0.5)
        self.assertAlmostEqual(transform.rotation[3], 2 ** -0.5)

    def test_legacy_v1_proxy_defaults_to_identity_base_pose(self):
        assembler = UrdfChunkAssembler()
        model = None
        for packet in chunk_packets(include_base_pose=False):
            candidate = assembler.consume(packet)
            if candidate is not None:
                model = candidate

        self.assertIsNotNone(model)
        self.assertEqual(model.base_to_root_translation, (0.0, 0.0, 0.0))
        self.assertEqual(model.base_to_root_rotation, (0.0, 0.0, 0.0, 1.0))

    def test_rejects_mesh_geometry_and_asset_uri(self):
        packets = chunk_packets(proxy_urdf('<mesh filename="GameData/Squad/model.mu"/>'))
        assembler = UrdfChunkAssembler()
        with self.assertRaisesRegex(ValueError, "not allowed"):
            for packet in packets:
                assembler.consume(packet)

    def test_accepts_bounded_cylinder_and_sphere_primitives(self):
        urdf = proxy_urdf('<cylinder radius="0.5" length="2"/>')
        urdf = urdf.replace(
            '    <collision>\n      <origin xyz="0 0 0" rpy="0 0 0"/>',
            '    <visual><origin xyz="0 0 1" rpy="0 0 0"/>'
            '<geometry><sphere radius="0.5"/></geometry></visual>\n'
            '    <collision>\n      <origin xyz="0 0 0" rpy="0 0 0"/>',
            1,
        )
        model = None
        assembler = UrdfChunkAssembler()
        for packet in chunk_packets(urdf):
            candidate = assembler.consume(packet)
            if candidate is not None:
                model = candidate
        self.assertIsNotNone(model)

    def test_refresh_with_same_parts_updates_geometry_and_joint_pose(self):
        assembler = UrdfChunkAssembler()
        models = []
        original = proxy_urdf()
        deployed = original.replace('<box size="1 2 3"/>', '<box size="1 20 3"/>')
        deployed = deployed.replace('xyz="1 2 3"', 'xyz="4 5 6"')
        for urdf in (original, deployed):
            model = None
            for packet in chunk_packets(urdf):
                candidate = assembler.consume(packet)
                if candidate is not None:
                    model = candidate
            self.assertIsNotNone(model)
            models.append(model)
        self.assertEqual(models[0].part_frames, models[1].part_frames)
        self.assertEqual(models[0].root_frame, models[1].root_frame)
        self.assertNotEqual(models[0].model_id, models[1].model_id)
        self.assertEqual(models[1].transforms[0].translation, (4.0, 5.0, 6.0))
        self.assertIn('<box size="1 20 3"/>', models[1].urdf)

    def test_accepts_full_primitive_budget_for_complex_part(self):
        import xml.etree.ElementTree as ET

        robot = ET.fromstring(proxy_urdf())
        link = robot.find('link')
        for index in range(47):
            for kind in ('visual', 'collision'):
                owner = ET.SubElement(link, kind)
                ET.SubElement(owner, 'origin', xyz=f'{index} 0 0', rpy='0 0 0')
                geometry = ET.SubElement(owner, 'geometry')
                ET.SubElement(geometry, 'box', size='1 2 3')
        assembler = UrdfChunkAssembler()
        model = None
        for packet in chunk_packets(ET.tostring(robot, encoding='unicode')):
            candidate = assembler.consume(packet)
            if candidate is not None:
                model = candidate
        self.assertIsNotNone(model)
        received_link = ET.fromstring(model.urdf).find('link')
        self.assertEqual(len(received_link.findall('visual')), 48)
        self.assertEqual(len(received_link.findall('collision')), 48)

    def test_rejects_non_positive_cylinder_dimension(self):
        packets = chunk_packets(proxy_urdf('<cylinder radius="0" length="2"/>'))
        assembler = UrdfChunkAssembler()
        with self.assertRaisesRegex(ValueError, "radius must be positive"):
            for packet in packets:
                assembler.consume(packet)

    def test_rejects_checksum_mismatch(self):
        packets = chunk_packets()
        packets[-1]["data"] = base64.b64encode(b"tampered").decode()
        assembler = UrdfChunkAssembler()
        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            for packet in packets:
                assembler.consume(packet)

    def test_rejects_gzip_expansion_over_limit(self):
        packets = chunk_packets(chunk_size=100000)
        assembler = UrdfChunkAssembler(max_uncompressed_bytes=100)
        with self.assertRaisesRegex(ValueError, "exceeds size limit"):
            assembler.consume(packets[0])

    def test_rejects_remote_policy_bundle(self):
        packets = chunk_packets()
        compressed = b"".join(base64.b64decode(packet["data"]) for packet in packets)
        bundle = json.loads(gzip.decompress(compressed))
        bundle["persistencePolicy"] = "write_to_disk"
        replacement = gzip.compress(json.dumps(bundle).encode())
        digest = hashlib.sha256(replacement).hexdigest()
        packet = dict(packets[0])
        packet.update(
            chunkIndex=0,
            chunkCount=1,
            sha256=digest,
            data=base64.b64encode(replacement).decode(),
        )
        with self.assertRaisesRegex(ValueError, "memory-only"):
            UrdfChunkAssembler().consume(packet)


if __name__ == "__main__":
    unittest.main()
