"""File and configuration contracts; no ROS nodes or hardware are started."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'ros2/miniarm_moveit_config/config'
DESCRIPTION = ROOT / 'ros2/miniarm_description'
MODEL = ET.parse(DESCRIPTION / 'urdf/miniarm.urdf').getroot()
JOINTS = {j.attrib['name']: j for j in MODEL.findall('joint')}
LINKS = {link.attrib['name'] for link in MODEL.findall('link')}
ACTUATED = {name for name, joint in JOINTS.items() if joint.attrib['type'] != 'fixed' and joint.find('mimic') is None}


def load(name):
    return yaml.safe_load((CONFIG / name).read_text())


def test_meshes_exist_and_have_meter_scale():
    for mesh in MODEL.iter('mesh'):
        assert mesh.attrib['scale'] == '0.001 0.001 0.001'
        prefix = 'package://miniarm_description/'
        filename = mesh.attrib['filename']
        assert filename.startswith(prefix)
        path = DESCRIPTION / filename[len(prefix):]
        data = path.read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        assert len(data) == 84 + count * 50
        values = [struct.unpack_from('<3f', data, 84 + i * 50 + 12 + k * 12) for i in range(count) for k in range(3)]
        size = max(max(v[k] for v in values) - min(v[k] for v in values) for k in range(3)) * .001
        assert .001 < size < .3, (path.name, size)


def test_semantic_chain_states_groups_and_collision_exclusions():
    semantic = ET.parse(CONFIG / 'miniarm.srdf').getroot()
    groups = {g.attrib['name'] for g in semantic.findall('group')}
    for node in semantic.iter():
        for attr in ('base_link', 'tip_link', 'child_link', 'parent_link', 'link1', 'link2'):
            if attr in node.attrib:
                assert node.attrib[attr] in LINKS
        if node.tag == 'link':
            assert node.attrib['name'] in LINKS
        if node.tag == 'joint':
            assert node.attrib['name'] in JOINTS
        if 'group' in node.attrib:
            assert node.attrib['group'] in groups
    chain = semantic.find("group[@name='miniarm_arm']/chain")
    child_joints = {j.find('child').attrib['link']: j for j in JOINTS.values()}
    tip = chain.attrib['tip_link']
    chain_joints = set()
    while tip != chain.attrib['base_link']:
        joint = child_joints[tip]
        chain_joints.add(joint.attrib['name'])
        tip = joint.find('parent').attrib['link']
    assert chain_joints == ACTUATED - {'miniarm_gripper_joint'}
    for state in semantic.findall('group_state'):
        assert {j.attrib['name'] for j in state} == chain_joints
        for joint in state:
            bounds = JOINTS[joint.attrib['name']].find('limit').attrib
            assert float(bounds['lower']) <= float(joint.attrib['value']) <= float(bounds['upper'])
    edges = {frozenset((j.find('parent').attrib['link'], j.find('child').attrib['link'])) for j in JOINTS.values()}
    for pair in semantic.findall('disable_collisions'):
        assert pair.attrib['reason'] == 'Adjacent'
        assert frozenset((pair.attrib['link1'], pair.attrib['link2'])) in edges


def test_controller_joint_sets_and_initial_values_agree():
    positions = load('initial_positions.yaml')['initial_positions']
    limits = load('joint_limits.yaml')['joint_limits']
    controllers = load('miniarm_ros_controllers.yaml')
    controlled = set(controllers['miniarm_controller']['ros__parameters']['joints'])
    controlled.add(controllers['miniarm_hand_controller']['ros__parameters']['joint'])
    assert set(positions) == set(limits) == controlled == ACTUATED
    for name, value in positions.items():
        bounds = JOINTS[name].find('limit').attrib
        assert float(bounds['lower']) <= value <= float(bounds['upper'])
        assert limits[name]['max_velocity'] <= float(bounds['velocity'])
    control = ET.parse(CONFIG / 'miniarm.ros2_control.xacro').getroot()
    assert [p.text for p in control.iter('plugin')] == ['mock_components/GenericSystem']
    declarations = {n.attrib['joint_name'] for n in control.iter() if n.tag.endswith('}miniarm_mock_joint')}
    assert declarations == ACTUATED
    assert 'miniarm_arm' in load('kinematics.yaml')
