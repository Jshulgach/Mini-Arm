"""Read-only structural audit. Does not import drivers or connect to hardware."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def audit(root):
    description = root / "ros2/miniarm_description"
    config = root / "ros2/miniarm_moveit_config/config"
    robot = ET.parse(description / "urdf/miniarm.urdf").getroot()
    semantic = ET.parse(config / "miniarm.srdf").getroot()
    control = ET.parse(config / "miniarm.ros2_control.xacro").getroot()
    links = {n.attrib["name"] for n in robot.findall("link")}
    joints = {n.attrib["name"]: n for n in robot.findall("joint")}
    mesh_names = {n.attrib["filename"] for n in robot.iter("mesh")}
    missing_meshes = sorted(n for n in mesh_names if not (
        description / n.removeprefix("package://miniarm_description/")).is_file())
    semantic_links = set()
    for n in semantic.iter():
        for key in ("base_link", "tip_link", "child_link", "parent_link", "link1", "link2"):
            if key in n.attrib:
                semantic_links.add(n.attrib[key])
    invalid_states = []
    for state in semantic.findall("group_state"):
        for n in state.findall("joint"):
            joint = joints.get(n.attrib["name"])
            if joint is None:
                continue
            limit = joint.find("limit")
            if limit is not None and "lower" in limit.attrib:
                value = float(n.attrib["value"])
                if not float(limit.attrib["lower"]) <= value <= float(limit.attrib["upper"]):
                    invalid_states.append({"state": state.attrib["name"],
                                           "joint": n.attrib["name"], "value": value})
    return {
        "scope": "Static XML and file-reference checks only; no hardware or ROS runtime validation",
        "links": len(links), "joints": len(joints),
        "missing_meshes": missing_meshes,
        "semantic_links_missing_from_urdf": sorted(semantic_links - links),
        "control_joints_missing_from_urdf": sorted({n.attrib["joint_name"] for n in control.iter()
            if n.tag.endswith("}miniarm_mock_joint")} - joints.keys()),
        "semantic_states_outside_urdf_limits": invalid_states,
        "hardware_plugins": [n.text for n in control.iter("plugin")],
        "links_with_inertial": sum(n.find("inertial") is not None for n in robot.findall("link")),
        "camera_links": sorted(n for n in links if "camera" in n or "optical" in n),
    }


if __name__ == "__main__":
    print(json.dumps(audit(Path(__file__).resolve().parents[2]), indent=2))
