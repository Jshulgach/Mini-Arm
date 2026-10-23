"""Exercise actual firmware methods using desktop numpy and mocked servo writes."""
from pathlib import Path
import sys
import math
from unittest.mock import Mock
import ast
import asyncio
import time
import json
from types import SimpleNamespace
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'firmware/circuitpython/lib'))
sys.path.insert(0, str(ROOT / 'ros2/miniarm_core'))
from arm_control.robotarm import RobotArm
from arm_control.base_calibration import BaseCalibration
from arm_utils.robot_models import miniarm_params
from arm_utils.armTransforms import Angle
from miniarm_core.client import MiniArmClient


def test_scalar_euler_conversion_does_not_require_numpy_2_aliases(monkeypatch):
    from arm_utils import armTransforms
    monkeypatch.delattr(armTransforms.np, 'asin', raising=False)
    result = armTransforms.rotationMatrixToEulerAngles(armTransforms.np.eye(3))
    assert [angle.rad for angle in result] == [0, 0, 0]


@pytest.fixture
def arm():
    robot = RobotArm(simulate_hardware=True, dh_params=miniarm_params)
    robot.forward_kinematics()
    robot.simulate_hardware = False
    robot.update_servo = Mock()
    return robot


@pytest.mark.parametrize('targets', [[0]*5, [0]*7, [0, 0, 0, 0, 0, 200], [float('nan')]*6, [float('inf')]*6])
def test_invalid_batch_has_no_partial_writes_or_state_change(arm, targets):
    before = arm.get_joints()
    with pytest.raises(ValueError):
        arm.set_joints(targets)
    arm.update_servo.assert_not_called()
    assert arm.get_joints() == before


@pytest.mark.parametrize('joint,channel', [('base',15), ('shoulder',14), ('elbow',13), ('wrist-rotation',12), ('wrist-bend',2), ('gripper-link',1), ('gripper',0)])
def test_raw_joint_writer_selects_only_requested_channel_without_pose_change(arm, joint, channel):
    before = arm.get_joints(), arm.get_gripper()
    for angle in (0, 75, 180, None):
        arm.calibration_joint_write(joint, angle)
        arm.update_servo.assert_called_with(channel, angle)
        assert (arm.get_joints(), arm.get_gripper()) == before
    arm.update_servo.reset_mock()
    for angle in (-1, 181, True, float('nan'), float('inf'), '90'):
        with pytest.raises(ValueError): arm.calibration_joint_write(joint, angle)
    arm.update_servo.assert_not_called()


def test_raw_joint_writer_rejects_unknown_or_changed_mapping(arm, monkeypatch):
    with pytest.raises(ValueError): arm.calibration_joint_write('neck', 75)
    monkeypatch.setattr('arm_control.robotarm.WRIST_BEND_SERVO', 3)
    with pytest.raises(ValueError): arm.calibration_joint_write('wrist-bend', 75)
    arm.update_servo.assert_not_called()


def test_signed_joint_commands_apply_offsets_once(arm):
    target = [10, -20, 30, -15, 25, -10]
    arm.set_joints(target)
    assert [call.args for call in arm.update_servo.call_args_list] == [(15, 80), (14, 110), (13, 75), (12, 75), (2, 85), (1, 80)]
    assert arm.get_joints() == pytest.approx(target)


def test_interpolation_reaches_signed_endpoint_without_double_flip(arm, monkeypatch):
    monkeypatch.setattr('arm_control.robotarm.time.sleep', lambda _: None)
    arm.set_joints([-20, 20, -15, 10, 0, 10])
    arm.update_servo.reset_mock()
    target = [10, -20, 30, -15, 25, -10]
    arm.set_joints_speed_control(target, desired_speed=10)
    assert arm.get_joints() == pytest.approx(target)
    assert len(arm.update_servo.call_args_list) == 90
    assert all(0 <= call.args[1] <= 180 for call in arm.update_servo.call_args_list)


def test_rejected_driver_write_does_not_report_joint_change(arm):
    arm.update_servo.side_effect = ValueError('driver rejected target')
    before = arm.get_joints()
    with pytest.raises(ValueError):
        arm.set_joint(0, 10)
    assert arm.get_joints() == before


@pytest.mark.parametrize('index', [-1, 6, 7, True, .5])
def test_invalid_joint_index_is_rejected(arm, index):
    with pytest.raises(ValueError):
        arm.set_joint(index, 10)
    arm.update_servo.assert_not_called()


def test_gripper_named_numeric_and_invalid_values(arm):
    for value, expected in [('open', 120), ('close', 0), ('45.5', 45.5)]:
        arm.set_gripper(value)
        arm.update_servo.assert_called_with(0, expected)
        assert arm.get_gripper() == expected
    arm.update_servo.reset_mock()
    for value in [-1, 181, float('nan'), 'unknown']:
        with pytest.raises(ValueError):
            arm.set_gripper(value)
    arm.update_servo.assert_not_called()


def test_rejected_ik_cannot_update_pose_or_gripper(arm):
    before = arm.get_joints()
    arm.inverse_kinematics = Mock(return_value=[Angle(200, 'deg')]*6)
    with pytest.raises(ValueError):
        arm.set_pose([.135, 0, .22, 0, 0, 0, 90])
    arm.update_servo.assert_not_called()
    assert arm.get_joints() == before


def test_client_converts_radians_and_rejects_invalid_values_before_send():
    # Bypass connection construction entirely.
    client = MiniArmClient.__new__(MiniArmClient)
    client.verbose = False
    client.s = None
    client.send_message = Mock()
    client.set_pose(.135, 0, .22, roll=math.pi/2)
    assert client.send_message.call_args.args[0] == 'set_pose:[0.135,0,0.22,90.0,0.0,0.0]'
    client.set_delta_pose(dyaw=-math.pi/2)
    assert client.send_message.call_args.args[0].endswith(',0.0,0.0,-90.0]')
    client.send_message.reset_mock()
    for action in [lambda: client.set_pose(99, 0, .22), lambda: client.set_pose(.135, 0, float('nan')), lambda: client.set_joints([0]*5), lambda: client.set_joint(6, 90)]:
        with pytest.raises(ValueError):
            action()
    client.send_message.assert_not_called()


def controller_type():
    # Compile the real class without importing board drivers or module-level demos.
    path = ROOT / 'firmware/circuitpython/lib/arm_control/async_controller.py'
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'AsyncController')
    namespace = {'RobotArm': Mock(), 'robot_models': SimpleNamespace(miniarm_params=miniarm_params),
                 'RGBLED': Mock(), 'MusicPlayer': Mock(), 'ForceSensor': Mock(),
                 'board': SimpleNamespace(A0=0, A2=2), 'USBSerialReader': Mock(),
                 'time': time, 'asyncio': asyncio, 'json': json,
                 'BaseCalibration': BaseCalibration}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['AsyncController'], namespace


def test_boot_does_not_home_and_serial_defaults_match_client():
    controller, context = controller_type()
    instance = controller(use_serial=True)
    instance.robot.home.assert_not_called()
    assert context['USBSerialReader'].call_args.kwargs['TERMINATOR'] == ';'
    controller(home_on_start=True).robot.home.assert_called_once()


def test_parser_rejects_bad_optional_gripper_before_arm_writes(arm):
    controller, _ = controller_type()
    instance = controller()
    instance.robot = arm
    asyncio.run(instance.movemotors(['movemotors', '[0,0,0,0,0,0,200]']))
    arm.update_servo.assert_not_called()
    asyncio.run(instance.parse_command([['set_gripper', 'open']]))
    arm.update_servo.assert_called_once_with(0, 120)


def test_hardware_boot_clears_retained_pwm_on_all_configured_channels(monkeypatch):
    outputs = [SimpleNamespace(angle=90) for _ in range(16)]
    kit = SimpleNamespace(servo=outputs)
    monkeypatch.setitem(sys.modules, 'board', SimpleNamespace(GP1=1, GP0=0))
    monkeypatch.setitem(sys.modules, 'busio', SimpleNamespace(I2C=Mock()))
    monkeypatch.setitem(sys.modules, 'adafruit_servokit', SimpleNamespace(ServoKit=Mock(return_value=kit)))
    RobotArm(simulate_hardware=False, dh_params=miniarm_params)
    wired_channels = {15, 14, 13, 12, 2, 1, 0}
    assert all(outputs[i].angle is None for i in wired_channels)
    assert all(outputs[i].angle == 90 for i in set(range(16)) - wired_channels)


def test_shoulder_raw_calibration_writes_channel_14_only_and_rejects_bad_values(arm):
    arm.calibration_shoulder_write(60)
    arm.update_servo.assert_called_once_with(14, 60)
    arm.calibration_shoulder_write(None)
    arm.update_servo.assert_called_with(14, None)
    arm.update_servo.reset_mock()
    for value in (True, -1, 181, float('nan'), '90'):
        with pytest.raises(ValueError): arm.calibration_shoulder_write(value)
    arm.update_servo.assert_not_called()
