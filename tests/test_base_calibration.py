"""Exercise the real pulse limiter with a fake clock and servo bus; no hardware."""
import asyncio
import json
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from test_firmware_commands import controller_type
from arm_control.base_calibration import BaseCalibration


@pytest.fixture
def setup():
    now = [10.0]
    outputs = [None] * 7
    def write(value):
        outputs[0] = value
    robot = SimpleNamespace(simulate_hardware=False,
                            calibration_outputs=lambda: list(outputs),
                            calibration_base_write=Mock(side_effect=write))
    return BaseCalibration(robot, clock=lambda: now[0]), robot, outputs, now


def test_pulse_releases_on_controller_deadline_without_usb(setup):
    session, robot, outputs, now = setup
    session.begin('abc', 90, None)
    assert outputs == [90] + [None]*6
    now[0] += .399
    session.tick()
    assert session.active
    now[0] += .002
    session.tick()
    assert not session.active
    assert outputs == [None]*7
    assert session.last == 90 and session.center == 90 and session.locked
    assert [call.args for call in robot.calibration_base_write.call_args_list] == [(90,), (None,)]


@pytest.mark.parametrize('angle', [-1, 181, True, float('nan'), float('inf'), '90'])
def test_invalid_target_never_writes(setup, angle):
    session, robot, _, _ = setup
    with pytest.raises(ValueError): session.begin('abc', angle, None)
    robot.calibration_base_write.assert_not_called()


def test_any_active_output_including_gripper_prevents_base_test(setup):
    session, robot, outputs, _ = setup
    outputs[6] = 0
    with pytest.raises(ValueError, match='seven outputs'): session.begin('abc', 90, None)
    robot.calibration_base_write.assert_not_called()


def test_no_extension_retry_or_immediate_repeat(setup):
    session, robot, _, now = setup
    session.begin('abc', 90, None)
    deadline = session.deadline
    with pytest.raises(ValueError): session.begin('def', 90, 90)
    assert session.deadline == deadline
    now[0] = deadline + .001; session.tick()
    with pytest.raises(ValueError, match='Duplicate'): session.begin('abc', 90, 90)
    with pytest.raises(ValueError, match='one second'): session.begin('def', 90, 90)
    assert robot.calibration_base_write.call_count == 2


def test_previous_target_step_and_initial_window_are_enforced(setup):
    session, robot, _, now = setup
    session.begin('a', 90, None); session.release(); now[0] += 2
    with pytest.raises(ValueError, match='Previous'): session.begin('b', 90, None)
    with pytest.raises(ValueError, match='one degree'): session.begin('b', 92, 90)
    for index in range(1, 6):
        session.begin(str(index), 90+index, 89+index)
        session.release(); now[0] += 2
    with pytest.raises(ValueError, match='five degrees'): session.begin('f', 96, 95)
    assert session.center == 90


def test_failed_initial_write_attempts_release_and_latches_fault(setup):
    session, robot, _, _ = setup
    robot.calibration_base_write.side_effect = [OSError('I2C error'), None]
    with pytest.raises(OSError): session.begin('a', 90, None)
    assert robot.calibration_base_write.call_args.args == (None,)
    assert session.fault and session.locked and not session.active
    assert session.last is None


def test_release_failure_is_never_reported_as_success(setup):
    session, robot, _, now = setup
    session.begin('a', 90, None)
    robot.calibration_base_write.side_effect = OSError('I2C error')
    now[0] += 1; session.tick()
    assert 'Disconnect servo power' in session.fault
    with pytest.raises(ValueError, match='faulted'): session.begin('b', 90, 90)


def test_probe_is_read_only_and_legacy_motion_is_blocked_after_pulse(capsys):
    controller, _ = controller_type()
    instance = controller()
    instance.robot.calibration_outputs.return_value = [None]*7
    asyncio.run(instance.parse_command([['cal_probe']]))
    response = capsys.readouterr().out.split('@cal:')[-1]
    assert json.loads(response)['protocol'] == 'miniarm-base-pulse-v1'
    instance.robot.calibration_base_write.assert_not_called()
    instance.calibration.locked = True
    asyncio.run(instance.parse_command([['home'], ['movemotor', '0', '90'], ['set_gripper', 'open']]))
    instance.robot.home.assert_not_called()
    instance.robot.set_joint.assert_not_called()
    instance.robot.set_gripper.assert_not_called()


def test_real_command_handler_releases_before_reply(setup, capsys):
    session, robot, outputs, now = setup
    controller, _ = controller_type()
    instance = controller()
    instance.calibration = session
    # Advancing fake time makes the actual async handler finish without waiting.
    def clock():
        now[0] += .05
        return now[0]
    session.clock = clock
    asyncio.run(instance.calibration_command(['cal_pulse', 'abc', '90', 'none']))
    result = json.loads(capsys.readouterr().out.split('@cal:')[-1])
    assert result['ok'] and result['outputs'] == [None]*7 and result['last'] == 90
    assert outputs == [None]*7


def prepare_hold(setup):
    session, robot, outputs, now = setup
    session.begin('a', 90, None); session.release(); now[0] += 2
    return session, robot, outputs, now


def test_hold_requires_a_prior_pulse_and_repeats_only_that_angle(setup):
    session, robot, _, _ = setup
    with pytest.raises(ValueError, match='pulse first'): session.begin_hold('b', None)
    robot.calibration_base_write.assert_not_called()
    session, robot, outputs, now = prepare_hold(setup)
    with pytest.raises(ValueError, match='Previous'): session.begin_hold('b', 91)
    session.begin_hold('b', 90)
    assert outputs[0] == 90 and session.mode == 'hold'
    assert session.deadline == now[0] + 3
    now[0] += 2.99; session.tick(); assert session.active
    now[0] += .02; session.tick()
    assert outputs == [None]*7 and not session.active
    assert session.mode is None and session.active_token is None


def test_owned_release_works_before_timeout_and_stale_release_cannot_stop_new_hold(setup):
    session, robot, outputs, now = prepare_hold(setup)
    session.begin_hold('b', 90)
    with pytest.raises(ValueError, match='own'): session.release_hold('a')
    assert session.active
    session.release_hold('b')
    assert not session.active and outputs == [None]*7
    now[0] += 2; session.begin_hold('c', 90)
    with pytest.raises(ValueError, match='own'): session.release_hold('b')
    assert session.active
    session.release_hold('c')


def test_hold_requests_never_extend_active_deadline(setup):
    session, _, _, now = prepare_hold(setup)
    session.begin_hold('b', 90); deadline = session.deadline
    now[0] += 1
    with pytest.raises(ValueError): session.begin_hold('b', 90)
    with pytest.raises(ValueError): session.begin_hold('c', 90)
    assert session.deadline == deadline
    session.release_hold('b')
    assert session.last == 90 and session.center == 90


def test_hold_command_returns_while_active_and_parser_accepts_release(setup, capsys):
    session, _, outputs, _ = prepare_hold(setup)
    controller, _ = controller_type(); instance = controller(); instance.calibration = session
    asyncio.run(instance.parse_command([['cal_hold', 'b', '90']]))
    result = json.loads(capsys.readouterr().out.split('@cal:')[-1])
    assert result['ok'] and result['active'] and result['hold_ms'] == 3000
    assert outputs[0] == 90
    asyncio.run(instance.parse_command([['cal_release', 'b']]))
    result = json.loads(capsys.readouterr().out.split('@cal:')[-1])
    assert result['ok'] and not result['active'] and outputs == [None]*7


def test_hold_release_failure_is_reported_and_latched(setup, capsys):
    session, robot, outputs, _ = prepare_hold(setup)
    controller, _ = controller_type(); instance = controller(); instance.calibration = session
    session.begin_hold('b', 90)
    robot.calibration_base_write.side_effect = OSError('release failed')
    asyncio.run(instance.calibration_command(['cal_release', 'b']))
    result = json.loads(capsys.readouterr().out.split('@cal:')[-1])
    assert not result['ok'] and 'Disconnect servo power' in result['error']
    assert session.fault


@pytest.fixture
def two_joints():
    now = [10.0]; outputs = [None]*7
    robot = Mock(); robot.simulate_hardware = False
    robot.calibration_outputs.side_effect = lambda: list(outputs)
    robot.calibration_base_write.side_effect = lambda value: outputs.__setitem__(0, value)
    robot.calibration_shoulder_write.side_effect = lambda value: outputs.__setitem__(1, value)
    joint_names = list(BaseCalibration.CHANNELS)
    robot.calibration_joint_write.side_effect = lambda joint, value: outputs.__setitem__(joint_names.index(joint), value)
    controller, _ = controller_type(); instance = controller(); instance.robot = robot
    instance.calibration = BaseCalibration(robot, clock=lambda: now[0])
    instance.shoulder_calibration = BaseCalibration(robot, clock=lambda: now[0], joint='shoulder')
    instance.other_calibrations = {joint: BaseCalibration(robot, clock=lambda: now[0], joint=joint) for joint in joint_names[2:]}
    return instance, robot, outputs, now


def test_shoulder_targets_are_independent_and_only_its_output_is_released(two_joints):
    instance, robot, outputs, now = two_joints
    base, shoulder = instance.calibration, instance.shoulder_calibration
    base.begin('a', 90, None); base.release(); now[0] += 2
    shoulder.begin('b', 60, None)
    assert outputs == [None, 60, None, None, None, None, None]
    assert shoulder.center == 60 and base.center == 90
    assert shoulder.status()['channel'] == 14
    assert shoulder.status()['protocol'] == 'miniarm-shoulder-pulse-v1'
    now[0] += .41; shoulder.tick()
    assert outputs == [None]*7
    robot.calibration_shoulder_write.assert_called_with(None)


def test_active_joint_or_fault_blocks_the_other_joint_even_when_registers_look_off(two_joints, capsys):
    instance, robot, outputs, now = two_joints
    instance.calibration.begin('a', 90, None); instance.calibration.release(); now[0] += 2
    instance.calibration.begin_hold('b', 90)
    # Exercise the global session interlock independently of the output guard.
    outputs[0] = None
    asyncio.run(instance.parse_command([['shoulder_pulse', 'c', '60', 'none']]))
    assert not json.loads(capsys.readouterr().out.split('@cal:')[-1])['ok']
    robot.calibration_shoulder_write.assert_not_called()
    instance.calibration.release(); instance.calibration.fault = 'I2C fault'
    asyncio.run(instance.parse_command([['shoulder_pulse', 'd', '60', 'none']]))
    assert not json.loads(capsys.readouterr().out.split('@cal:')[-1])['ok']
    robot.calibration_shoulder_write.assert_not_called()


def test_shoulder_protocol_routes_hold_release_and_locks_legacy_motion(two_joints, capsys):
    instance, robot, outputs, now = two_joints
    shoulder = instance.shoulder_calibration
    shoulder.begin('a', 60, None); shoulder.release(); now[0] += 2
    asyncio.run(instance.parse_command([['shoulder_hold', 'b', '60']]))
    status = json.loads(capsys.readouterr().out.split('@cal:')[-1])
    assert status['active'] and status['hold_protocol'] == 'miniarm-shoulder-hold-v1'
    assert outputs[1] == 60 and outputs[0] is None
    asyncio.run(instance.parse_command([['cal_release', 'b']]))
    assert shoulder.active and outputs[1] == 60  # wrong joint cannot release it
    asyncio.run(instance.parse_command([['shoulder_release', 'b']]))
    assert not shoulder.active and outputs == [None]*7
    asyncio.run(instance.parse_command([['home'], ['movemotor', '0', '90']]))
    robot.home.assert_not_called(); robot.set_joint.assert_not_called()


def test_shoulder_hold_timeout_and_controller_stop_clear_its_output(two_joints):
    instance, _, outputs, now = two_joints
    shoulder = instance.shoulder_calibration
    shoulder.begin('a', 60, None); shoulder.release(); now[0] += 2
    shoulder.begin_hold('b', 60); now[0] += 3.01; shoulder.tick()
    assert outputs == [None]*7
    now[0] += 2; shoulder.begin_hold('c', 60); instance.stop()
    assert outputs == [None]*7 and instance.all_stop


def test_unknown_joints_are_not_constructible(two_joints):
    _, robot, _, _ = two_joints
    for joint in ('neck', 'wrist', ''):
        with pytest.raises(ValueError, match='Unsupported'): BaseCalibration(robot, joint=joint)


@pytest.mark.parametrize('joint,index,channel', [('base',0,15), ('shoulder',1,14), ('elbow',2,13), ('wrist-rotation',3,12), ('wrist-bend',4,2), ('gripper-link',5,1), ('gripper',6,0)])
def test_every_joint_routes_and_releases_independently(two_joints, capsys, joint, index, channel):
    instance, robot, outputs, now = two_joints
    session = list(instance.calibration_sessions())[index]
    prefix = 'cal' if joint == 'base' else joint.replace('-', '_')
    asyncio.run(instance.parse_command([[prefix + '_probe']]))
    status = json.loads(capsys.readouterr().out.split('@cal:')[-1])
    assert status['channel'] == channel and status['protocol'] == 'miniarm-' + joint + '-pulse-v1'
    def advancing_clock():
        now[0] += .05
        return now[0]
    session.clock = advancing_clock
    asyncio.run(instance.parse_command([[prefix + '_pulse', 'a', '75', 'none']]))
    status = json.loads(capsys.readouterr().out.split('@cal:')[-1])
    assert status['ok'] and status['last'] == 75 and outputs == [None]*7
    assert all(other.last is None for other in instance.calibration_sessions() if other is not session)
    now[0] += 2
    asyncio.run(instance.parse_command([[prefix + '_hold', 'b', '75']]))
    assert json.loads(capsys.readouterr().out.split('@cal:')[-1])['active']
    assert outputs == [75 if i == index else None for i in range(7)]
    asyncio.run(instance.parse_command([['home'], ['set_gripper', 'open']]))
    robot.home.assert_not_called(); robot.set_gripper.assert_not_called()
    asyncio.run(instance.parse_command([[prefix + '_release', 'b']]))
    assert outputs == [None]*7
    now[0] += 2; session.begin_hold('c', 75)
    now[0] += 3.01; session.tick()
    assert outputs == [None]*7
    now[0] += 2; session.begin_hold('d', 75); instance.stop()
    assert outputs == [None]*7


def test_extra_joint_blocks_all_others_on_active_or_fault(two_joints, capsys):
    instance, robot, outputs, now = two_joints
    gripper = instance.other_calibrations['gripper']
    gripper.begin('a', 75, None)
    outputs[6] = None  # Check session interlock independently of PWM registers.
    for active in (True, False):
        gripper.active = active
        gripper.fault = None if active else 'bus fault'
        for prefix in ('cal', 'shoulder', 'elbow', 'wrist_rotation', 'wrist_bend', 'gripper_link'):
            asyncio.run(instance.parse_command([[prefix + '_pulse', 'b', '75', 'none']]))
            status = json.loads(capsys.readouterr().out.split('@cal:')[-1])
            assert not status['ok'] and 'active or' in status['error']
    assert robot.calibration_joint_write.call_count == 1
    robot.calibration_base_write.assert_not_called(); robot.calibration_shoulder_write.assert_not_called()
