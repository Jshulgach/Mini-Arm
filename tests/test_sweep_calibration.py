"""Slider motion uses a fake clock and bus; these tests never access hardware."""
import asyncio
import json
import pytest
from test_base_calibration import two_joints


@pytest.mark.parametrize('index', range(7))
def test_each_joint_slews_only_its_channel_and_releases_on_watchdog(two_joints, index):
    controller, _, outputs, now = two_joints
    session = list(controller.calibration_sessions())[index]
    session.begin('a', 90, None); session.release(); now[0] += 2
    session.begin_sweep('b', 90)
    session.update_sweep('b', 1, 95)
    for _ in range(5):
        now[0] += .01; session.tick()
    assert session.last == pytest.approx(90.5)
    assert outputs[index] == session.last
    assert all(value is None for i, value in enumerate(outputs) if i != index)
    # A long loop stall is capped to a 0.5-degree change, never a catch-up jump.
    now[0] += .3; session.tick()
    assert session.last == pytest.approx(91)
    now[0] += .3; session.tick()
    assert not session.active and outputs == [None]*7
    with pytest.raises(ValueError, match='inactive'): session.update_sweep('b', 2, 94)


def test_sweep_absolute_deadline_cannot_be_renewed(two_joints):
    controller, _, outputs, now = two_joints
    s = controller.calibration
    s.begin('a', 90, None); s.release(); now[0] += 2; s.begin_sweep('b', 90)
    deadline = s.deadline
    for sequence in range(1, 30):
        now[0] += .1; s.update_sweep('b', sequence, 95)
        assert s.deadline == deadline
    now[0] = deadline + .001
    with pytest.raises(ValueError, match='inactive'): s.update_sweep('b', 30, 95)
    assert outputs == [None]*7


def test_sweep_rejects_uninitialized_stale_foreign_and_invalid_updates(two_joints):
    controller, _, outputs, now = two_joints
    s = controller.calibration
    with pytest.raises(ValueError, match='pulse first'): s.begin_sweep('a', None)
    s.begin('a', 90, None); s.release(); now[0] += 2; s.begin_sweep('b', 90)
    for token, sequence, target in [('c',1,90), ('b',0,90), ('b',2,90), ('b',True,90), ('b',1,96), ('b',1,-1), ('b',1,181), ('b',1,float('nan')), ('b',1,True)]:
        before = s.sweep_seen
        with pytest.raises(ValueError): s.update_sweep(token, sequence, target)
        assert s.sweep_seen == before and s.sweep_sequence == 0
    with pytest.raises(ValueError, match='own'): s.release_hold('c')
    assert s.active
    s.release_hold('b'); assert outputs == [None]*7
    now[0] += 2
    with pytest.raises(ValueError, match='Duplicate'): s.begin_sweep('b', 90)


def test_sweep_write_failure_latches_and_releases(two_joints):
    controller, robot, outputs, now = two_joints
    s = controller.calibration
    s.begin('a', 90, None); s.release(); now[0] += 2; s.begin_sweep('b', 90)
    s.update_sweep('b', 1, 95)
    writes = []
    def fail(angle):
        writes.append(angle)
        if angle is not None: raise OSError('bus failure')
        outputs[0] = None
    robot.calibration_base_write.side_effect = fail
    now[0] += .01; s.tick()
    assert s.fault and not s.active and writes[-1] is None and outputs == [None]*7


@pytest.mark.parametrize('start,target', [(0,5), (180,175), (90,85)])
def test_sweep_reaches_target_without_overshoot_and_can_reverse(two_joints, start, target):
    controller, _, outputs, now = two_joints
    s = controller.calibration
    s.begin('a', start, None); s.release(); now[0] += 2; s.begin_sweep('b', start)
    s.update_sweep('b', 1, target)
    for sequence in range(2, 16):
        now[0] += .05; s.update_sweep('b', sequence, target)
        assert min(start, target) <= s.last <= max(start, target)
    assert s.last == pytest.approx(target)
    s.update_sweep('b', 16, start)
    now[0] += .05; s.tick()
    assert abs(s.last - target) == pytest.approx(.5)
    s.release_hold('b'); assert outputs == [None]*7


def test_sweep_release_fault_blocks_further_motion(two_joints):
    controller, robot, _, now = two_joints
    s = controller.calibration
    s.begin('a', 90, None); s.release(); now[0] += 2; s.begin_sweep('b', 90)
    robot.calibration_base_write.side_effect = OSError('release bus fault')
    now[0] += .61; s.tick()
    assert s.fault and not s.active and s.locked
    now[0] += 2
    with pytest.raises(ValueError, match='faulted'): s.begin_sweep('c', 90)


@pytest.mark.parametrize('index', range(7))
def test_immediate_jog_initializes_on_explicit_target_and_idle_keep_never_rewrites(two_joints, index):
    controller, robot, outputs, now = two_joints
    s = list(controller.calibration_sessions())[index]
    s.begin_jog('a', 80, None)
    assert s.center == 80 and s.last == 80 and outputs[index] == 80
    calls = robot.calibration_base_write.call_count + robot.calibration_shoulder_write.call_count + robot.calibration_joint_write.call_count
    deadline = s.deadline
    for sequence in range(1, 10):
        now[0] += .1; s.keep_sweep('a', sequence)
        assert s.deadline == deadline and s.sweep_target == 80
    assert calls == robot.calibration_base_write.call_count + robot.calibration_shoulder_write.call_count + robot.calibration_joint_write.call_count
    with pytest.raises(ValueError): s.keep_sweep('a', 9)
    with pytest.raises(ValueError): s.keep_sweep('b', 10)
    now[0] += .61; s.tick()
    assert outputs == [None]*7
    with pytest.raises(ValueError): s.keep_sweep('a', 10)


def test_jog_retarget_is_rate_limited_and_validated_before_any_write(two_joints):
    controller, robot, outputs, now = two_joints
    s = controller.calibration
    s.begin_jog('a', 90, None); s.release(); now[0] += 2
    robot.calibration_base_write.reset_mock()
    with pytest.raises(ValueError): s.begin_jog('b', 96, 90)
    robot.calibration_base_write.assert_not_called()
    s.begin_jog('b', 95, 90)
    assert outputs[0] == 90
    now[0] += .01; s.tick()
    assert outputs[0] == pytest.approx(90.1)


def test_jog_keep_parser_routes_without_refreshing_an_expired_session(two_joints, capsys):
    controller, _, outputs, now = two_joints
    asyncio.run(controller.parse_command([['gripper_jog', 'a', '80', 'none']]))
    assert json.loads(capsys.readouterr().out.split('@cal:')[-1])['jog_protocol'] == 'miniarm-gripper-jog-v1'
    asyncio.run(controller.parse_command([['gripper_keep', 'a', '1']]))
    assert json.loads(capsys.readouterr().out.split('@cal:')[-1])['sweep_target'] == 80
    now[0] += .61
    asyncio.run(controller.parse_command([['gripper_keep', 'a', '2']]))
    assert not json.loads(capsys.readouterr().out.split('@cal:')[-1])['ok']
    assert outputs == [None]*7


@pytest.mark.parametrize('joint', ['base', 'shoulder', 'elbow', 'wrist-rotation', 'wrist-bend', 'gripper-link', 'gripper'])
def test_parser_routes_slider_and_global_lock(two_joints, capsys, joint):
    controller, _, outputs, now = two_joints
    s = next(s for s in controller.calibration_sessions() if s.joint == joint)
    prefix = 'cal' if joint == 'base' else joint.replace('-', '_')
    s.begin('a', 90, None); s.release(); now[0] += 2
    asyncio.run(controller.parse_command([[prefix + '_sweep', 'b', '90']]))
    result = json.loads(capsys.readouterr().out.split('@cal:')[-1])
    assert result['active'] and result['mode'] == 'sweep'
    assert result['sweep_protocol'] == 'miniarm-' + joint + '-sweep-v1'
    for other in ('cal', 'shoulder', 'elbow', 'wrist_rotation', 'wrist_bend', 'gripper_link', 'gripper'):
        asyncio.run(controller.parse_command([[other + '_sweep', 'c', '90']]))
        assert not json.loads(capsys.readouterr().out.split('@cal:')[-1])['ok']
    asyncio.run(controller.parse_command([[prefix + '_target', 'b', '1', '95']]))
    assert json.loads(capsys.readouterr().out.split('@cal:')[-1])['sweep_target'] == 95
    asyncio.run(controller.parse_command([[prefix + '_release', 'b']]))
    assert not json.loads(capsys.readouterr().out.split('@cal:')[-1])['active']
    assert outputs == [None]*7
