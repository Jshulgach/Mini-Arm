"""Explicit ROS runtime test. Refuses to run unless only mock hardware is present.

Not collected by pytest. Launch mock.launch.py in localhost-only ROS domain 87.
"""
import os
import time

import rclpy
from rclpy.action import ActionClient
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=87 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    rclpy.init()
    node = rclpy.create_node('miniarm_mock_smoke')
    latest = {}
    samples = [0]
    def observe(msg):
        latest.update(zip(msg.name, msg.position))
        samples[0] += 1
    subscription = node.create_subscription(JointState, '/joint_states', observe, 10)
    def result(future, timeout=10):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('ROS operation timed out')
        return future.result()
    def service(kind, name):
        client = node.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Missing service: ' + name)
        return result(client.call_async(kind.Request()))
    try:
        hardware = service(ListHardwareComponents, '/controller_manager/list_hardware_components').component
        assert len(hardware) == 1 and hardware[0].name == 'MiniarmMockSystem'
        assert hardware[0].plugin_name == 'mock_components/GenericSystem', 'Physical hardware is forbidden'
        states = {c.name: c.state for c in service(ListControllers, '/controller_manager/list_controllers').controller}
        for name in ('joint_state_broadcaster', 'miniarm_controller', 'miniarm_hand_controller'):
            assert states.get(name) == 'active', (name, states)
        joints = ['miniarm_joint1', 'miniarm_joint2', 'miniarm_joint3', 'miniarm_joint4', 'miniarm_joint5', 'miniarm_gripper_link_joint']
        arm = ActionClient(node, FollowJointTrajectory, '/miniarm_controller/follow_joint_trajectory')
        hand = ActionClient(node, GripperCommand, '/miniarm_hand_controller/gripper_cmd')
        assert arm.wait_for_server(timeout_sec=5) and hand.wait_for_server(timeout_sec=5)
        def send(client, goal):
            handle = result(client.send_goal_async(goal))
            assert handle.accepted
            completed = result(handle.get_result_async())
            assert completed.status == 4, completed.status  # action_msgs/GoalStatus SUCCEEDED
            return completed.result
        def await_state(targets):
            before = samples[0]
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if samples[0] > before and all(name in latest and abs(latest[name]-value) < .001 for name, value in targets.items()):
                    return
            raise AssertionError(('Joint state did not reach target', targets, latest))
        for targets, gripper in [([.15, -.1, .1, 0., 0., 0.], -.2), ([0.]*6, 0.)]:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = joints
            point = JointTrajectoryPoint()
            point.positions = targets
            point.time_from_start.sec = 2
            goal.trajectory.points = [point]
            assert send(arm, goal).error_code == 0
            hand_goal = GripperCommand.Goal()
            hand_goal.command.position = gripper
            hand_goal.command.max_effort = 1.
            assert send(hand, hand_goal).reached_goal
            await_state(dict(zip(joints + ['miniarm_gripper_joint'], targets + [gripper])))
        print('PASS: mock-only hardware, 3 active controllers, arm/gripper actions, fresh joint feedback, return to zero')
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
