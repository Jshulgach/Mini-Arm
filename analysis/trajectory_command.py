# Script that creates a circular trajectory reference for the actual robot to follow
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import argparse
from mini_arm import MiniArmClient


def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale. Adapted from @Mateen Ulhaq and @karlo

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """

    def _set_axes_radius(ax, origin, radius):
        x, y, z = origin
        ax.set_xlim3d([x - radius, x + radius])
        ax.set_ylim3d([y - radius, y + radius])
        ax.set_zlim3d([z - radius, z + radius])

    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)


def create_circular_trajectory_YZ(center, radius=10, steps=101):
    """ Creates set of points along circle in Y-Z plane with given center and radius """
    theta = np.linspace(0, 2 * np.pi, steps)
    temp = np.array([np.zeros(len(theta)), np.cos(theta), np.sin(theta)]).transpose()
    return center + radius * temp
def create_circular_trajectory_XZ(center, radius=10, steps=101):
    """ Creates set of points along circle in X-Z plane with given center and radius """
    theta = np.linspace(0, 2 * np.pi, steps)
    temp = np.array([np.cos(theta), np.zeros(len(theta)), np.sin(theta)]).transpose()
    return center + radius * temp
def create_circular_trajectory_XY(center, radius=10, steps=101):
    """ Creates set of points along circle in X-Y plane with given center and radius """
    theta = np.linspace(0, 2 * np.pi, steps)
    temp = np.array([np.cos(theta), np.sin(theta), np.zeros(len(theta))]).transpose()
    return center + radius * temp

def create_linear_trajectory(start_point, end_point, steps):
    """ Creates set of points along line in X-Y-Z space with given start and end points """
    x = np.linspace(start_point[0], end_point[0], steps)
    y = np.linspace(start_point[1], end_point[1], steps)
    z = np.linspace(start_point[2], end_point[2], steps)
    return np.array([x, y, z]).transpose()


if __name__ == "__main__":

    parse = argparse.ArgumentParser(description='Create a circular trajectory for the robot to follow.')
    parse.add_argument('--radius', type=float, default=0.02, help='Radius of the circular trajectory')
    parse.add_argument('--steps', type=int, default=201, help='Number of steps in the trajectory')
    parse.add_argument("--use_serial", type=bool, default=False, help='Use serial connection to robot')
    parse.add_argument("--port", type=str, default='COM11', help='Serial port for the robot')
    parse.add_argument("--baudrate", type=int, default=115200, help='Baudrate for the serial connection')
    parse.add_argument("--save_filename", type=str, default='_command', help='Filename to save the trajectory data')
    args = parse.parse_args()

    # ====== Create the trajectory here =======
    # Circular trajectory (specify the plane too)
    traj = create_circular_trajectory_XZ([0.1, 0.0, 0.24], args.radius, args.steps)  # draw a circle of radius 0.02m

    # Create set of trajectory points
    #traj1 = create_circular_trajectory_XY([0.1, 0.030, 0.2], args.radius, args.steps)  # draw a circle of radius 0.02m
    #traj2 = create_circular_trajectory_XZ([0.1, 0.030, 0.2], args.radius, args.steps)  # draw a circle of radius 0.02m
    #traj3 = create_circular_trajectory_YZ([0.1, 0.030, 0.2], args.radius, args.steps)  # draw a circle of radius 0.02m
    #traj = np.concatenate((traj1, traj2, traj3), axis=0)

    # Square trajectory
    #traj1 = create_linear_trajectory([0.1, 0.0, 0.20], [0.1, 0.0, 0.25], args.steps)  # draw a line from z=0.25 to z=0.15
    #traj2 = create_linear_trajectory([0.1, 0.0, 0.25], [0.15, 0.0, 0.25], args.steps)  # draw a line from y=0.05 to y=0.15
    #traj3 = create_linear_trajectory([0.15, 0.0, 0.25], [0.15, 0.0, 0.20], args.steps)  # draw a line from z=0.25 to z=0.15
    #traj4 = create_linear_trajectory([0.15, 0.0, 0.20], [0.1, 0.0, 0.20], args.steps)  # draw a line from y=0.05 to y=0.15
    #traj = np.concatenate((traj1, traj2, traj3, traj4), axis=0)

    if args.use_serial:
        # Create serial connection to the robot and get its current info
        robot = MiniArmClient(port=args.port, baudrate=args.baudrate, command_delimiter=';', verbose=True)

        # Set the debugging state to false, and get the robot information displayed in the terminal
        robot.send_message('debug:off')
        time.sleep(0.1)
        robot.send_message('home')
        time.sleep(0.1)
        robot.get_info()

    try:
        # Initialize list to store actual robot pose
        actual_pose = []
        cmd_timestamp = []
        transforms = []

        # Send a new trajectory point to the robot every 2 seconds
        for i, val in enumerate(traj):
            print(i)
            pose_msg = "set_pose:[" + str(traj[i, 0]) + "," + str(traj[i, 1]) + "," + str(traj[i, 2]) + "];"

            try:
                if args.use_serial:
                    robot.send_message(pose_msg)
                    time.sleep(0.05)

                    # Get the current pose of the robot
                    pose = robot.get_current_pose()  # Should output the current pose of the robot
                    time.sleep(0.05)

                else:
                    # Use the trajectory as the pose
                    pose = traj[i, :].tolist()
                    # Extract pose from message "Current pose:   cords: [x: 0.13500, y: 0.01984, z: 0.25251]
                    #pose = msg.split('[')[1].split(']')[0].split(',')  # Extract the pose from the message
                    pose = [float(pose[0]), float(pose[1]), float(pose[2])]  # Convert to float
                    print(f"Current pose: {pose}")

                    # Add noise to the pose if simulating real-world conditions
                    #pose[0] += np.random.normal(0, 0.001)
                    #pose[1] += np.random.normal(0, 0.001)
                    #pose[2] += np.random.normal(0, 0.001)
                    actual_pose.append(pose)
                    cmd_timestamp.append(time.time())
                    # Update the transforms list to include the rotation and translation components
                    T = np.zeros((4, 4))  # Initialize transforms list with zeros
                    T[0:3, 0:3] = np.eye(3)  # Set rotation to identity for now
                    T[0:3, 3] = pose
                    T[3, 3] = 1.0
                    transforms.append(T)


            except Exception as e:
                print(f"Error extracting pose: {e}. Value: {pose}")

        # Convert the actual pose list to numpy array
        actual_pose = np.array(actual_pose)

        # plot the reference and actual trajectory
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color='red', linewidth=1, label='Reference Trajectory')
        ax.plot(actual_pose[:, 0], actual_pose[:, 1], actual_pose[:, 2], color='blue', linewidth=1,
                label='Actual Trajectory')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Trajectory Comparison')
        ax.set_box_aspect([1, 1, 1])  # IMPORTANT - this is the new, key line
        set_axes_equal(ax)  # IMPORTANT - this is also required
        ax.legend()
        plt.show()

        # Save the transforms and timestamp to a .npz file
        save_path = "_command.npy" # os.path.join(os.path.dirname(args.filepath))
        np.save(save_path, actual_pose)
        #np.savez(save_path, transforms=transforms)
        print(f"Saved gripper transformation series to {save_path}")

    except KeyboardInterrupt:
        print("Keyboard interrupt received")

    finally:
        if args.use_serial:
            robot.disconnect()
        print("Connection closed")
