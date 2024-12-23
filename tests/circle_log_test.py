# Script that creates a circular trajectory reference for the actual robot to follow
import time
import numpy as np
import serial
import matplotlib.pyplot as plt


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


def create_circular_trajectory(center, radius=10, steps=101):
    """ Creates set of points along circle in Y-Z plane with given center and radius """
    theta = np.linspace(0, 2 * np.pi, steps)
    temp = np.array([np.zeros(len(theta)), np.cos(theta), np.sin(theta)]).transpose()
    return center + radius * temp


if __name__ == "__main__":
    # Create set of trajectory points
    traj = create_circular_trajectory([0.135, 0.000, 0.25], 0.02, 101)  # draw a circle of radius 0.02m

    # Initialize list to store actual robot pose
    actual_pose = []

    # Create serial connection to the robot
    s = serial.Serial('COM11', 9600, timeout=1)
    s.flush()
    print("Connection opened, try getting info")
    s.write(b';')
    time.sleep(0.01)  # Need to wait for a bit after sending each command
    s.write(b'info;')
    time.sleep(0.01)

    # if there are bytes available in the input buffer, read them
    while s.in_waiting > 0:
        print(s.readline())

    # Set the debugging state to false
    s.write(b'debug:off;')
    time.sleep(0.01)

    try:
        # Send a new trajectory point to the robot every 2 seconds
        for i, val in enumerate(traj):
            print(i)
            pose_msg = "set_pose:[" + str(traj[i, 0]) + "," + str(traj[i, 1]) + "," + str(traj[i, 2]) + "];"
            s.write(pose_msg.encode('utf-8'))
            time.sleep(0.02)

            # Get the current pose of the robot
            s.write(b'get_pose;')  # Should output the current pose of the robot
            time.sleep(0.02)

            # If there are bytes available in the input buffer, read them
            msg = ""
            while s.in_waiting > 0:
                msg += s.read(1).decode('utf-8')

            # Extract pose from message "Current pose:   cords: [x: 0.13500, y: 0.01984, z: 0.25251]
            try:
                pose = msg.split('[')[1].split(']')[0].split(',')  # Extract the pose from the message
                pose = [float(pose[0]), float(pose[1]), float(pose[2])]  # Convert to float

                # Add noise to the pose if simulating real-world conditions
                #pose[0] += np.random.normal(0, 0.001)
                #pose[1] += np.random.normal(0, 0.001)
                #pose[2] += np.random.normal(0, 0.001)

                actual_pose.append(pose)
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

    except KeyboardInterrupt:
        print("Keyboard interrupt received")

    finally:
        s.close()
        print("Connection closed")
