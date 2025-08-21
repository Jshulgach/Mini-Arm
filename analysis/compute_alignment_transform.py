import os
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R
import utils


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Compute alignment transformation.')
    parser.add_argument('--root_path', type=str, default=r'C:\Users\HP\Desktop\MiniArmTrajectoryRecording_041225\data\trajectory', help='Root directory for the data')
    parser.add_argument("--file_path", type=str, default='Static_Rotated_measured.npz', help='File name for the static alignment data')
    parser.add_argument("--visualize", type=bool, default=False, help="Visualize the trajectory")
    args = parser.parse_args()

    # Load and compute the mean static transform
    T_static = utils.load_transform(os.path.join(args.root_path, args.file_path))
    T_static = utils.get_mean_transform(T_static)

    # The robot's expected end-effector pose in the static measurement is close to [135.0, 0.0, 215.0], so we need to
    # find a transformation that will allow us to align the rest of our data with the robot's expected coordinate frame
    T_desired = np.eye(4)
    # Rotate the static frame to match the robot's expected frame. This is a rough estimate and i based on the orientation of the markers
    T_desired[:3,:3] = R.from_euler('x', np.pi).as_matrix()
    T_desired[0:3, 3] = np.array([120.0, 0.0, 220.0])  # The gripper centers expected position (defined by the marker position)
    T_align = utils.get_alignment_transformation(T_static, T_desired)

    # Let's check to see if this works, should show the robot's expected position
    T_static_global = T_align @ T_static
    print(f"New global transform: {T_static_global[:3,3]}")
    # If it matches our expected position, we can save the transformation

    # Plot the alignment transformation if desired
    if args.visualize:
        utils.plot_transform(None, T_mean=[T_static_global], title='Aligned frame', show=True)

    # Save the alignment transformation
    save_path = os.path.join(args.root_path, 'alignment_transform.npz')
    np.savez(save_path, transforms=T_align)
    print(f"Saved alignment transformation to {save_path}")
