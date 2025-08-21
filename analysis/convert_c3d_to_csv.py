import os
import numpy as np
import argparse
import ezc3d
import utils


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Plot trajectory from c3d file.')
    parser.add_argument('--filepath', type=str, default='Circle_Step400_Radius2cm.c3d', help='Path to the c3d file')
    parser.add_argument('--visualize', type=bool, default=False, help='Visualize the trajectory')
    parser.add_argument('--save', type=bool, default=False, help='Save the trajectory to a file')
    args = parser.parse_args()

    # Load the c3d file and get parameters
    c3d = ezc3d.c3d(args.filepath)
    n_frames = c3d['data']['points'].shape[2]
    fs = c3d['header']['points']['frame_rate']

    # Generate timestamps (in seconds)
    t_s = np.arange(n_frames) / fs

    # Get all the point labels and strip whitespace
    # Output should be "['Center1', 'LeftFinger2', 'LeftFinger3', 'RightFinger2', 'RightFinger3'"
    point_labels = [label.strip() for label in c3d['parameters']['POINT']['LABELS']['value']]
    print(f"Marker labels: {point_labels}")

    # Get the point data
    marker_positions = {}
    for i, label in enumerate(point_labels):
        xyz = c3d['data']['points'][:3, i, :]
        marker_positions[label] = xyz.T

    print(f"Marker data shape: {marker_positions['Center1'].shape}")

    origin_points = marker_positions['LeftFinger2']
    side_points = marker_positions['Center1']
    front_points = marker_positions['LeftFinger3']

    # Compute transformation matrix for each frame
    transforms = utils.computeTransformSeries(origin_points, side_points, front_points)
    print(f"Shape of Transforms output: {transforms.shape}")

    if args.visualize:
        utils.plot_transform(transforms, T_mean=None, title='Trajectory', show=True)

    # Save the Tg xyz points to a .npy file
    pose_file_name = os.path.basename(args.filepath).split('.')[0]
    save_path = os.path.join(os.path.dirname(args.filepath), f"{pose_file_name}_measured.npz")
    np.savez(save_path, transforms=transforms)
    print(f"Saved gripper transformation series to {save_path}")
