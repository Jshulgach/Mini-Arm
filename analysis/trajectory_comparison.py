import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import utils

def transform_pipeline(file_name, root_path, T_align):
    """ Helper function to load and process trajectory data """
    m_path = os.path.join(root_path, f"{file_name}_measured.npz")
    c_path = os.path.join(root_path, f"{file_name}_command.npy")

    T_data = utils.load_transform(m_path)
    T_c = utils.load_transform(c_path)
    T_c[:, :3, 3] *= 1000  # Convert m to mm since robot takes commands in meters

    # Let's trim some of the measured data to keep what we want, avoid artifacts from sudden robot movements
    T_data = utils.trim_transform_data(T_data, acc=100, extra_pad=200)  # Trim the data to avoid artifacts

    # Downsample the data to reduce the number of points
    T_data = utils.downsample_transforms(T_data, T_c.shape[0])  # Downsample the data to reduce the number of points

    # Apply alignment transformation to data to get it in the robot's expected frame
    T_data = utils.apply_offset_to_transforms(T_data, T_align)
    return T_data, T_c

def trajectory_comparison_plot(T_m, T_c, color='blue', label='Trajectory', visualize=True, save_path=None):
    """ Helper function to plot trajectory comparison """
    if visualize:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        utils.plot_trajectory_comparison(ax, T_m, T_c, color=color, label=label)
        ax.set_title('Spherical Trajectory')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.legend()
        ax.set_box_aspect([1, 1, 1])
        utils.set_axes_equal(ax)
        plt.show()

        # Save the figure
        if save_path is not None:
            fig.savefig(save_path, dpi=300)
            print(f"Saved trajectory comparison figure to {save_path}")

def process_trials(trial_configs, root_path, T_align, visualize=True, save_plots=True):
    """ Helper function to process trials """
    rmse_list = []
    for trial in trial_configs:
        file_name, color, label = trial
        T_m, T_c = transform_pipeline(file_name, os.path.join(root_path, 'trajectory'), T_align)
        metrics = utils.compute_optimal_matching_error(T_m, T_c)
        print(f"| Optimal RMSE: {metrics['rmse']:.2f} mm")
        print(f"| Least Squares Error: {metrics['l2_error']:.2f}")
        rmse_list.append(metrics['rmse'])
        save_path = None
        if save_plots:
            save_path = os.path.join(root_path, 'media', f"{file_name}_3DTrajectory.png")
        trajectory_comparison_plot(T_m, T_c, color=color, label=label, visualize=visualize, save_path=save_path)
    return rmse_list

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Compare trajectory data.')
    parser.add_argument('--root_path', type=str, default=r'C:\Users\HP\Desktop\MiniArmTrajectoryRecording_041225\data', help='Root directory for the data')
    parser.add_argument("--alignment_file", type=str, default='alignment_transform.npz', help='File name for the static alignment data')
    args = parser.parse_args()

    # ============ ALIGNMENT TRANSFORMATION =============
    trajectory_data_path = os.path.join(args.root_path, 'trajectory')
    T_align = utils.load_transform(os.path.join(trajectory_data_path, args.alignment_file))

    # ============== MEASURED TRAJECTORY DATA ===================
    # We can set up information to load and process trajectory data. Once the alignment transformation is computed, we
    # can apply it to all the measured trajectories to get them in the robot's expected frame. Lastly, we can save plots
    trial_configs = [
        # file_name, color, label
        ("GCircle_Step150_Radius2cm_XYZ_p1", 'blue', "Sphere P1"),
        ("GCircle_Step150_Radius2cm_XYZ_p2", 'red', "Sphere P2"),
        ("GCircle_Step150_Radius2cm_XYZ_p3", 'orange', "Sphere P3"),
        ("CircleYZ_Step200_Radius2cm", 'purple', "Circle Step200"),
        ("CircleYZ_Step100_Radius2cm", 'yellow', "Circle Step100"),
        ("CircleYZ_Step400_Radius2cm", 'green', "Circle Step400"),
        ("SquareYZ_Step200_Edge5cm", 'blue', "Square Step200"),
        ("SquareYZ_Step400_Edge5cm", 'magenta', "Square Step400"),
        ("SagittalSquareXZ_Step200_Edge5cm", 'cyan', "Sagittal Square200"),
        ("SagittalSquareXZ_Step400_Edge5cm", 'blue', "Sagittal Square400"),
        ("CircleYZ_Step200_Radius2cm_NoRubberBand_p1", 'purple', "CircleYZ NoBand200"),
        ("CircleYZ_Step200_Radius2cm_NoRubberBand_p2", 'blue', "CircleXZ NoBand200"),
        ("CircleXZ_Step200_Radius2cm_NoRubberBand_p3", 'green', "CircleXZ NoBand200"),
    ]
    rmse_list = process_trials(trial_configs, args.root_path, T_align, visualize=False, save_plots=False)

    # Save rms error
    # rmse_path = os.path.join(trajectory_data_path, 'rmse.txt')
    # with open(rmse_path, 'w') as f:
    #     for trial, rmse in zip(trial_configs, rmse_list):
    #         f.write(f"{trial[0]}: {rmse:.2f} mm\n")
    #     f.write(f"Average RMSE: {np.mean(rmse_list):.2f} mm\n")
    #     f.write(f"Min RMSE: {np.min(rmse_list):.2f} mm\n")
    #     f.write(f"Max RMSE: {np.max(rmse_list):.2f} mm\n")
    # print(f"Saved RMSE values to {rmse_path}")