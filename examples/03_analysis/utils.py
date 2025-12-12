import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment


# =============== General utilities ==============

# The offset from the probe tip center to the probe model corner

def apply_axis_rotation(transform: np.ndarray, axis: str, angle: float, frame: str = 'local') -> np.ndarray:
    """
    Apply a rotation around a specified axis to a transformation matrix.

    Args:
        transform (np.ndarray): The 4x4 transformation matrix to modify.
        axis (str): The axis to rotate around: 'x', 'y', or 'z'.
        angle (float): Rotation angle in radians.
        frame (str): 'local' to rotate in the transform's frame, 'global' to rotate in the world frame.

    Returns:
        np.ndarray: The resulting 4x4 transformation matrix.
    """
    if axis.lower() not in ['x', 'y', 'z']:
        raise ValueError("Invalid axis. Use 'x', 'y', or 'z'.")
    if frame.lower() not in ['local', 'global']:
        raise ValueError("Invalid frame type. Use 'local' or 'global'.")

    # Create the 3x3 rotation matrix
    R_axis = R.from_euler(axis.lower(), angle).as_matrix()

    # Convert to homogeneous 4x4 matrix
    R_homog = np.eye(4)
    R_homog[:3, :3] = R_axis

    if frame == 'local':
        return transform @ R_homog
    else:  # frame == 'global'
        return R_homog @ transform

def apply_offset_to_transforms(T_list, T_offset):
    """Apply a fixed transformation offset to a list of transforms.

    Args:
        T_list (np.ndarray): Array of shape (N, 4, 4)
        T_offset (np.ndarray): Offset transform (4, 4)

    Returns:
        np.ndarray: Transformed array of shape (N, 4, 4)
    """
    T_transformed = [T_offset @ T for T in T_list]
    return np.stack(T_transformed, axis=0)

def computeAccelerationMagnitude(samples):
    """ Helper function for calculating the acceleration magnitude between matrices
    """
    translations = [sample[:3, 3] for sample in samples]  # Extract translation vectors
    velocities = np.diff(translations, axis=0)  # Compute velocities
    accelerations = np.diff(velocities, axis=0)  # Compute accelerations
    acceleration_magnitudes = np.linalg.norm(accelerations, axis=1)  # Compute magnitudes
    return acceleration_magnitudes


def computeMatrixFromMarkers(origin, side, front, rot_offset=None):
    """
    Calculate the transformation matrix of the coordinate frame defined by the marker positions.

    Parameters:
    ----------------
        origin (array): XYZ coordinates of the origin marker
        side (array): XYZ coordinates of the side marker
        front (array): XYZ coordinates of the front marker
        rot_offset (array): Rotation offset to apply to the coordinate frame

    Returns:
    ----------------
        rotation_matrix (array): Rotation matrix of the coordinate frame
        origin (array): XYZ coordinates of the origin marker
    """

    # if any of the lists contain nan values, return list of zeros
    if np.isnan(origin).any() or np.isnan(side).any() or np.isnan(front).any():
        return np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 3))

    # Define the origin
    origin = np.array(origin)

    # Define the X-axis (vector from Origin to Front)
    x_axis = np.array(front) - origin
    x_axis = x_axis / np.linalg.norm(x_axis)  # Normalize to unit vector

    # Define the Y-axis (vector from Origin to Side)
    y_axis = np.array(side) - origin
    y_axis = y_axis / np.linalg.norm(y_axis)  # Normalize to unit vector

    # Define the Z-axis (cross product of X and Y axes)
    z_axis = np.cross(x_axis, y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)  # Normalize to unit vector

    # Ensure orthogonality by recalculating Y-axis
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)  # Normalize to unit vector

    # Rotation matrix (transformation matrix without translation)
    rotation_matrix = np.vstack([x_axis, y_axis, z_axis])

    if rot_offset is not None:
        if rot_offset[0] != 0:
            origin = rotate_around_x(origin, rot_offset[0])

        if rot_offset[1] != 0:
            origin = rotate_around_y(origin, rot_offset[1])

        if rot_offset[2] != 0:
            origin = rotate_around_z(origin, rot_offset[2])


    return rotation_matrix, origin


def computeTransformMatrixFromMarkers(origin, side, front, rot_offset=None):
    """
    Computes a 4x4 homogeneous transformation matrix from 3 marker points.

    Parameters:
    - origin (array): XYZ coordinates of the origin marker
    - side (array): XYZ coordinates of the side marker (Y-axis reference)
    - front (array): XYZ coordinates of the front marker (X-axis reference)
    - rot_offset (tuple or array): Optional Euler angle rotation offset (radians)

    Returns:
    - transform_matrix (4x4 numpy array): Homogeneous transformation matrix
    """

    if np.isnan(origin).any() or np.isnan(side).any() or np.isnan(front).any():
        return np.eye(4)  # return identity if bad input

    origin = np.array(origin)
    x_axis = np.array(front) - origin
    x_axis /= np.linalg.norm(x_axis)

    y_axis = np.array(side) - origin
    y_axis /= np.linalg.norm(y_axis)

    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)

    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    # Compose rotation matrix (axes as columns)
    R = np.column_stack((x_axis, y_axis, z_axis))  # shape (3, 3)

    # Apply optional rotation offset
    if rot_offset is not None:
        # Build rotation matrix from Euler angles
        rx, ry, rz = rot_offset
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(rx), -np.sin(rx)],
                       [0, np.sin(rx),  np.cos(rx)]])
        Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                       [0, 1, 0],
                       [-np.sin(ry), 0, np.cos(ry)]])
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                       [np.sin(rz),  np.cos(rz), 0],
                       [0, 0, 1]])
        R_offset = Rz @ Ry @ Rx
        R = R @ R_offset

    # Assemble full 4x4 homogeneous transform
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = origin

    return T


def computeTransformSeries(origins, sides, fronts, rot_offset=None):
    transforms = []
    for o, s, f in zip(origins, sides, fronts):
        T = computeTransformMatrixFromMarkers(o, s, f, rot_offset)
        transforms.append(T)
    return np.stack(transforms)


# ======== Pose and transformation utilities =======

def downsampleByAveraging(source_array, target_length):
    # Calculate the segment size
    segment_size = len(source_array) / target_length

    # Create the downsampled array
    downsampled_array = np.array([
        np.mean(source_array[int(i * segment_size):int((i + 1) * segment_size)])
        for i in range(target_length)
    ])
    return downsampled_array

def average_rotations(R_matrices):
    """Average a set of rotation matrices using SVD."""
    R_stack = np.sum(R_matrices, axis=0)
    U, _, Vt = np.linalg.svd(R_stack)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:  # Fix improper rotation
        U[:, -1] *= -1
        R_mean = U @ Vt
    return R_mean

def downsample_transforms(T: np.ndarray, target_len: int) -> np.ndarray:
    """
    Downsample a (N, 4, 4) transformation array using SVD-averaged rotations.

    Args:
        T (np.ndarray): Input array of shape (N, 4, 4)
        target_len (int): Desired number of frames

    Returns:
        np.ndarray: Downsampled array of shape (target_len, 4, 4)
    """
    N = T.shape[0]
    indices = np.linspace(0, N, target_len + 1, dtype=int)
    downsampled = []

    for i in range(target_len):
        segment = T[indices[i]:indices[i + 1]]
        Rs = segment[:, :3, :3]
        ts = segment[:, :3, 3]

        R_avg = average_rotations(Rs)
        t_avg = np.mean(ts, axis=0)

        T_avg = np.eye(4)
        T_avg[:3, :3] = R_avg
        T_avg[:3, 3] = t_avg
        downsampled.append(T_avg)

    return np.stack(downsampled)

def downsampleMatrixData(pose_data, target_length):
    """
    Downsample the pose data to match the target length.

    Parameters:
    ----------------
    pose_data : list of np.ndarray
        List of 4x4 transformation matrices representing the pose data.
    target_length : int
        The target length to downsample to (number of ultrasound frames).

    Returns:
    ----------------
    downsampled_pose_data : list of np.ndarray
        List of downsampled 4x4 transformation matrices.
    """
    current_length = len(pose_data)
    indices = np.linspace(0, current_length - 1, target_length, dtype=int)
    downsampled_pose_data = np.array([pose_data[i] for i in indices])
    return downsampled_pose_data

def drawAxes(ax, r, t, scale=50):
    """ Draw the coordinate axes on the plot.

    Parameters:
    ----------------
    ax : matplotlib.axes._subplots.Axes3DSubplot
        The 3D axes object to draw the axes on.
    r : np.ndarray
        The rotation matrix.
    t : np.ndarray
        The translation vector.
    scale : float
        The scale of the axes.
    """

    # Define the axes and apply rotation
    axes = np.array([[scale, 0, 0], [0, scale, 0], [0, 0, scale]])
    axes = np.dot(r, axes.T).T

    # Draw the axes
    for color, axis in zip(['r', 'g', 'b'], axes):
        ax.quiver(*t, *axis, color=color)

def drawMarkerTrail(ax, r, t, probe_offset=None):
    """
    Draw just the vectors and probe tip on the plot in a trail.

    Parameters:
    ----------------
    ax : matplotlib.axes._subplots.Axes3DSubplot
        The 3D axes object to draw the axes on.
    r : np.ndarray
        The rotation matrix.
    t : np.ndarray
        The translation vector.
    probe_offset : np.ndarray
        The translation matrix for the probe offset.
    """

    # Create homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = r
    T[:3, 3] = t

    if probe_offset is not None:
        # If probe_offset is 3D, convert it to 4D by appending a 1 for homogeneous coordinates
        if probe_offset.shape == (3,):
            probe_offset_homogeneous = np.eye(4)
            probe_offset_homogeneous[:3, 3] = probe_offset
            T_combined = T @ probe_offset_homogeneous
        elif probe_offset.shape == (4, 4):
            T_combined = T @ probe_offset
        else:
            raise ValueError("probe_offset should either be a 3D vector or a 4x4 transformation matrix.")
    else:
        T_combined = T

    # Define the axes and apply rotation
    scale = 50
    axes = np.array([[scale, 0, 0], [0, scale, 0], [0, 0, scale]])
    axes = np.dot(r, axes.T).T

    # Draw the axes
    for color, axis in zip(['r', 'g', 'b'], axes):
        ax.quiver(*t, *axis, color=color)


    # Also add a marker to the bottom center of the probe model, so we know where the ultrasound images will
    # be centered from (assuming the probe piece size is 50mm x 35mm)
    T_us_center = np.eye(4)
    T_us_center[:3, 3] = np.array([50/2, -1, 35/2])
    T_marker = T_combined @ T_us_center

    # Plot the marker
    ax.scatter(*T_marker[:3, 3], color='r', marker='o', s=10)

def get_alignment_transformation(T_static, T_desired):
    if T_desired.shape != (4, 4):
        raise ValueError("T_expected must be a 4x4 transformation matrix.")
    T_align = T_desired @ np.linalg.inv(T_static) # Alignment transformation we can use for all our data
    print(f"Alignment transform: {T_align[:3,3]}")
    return T_align

def get_mean_transform(data):
    """ Computes the mean transform from a set of transforms
    Args:
        data (numpy.ndarray): The data to compute the mean transform from. (N, 4, 4)
    Returns:
        numpy.ndarray: The mean transform. (4, 4)
    """
    # Compute the mean transform
    T_mean = np.mean(data, axis=0)
    return T_mean

def load_transform(file_path):
    """ Loads the transform data into a file, makes sure the output is a transform matrix
    """
    data = np.load(file_path)
    # If the shape of the data is (N, 3), we need to add a column of ones to make it (N, 4)
    T_list = []
    if isinstance(data, np.lib.npyio.NpzFile):
        return data['transforms']
    # If the shape of the data is (N, 3), we need to add a column of ones to make it (N, 4)
    elif data.shape[1] == 3:
        for i in range(data.shape[0]):
            # Create a 4x4 matrix for each command point
            T = np.eye(4)
            T[0:3, 3] = data[i, :]  # Set the translation
            T_list.append(T)
        return np.stack(T_list)
    elif 'transforms' in data.keys():
        # If the shape of the data is (N, 4, 4) from an .npz, we need to extract the transforms
        return data['transforms']
    else:
        return data

def natsort(l):
    """
    Lambda function for nautural sorting of strings. Useful for sorting the
    list of file name of images with the target. Taken from:
    https://blog.codinghorror.com/sorting-for-humans-natural-sort-order/

    input:
        l: list of input images with the target
    output:
        Nutural sorted list of images
    """
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]

    return sorted(l, key=alphanum_key)

def plot_transform(ax=None, T_list=None, T_mean=None, color=None, alpha=0.5, arrow_scale=1, title='Trajectory', show=False):
    """ Plot the trajectory of the robot
    Args:
        data (list) : A list of transformations to plot of numpy.ndarray (N, 4, 4)
        ax (matplotlib.axes.Axes): The axes to plot on. If None, create a new figure.
        color (str): The color of the trajectory.
        alpha (float): The alpha value of the trajectory.
    """
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    if color is None:
        color = np.random.rand(3,)

    # Plot the trajectory
    if T_list is not None:
        for T in T_list:
            ax.scatter(T[:, 0, 3], T[:, 1, 3], T[:, 2, 3], color=color, alpha=alpha, linewidth=1, s=3)

    if T_mean is not None:
        for t in T_mean:
            ax.scatter(t[0, 3], t[1, 3], t[2, 3], color='k', alpha=alpha, linewidth=1, s=3)
            drawAxes(ax, t[0:3, 0:3], t[0:3, 3], scale=arrow_scale)

    ax.set_box_aspect([1, 1, 1])  # IMPORTANT - this is the new, key line
    set_axes_equal(ax)  # IMPORTANT - this is also required
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    if show:
        plt.show()

def plot_trajectory_comparison(ax, T_measured, T_reference, label='', color=None, match_color=False, alpha=0.8):
    """
    Plot measured vs reference trajectories for visual comparison.

    Args:
        ax: matplotlib 3D axes
        T_measured: (N, 4, 4) measured trajectory
        T_reference: (N, 4, 4) reference/command trajectory
        label: Label for legend prefix
        color: Optional base color
        alpha: Transparency level
    """
    if color is None:
        color = np.random.rand(3,)

    # Extract XYZ points
    X_meas, Y_meas, Z_meas = T_measured[:, 0, 3], T_measured[:, 1, 3], T_measured[:, 2, 3]
    X_ref, Y_ref, Z_ref = T_reference[:, 0, 3], T_reference[:, 1, 3], T_reference[:, 2, 3]

    ref_color = 'black'
    if match_color:
        ref_color = color
    # Plot reference trajectory (solid)
    ax.plot(X_ref, Y_ref, Z_ref, linestyle='--', linewidth=2, color=ref_color,
            label=f'{label} Reference')

    # Plot measured trajectory (dashed)
    ax.plot(X_meas, Y_meas, Z_meas, linestyle='-', linewidth=1.5, color=color,
            label=f'{label} Measured', alpha=alpha)

    # Optional: mark start and end points
    #ax.scatter(X_ref[0], Y_ref[0], Z_ref[0], color='green', marker='o', label=f'{label} Start')
    #ax.scatter(X_ref[-1], Y_ref[-1], Z_ref[-1], color='red', marker='X', label=f'{label} End')

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

def trim_transform_data(data, acc=100, fs=150, extra_pad=150, visualize=False):
    """Removed data before the first index where acceleration exceeds the threshold
    Args:
        data (numpy.ndarray): The data to trim. (N, 4, 4)
        acc (float): The acceleration threshold.
        fs (int): The sampling frequency. (Default 150Hz)
        extra_pad (int): The number of extra samples to remove after the threshold is crossed.
    """

    # Compute the velocity of the gripper over time
    TG_v = np.zeros((data.shape[0], 3))
    TG_v[0:-1, :] = np.diff(data[:, 0:3, 3], axis=0) * fs
    TG_v[-1, :] = TG_v[-2, :]  # Set the last velocity to the second last velocity
    TG_v_magnitude = np.linalg.norm(TG_v, axis=1)

    # Find the index when the magnitude velocity crosses a threshold
    threshold = acc  # m/s
    start_index = np.where(TG_v_magnitude > threshold)[0]
    start_index = start_index[0] if len(start_index) > 0 else None
    #print(f"Start index: {start_index}")

    # Plot the velocity magnitude
    if visualize:
        plt.plot(TG_v_magnitude)
        if start_index is not None:
            plt.axvline(x=(start_index+extra_pad), color='r', linestyle='--', label='Start Index')
        plt.title('Velocity Magnitude')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (mm/s)')
        plt.show()

    if start_index is not None:
        data = data[(start_index+extra_pad):, :, :]
    return data

def nearest_neighbor_match(T_meas, T_ref):
    """
    Match each measured point to the closest reference point.

    Args:
        T_measured (np.ndarray): Measured trajectory (N, 4, 4)
        T_reference (np.ndarray): Command/reference trajectory (N, 4, 4)

    Returns:
        distances (np.ndarray): Distance to closest point in P_ref for each P_meas
        indices (np.ndarray): Indices in P_ref of the closest points
    """
    P_meas = T_meas[:, :3, 3]
    P_ref = T_ref[:, :3, 3]
    tree = cKDTree(P_ref)
    distances, indices = tree.query(P_meas)
    return distances, indices


def compute_optimal_matching_error(T_m, T_c):
    """
    Match points in P1 to points in P2 using optimal 1-to-1 assignment (Hungarian algorithm).

    Args:
        T_m (np.ndarray): Measured trajectory (N, 4, 4)
        T_c (np.ndarray): Command/reference trajectory (N, 4, 4)

    Returns:
        dict: {
            'rmse': float,
            'l2_error': float,
            'match_indices': (row_idx, col_idx),
            'errors': np.ndarray
        }
    """
    P1 = T_m[:, :3, 3]
    P2 = T_c[:, :3, 3]
    assert P1.shape == P2.shape, "Point sets must have the same shape"

    dist_matrix = cdist(P1, P2)  # Pairwise distances
    row_idx, col_idx = linear_sum_assignment(dist_matrix)

    matched_errors = dist_matrix[row_idx, col_idx]
    l2_error = np.sum(matched_errors ** 2)
    rmse = np.sqrt(np.mean(matched_errors ** 2))

    return {
        'rmse': rmse,
        'l2_error': l2_error,
        'match_indices': (row_idx, col_idx),
        'errors': matched_errors
    }
