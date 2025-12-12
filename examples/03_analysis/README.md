# Analysis Scripts

Tools for analyzing Mini-Arm trajectory data and motion capture alignment.

## Scripts

### trajectory_command.py

Generate and execute circular trajectories:

```bash
python trajectory_command.py --port COM3 --radius 0.05 --center 0.135 0.0 0.22 --points 20
```

**Features:**
- Generate circular trajectories in 3D space
- Configurable radius, center, and number of points
- Direct execution on Mini-Arm via serial

**Arguments:**
- `--port` - Serial port for Mini-Arm
- `--radius` - Circle radius in meters
- `--center` - Center point [X, Y, Z]
- `--points` - Number of trajectory points
- `--help` - Show all options

---

### trajectory_comparison.py

Compare executed trajectory with commanded trajectory:

```bash
python trajectory_comparison.py --commanded data/commanded.csv --executed data/executed.csv
```

**Features:**
- Load commanded vs actual trajectories
- Compute position errors (RMSE, max error)
- Visualize differences with matplotlib
- Export error statistics

**Arguments:**
- `--commanded` - Path to commanded trajectory CSV
- `--executed` - Path to executed trajectory CSV
- `--output` - Output plot filename
- `--help` - Show all options

---

### convert_c3d_to_csv.py

Convert motion capture C3D files to CSV format:

```bash
python convert_c3d_to_csv.py --input data/capture.c3d --output data/capture.csv
```

**Features:**
- Read C3D motion capture files
- Extract marker positions
- Export to CSV for analysis

**Arguments:**
- `--input` - Input C3D file path
- `--output` - Output CSV file path
- `--help` - Show all options

**Dependencies:**
```bash
pip install c3d numpy pandas
```

---

### compute_alignment_transform.py

Compute transformation matrix between coordinate frames:

```bash
python compute_alignment_transform.py --mocap data/mocap.csv --robot data/robot.csv
```

**Features:**
- Align motion capture data with robot frame
- Compute rigid transformation (rotation + translation)
- Apply transformation to trajectory data
- Export aligned data

**Arguments:**
- `--mocap` - Motion capture data CSV
- `--robot` - Robot trajectory CSV
- `--output` - Output aligned data CSV
- `--help` - Show all options

**Dependencies:**
```bash
pip install numpy scipy
```

---

### utils.py

Utility functions for trajectory analysis:

**Functions:**
- `load_trajectory(filename)` - Load CSV trajectory data
- `compute_rmse(predicted, actual)` - Calculate root mean square error
- `plot_trajectory_3d(trajectory)` - 3D visualization
- `export_trajectory(trajectory, filename)` - Save to CSV

## Usage Workflow

### 1. Generate Trajectory

```bash
python trajectory_command.py --port COM3 --radius 0.05
```

### 2. Record Data

- Commanded trajectory saved automatically
- Actual trajectory from motion capture or robot feedback

### 3. Compare Results

```bash
python trajectory_comparison.py --commanded cmd.csv --executed exec.csv
```

### 4. Align Coordinate Frames (if using motion capture)

```bash
python compute_alignment_transform.py --mocap mocap.csv --robot robot.csv
```

## Data Format

All CSV files should have columns: `X, Y, Z` (in meters)

```csv
X,Y,Z
0.135,0.000,0.220
0.140,0.010,0.220
...
```

## Dependencies

Install analysis dependencies:

```bash
pip install numpy pandas matplotlib scipy c3d
```

## Troubleshooting

**Import errors:**
- Ensure all dependencies are installed
- Check Python environment activation

**File not found:**
- Use absolute paths or ensure working directory is correct
- Verify CSV files have correct format

**Plot not displaying:**
- Check matplotlib backend: `matplotlib.use('TkAgg')`
- Save to file instead: `--output plot.png`
