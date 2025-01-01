# Mini-Arm Servo 

Repository for the Mini-Arm Servo ROS package. This package is responsible for controlling the servo motors of the Mini-Arm.

## Robot Visualization Demo

Open a terminal with the ROS2 libraries as well as your workspace sourced. RUn the command below to launch the Mini-Arm visualizer demo in RViz2:

```bash
ros2 launch miniarm_servo show_robot.launch.py
```

In the RViz2 Displays panel, in "Gobal Options" set "fixed frame" to `world`. Add a "RobotModel" display and set "Description Topic" to `/robot_description`.

Now you can play with the robot joint positions using the sliders in the "Joint State" GUI panel.