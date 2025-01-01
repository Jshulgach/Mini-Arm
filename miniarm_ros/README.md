# Mini-Arm ROS2 packages

This folder contains several RPS2 packages for using the arm in ROS2. Package building using `colcon` is recommended.

- Clone the Mini-Arm repo into your ROS2 workspace if you haven't done so:
    ```bash
    cd /your/ros2/workspace/src
    git clone https://github.com/Jshulgach/Mini-Arm.git
    ```
- Install dependencies using rosdep:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
  
- Build the packages using `colcon`:
    ```bash
    cd ..
    colcon build --packages-select miniarm_description miniarm_servo miniarm_moveit_config
    ```
- Source the workspace. For linux:
    ```bash 
    source install/setup.bash
    ```
- For Windows:
    ```cmd
    call install/setup.bat
    ```
  
Refer to the individual packages for usage instructions. 

<!-- There's also support for [ROS](https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico) on the microcontroller if you want a challenge... -->