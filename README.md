# Visual Servoing Using Turtlebot Project
A visual servoing task using turtlebot (ROS) and VISP

*This repo mainly inspired by Demo_Pioneer projects (http://wiki.ros.org/demo_pioneer)*

Author:
- Anissa Lintang Ramadhani (alintangr@gmail.com)
- Sepideh Hadadi (tara.hadadi@gmail.com)

##### Objective:
Implement visual servoing schemes in order to control a Turtlebot robot.

This package contains a set of nodes that can be used to run visual servoing demonstrations on a real Turtlebot 2 robot equipped with a camera (Kinect is used). These nodes illustrate the usage of vision_visp stack; especially visp_auto_tracker for the tracking of the target, and visp usage in order to implement a position based visual servoing.

Two launch file for demonstrations are provided:

- Demonstration with real turtlebot (Video available **HERE**)
- Simulation with gazebo (Video available [**HERE**](https://youtu.be/-IIlRZ5Ic-g))
  -  Setting simulation environment [**HERE**](https://github.com/anissalintang/vs_turtlebot_project/blob/master/Simulation_setting.md)

Technical Report regarding the project can be found more detail in ([**HERE**](https://github.com/anissalintang/vs_turtlebot_project/blob/master/Technical_Report.md))

##### Installation
- Install ROS and create a catkin workspace

  `mkdir -p ~/catkin_ws/src`

  `cd ~/catkin_ws/src`

  `catkin_init_workspace`

  `cd ~/catkin_ws`

  `catkin_make`

- Bring the source

  `cd ~/catkin_ws/src`

- Get vs_turtlebot stack that does the visual servoing

  `git clone https://github.com/anissalintang/vs_turtlebot_project.git (master branch)`

- Install dependencies

  `source ~/catkin_ws/devel/setup.bash`

  `rosdep update`

  `rosdep install vs_turtlebot`

- Build the source

  `cd ~/catkin_ws`

  `catkin_make -DCMAKE_BUILD_TYPE=Release --pkg vs_turtlebot`

##### Usage
- Start visual servoing demo on real Turtlebot using:

  `source ~/catkin_ws/devel/setup.bash`

  `roslaunch vs_turtlebot visual-servo-turtlebot.launch`

- Start visual servoing demo on gazebo simulation using:

  `source ~/catkin_ws/devel/setup.bash`

  `roslaunch vs_turtlebot visual-servo-turtlebot-sim.launch`

This will open a windows name "visp_auto_tracker debug display" that shows the images provided by the camera. Present the target (marker) in front of the camera. It will automatically detect the marker and give velocity to the robot so it will move towards target.
