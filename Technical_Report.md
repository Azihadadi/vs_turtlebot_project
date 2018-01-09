### Technical Report of Visual Servoing using Turtlebot

Position Based Visual Servoing (PBVS) are used in this package to do the visual servoing using Turtlebot robot. How it works in generally we extracted information from images (features) that are used to reconstruct the current 3D pose (pose/orientation) of an object.
Combined with the knowledge of a desired 3D pose, we generate a Cartesian pose error signal that drives the robot to the goal.

Hardware used:
- Turtlebot 2
- Kinect
- Local machine

Software used:
- Gazebo (simulator)

Programming language used:
- C++

Dependencies:
- rosconsole
- roscpp
- visp_bridge
- visp_auto_tracker

In the implementation of this package, Visual Servoing Platform (VISP) library is used. It allows prototyping and developing applications using visual tracking and visual servoing technics at the heart of the researches done by Inria Lagadic team. ViSP is able to compute control laws that can be applied to robotic systems. It provides a set of visual features that can be tracked using real time image processing or computer vision algorithms.

By including VISP library in the main source code, most of the function such as declare feature point, declare homogeneous matrix until compute the control law are already provided. The general step of how this package works are:
1. By using VISP_auto_tracker we extract the features from the target (here we used qr code, which already defined for VISP_auto_tracker)
2. From it we got the geometry message which we transform to homogeneous matrix (here we reconstruct the 3d pose)
3. Project the world coordinate from homogeneous matrix and extract the Z value
4. Add the feature and compute the control law. To compute control law using VISP, first we have to declare the type of visual servoing we used, in here we used EYEINHAND_L_cVe_eJe. The control law will compute based on the features that already been added
5. Update the features every iteration and compute control law again to get the velocity that will send to the turtlebot 
6. Check the condition if the robot already close enough to the target by checking the value of current Z and desired Z and give some flag to stop
7. Visual Servoing is done.

The algorithm implemented can be simply viewed in flowchart diagram below:

[![vs_main_diag.png](https://s14.postimg.org/qg8g90cw1/vs_main_diag.png)](https://postimg.org/image/6ydst2fy5/)


Because this task is part of bigger task which have mapping task and pick and place task, we need a package named Multimaster that can organize all of our task. To put it simply, Multimaster package will execute each task from each other system that communicate using same topic. 

In visual servoing task, we are the middle task between mapping task and pick and place task, so we need to subscribe to the same topic as mapping task publish which is "nav_status" that will send us message that showing that their task is done, so it will be the clue for us to do our task. Meanwhile, after visual servoing task is done, we also have to publish message to the same topic as pick and place task will subscribe which is "vs_status" that will send them message that showing our task is done, so they can do their task. This multimaster flow can be seen in flow diagram below:

[![vs_multimaster.png](https://s14.postimg.org/cmk3kfxap/vs_multimaster.png)](https://postimg.org/image/gvotmm0jx/)
