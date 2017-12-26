### Steps to set the simulation environment

In here we will set up the environment of the simulation, which using Gazebo. The main thing to do is to insert the marker (in this case, we are using qrcode) to the gazebo world. It will be explain step by step:

1. Download the marker0 folder ([HERE](https://github.com/anissalintang/vs_turtlebot_project/tree/master/marker0))
2. Move the marker0 folder to gazebo environment
   - Usually its in Home directory, press Ctrl + H to show hidden files
   - Go to .gazebo
   - Go to model
   - Paste the marker0 folder
3. Starts the turtlebot in the simulated world using this command:

   `$ roslaunch turtlebot_gazebo turtlebot_world.launch`

4. Choose Insert in the properties tab, and choose marker0 model
5. Place the marker in the world
6. Save the world file (.world) -- We save it as marker_world.world --
7. Open visual-servo-turtlebot-sim.launch and change the directory in line 16

   `<arg name="world_name" value="[PATH TO YOUR MARKER WORLD]"/>`
