This is README to run the ROS package created for RWA2. 

TO BE FILLED

## **Dependencies**
  * ROS Melodic 
  * Gazebo 9.6 
  * Ariac 2019

## **How to build**
Create and build a catkin workspace

 `mkdir -p ~/catkin_ws/src`
 
 `cd ~/catkin_ws/`
 
 
 
Build Project in Catkin Workspace

 `cd ~/catkin_ws/`

  `source opt/ros/melodic/setup.bash `

 `source devel/setup.bash`
 
 
 Extract the package in `~/catkin_ws/src`
 
 `cd ~/catkin_ws/`
 
 `catkin_make`
 
How to run nodes using launch file
once your environment is set

`
~/catkin_ws`


To run the environment and the listener node, open another terminal and run the following command

`roslaunch group7_rwa2 group7_rwa2.launch`
