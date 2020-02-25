## **ENPM809B: Manufacturing Robot Software System: RWA-2**

This is README to run the ROS package created for RWA2. 

## **Package Name: group7_rwa2**
This package consists of a listner (ROS Program) to get the sensor/camera data on the screen. 


## **Dependencies**
  * ROS Melodic 
  * Gazebo 9.6 
  * Ariac 2019

## **How to build**
1. Create and build a catkin workspace

      `mkdir -p ~/catkin_ws/src`
       
      `cd ~/catkin_ws/`
        
 
 
 
2.  Extract the package in Catkin Workspace

       `cd ~/catkin_ws/src/group7_rwa2`
         
3. Source the setup.bash to add environment variables to your path to allow ROS to function

      `source /opt/ros/melodic/setup.bash`

 
4. Build your catkin workspace

      `cd ~/catkin_ws`
       
      `catkin_make`

    Note: Always call `catkin_make` in the root of your catkin workspace. 

5. Overlay your cactkin workspace on top of your environment.

     `source devel/setup.bash`
 

## **Launch the Package**

To run the environment and the listener node, open another terminal and run the following command

`roslaunch group7_rwa2 group7_rwa2.launch`
