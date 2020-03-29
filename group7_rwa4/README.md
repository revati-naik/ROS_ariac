### **Tasks have been completed**
  * Framework of arm2 grabbing, go to checking pose, and putting back gears from gear bin corresponding to the order
  * Framework of arm1 grabbing, go to checking pose from the belt
  * Sensor configuration

### **Tasks not yet completed**
  * Confirm if the part has to be in the tray to be checked or it can just be "shown" to QC
  * Arm1 drops good products into the left-most-bin
  * Building kit


### **How to build package**

Direct to your catkin_ws directory and run

 `catkin_make --only-pkg-with-deps group7_rwa4`

### **How to run**
To launch the environment, open a terminal and run

 `roslaunch group7_rwa4 group7_rwa4.launch`

Open two different terminals and run

 `roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1`
 
 `roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2`

Open the 4th terminal and run

 `rosrun group7_rwa4 main_node`

Open 5th terminal, wait for all robots be stationary, then run

 `rosservice call /ariac/start_competition `

