# TiagoDetectionMotionPickPlace

## Brief description of the software
The project developed consists in making Tiago (a real existing mobile robot) move in an environment in order to pick three objects with different shapes and colors in the order specified by the user and then place them on three tables detected by the robot, such that each object is placed in the table that has the same color of the object itself.

## Main modules developed
The entire software has been produced, based on ROS.
The main modules developed are:
- movement of Tiago: make the robot navigate in the environment and reach whatever desired position, correctly avoiding obstacles;
- object detection using the laser: while Tiago has several cameras, in order to make the detection of obstacles and tables more challenging, a module that uses laser scanner measurements to detect obstacles has been developed and used to find the tables where to put the collected objects;
- planning scene and tag detection: the object is detected by means of an Apriltag placed on it and recognized from images collected by the cameras of the robot. Then the planning scene is created;
- pick routine: make the robot correctly pick the desired object without colliding with the table or other objects in the scene;
- place routine: software to place the object that Tiago holds on the desired position on the table.

## List of commands used to execute the code

1. Terminal 1: roscore
   * cd ~/tiago_public_ws
   * source devel/setup.bash
   * catkin build
   * roscore

2. Start the Xserver in Windows (we used VcXsrv with XLaunch)

3. Terminal 2: connect to the Xserver and start the simulation
   * cd ~/tiago_public_ws
   * source devel/setup.bash
   * export DISPLAY=$(ip route|awk '/^default/{print $3}'):0.0
   * roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables

4. Terminal 3: start the apriltag
   * cd ~/tiago_public_ws
   * source devel/setup.bash
   * roslaunch tiago_iaslab_simulation apriltag.launch

5. Terminal 4: start the navigation stack
   * cd ~/tiago_public_ws
   * source devel/setup.bash
   * roslaunch tiago_iaslab_simulation navigation.launch

6. Terminal 5: start the service to retrieve the picking order
   * cd ~/tiago_public_ws
   * source devel/setup.bash
   * rosrun tiago_iaslab_simulation human_node

7. Terminal 6: start the action server for moving Tiago,
   node C for pick and place operations, 
   node B for tag detection operations,
   node A for dictating the actions to perform
   * cd ~/tiago_public_ws
   * source devel/setup.bash
   * roslaunch assignment2 assignment2.launch
