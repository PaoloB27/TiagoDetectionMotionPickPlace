**List of commands used to execute the code**

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
