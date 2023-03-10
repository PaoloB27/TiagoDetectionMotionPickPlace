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
   * roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full

4. Terminal 3: start the navigation stack
   * cd ~/tiago_public_ws
   * source devel/setup.bash
   * roslaunch tiago_iaslab_simulation navigation.launch

5. Terminal 4: start the action server
   * cd ~/tiago_public_ws
   * source devel/setup.bash
   * rosrun simple_navigation_goals server

6. Terminal 5: start the action client with a final pose as command line argument
   * cd ~/tiago_public_ws
   * source devel/setup.bash
   * rosrun simple_navigation_goals client 11.448230601 0.500488424656 0.0 0.0 0.0 -0.784084627418 0.620653926957
