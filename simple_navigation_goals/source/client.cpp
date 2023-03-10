//Written and revised by Paolo Bresolin, Giacomo Gonella and Pietro Picardi
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <simple_navigation_goals/tiagoAction.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>

//Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const simple_navigation_goals::tiagoResultConstPtr& result) {
    //print the results
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("%d obstacles detected at: ", result->objs_locations_x_coord.size());
    for(int i = 0; i < result->objs_locations_x_coord.size(); i++)
        ROS_INFO("[%f, %f]", result->objs_locations_x_coord[i], result->objs_locations_y_coord[i]);
    //everything is finished, so shutdown
    ros::shutdown();
} /*doneCb*/

//Called once when the goal becomes active
void activeCb() {
    ROS_INFO("Goal just went active");
} /*activeCb*/

//Called every time a feedback is received. It prints the feedback
void feedbackCb(const simple_navigation_goals::tiagoFeedbackConstPtr& feedback) {
    ROS_INFO("Got Feedback: Tiago is %s", feedback->current_action.c_str());
} /*feedbackCb*/

int main(int argc, char** argv) {
    //Setting up the action client
    ros::init(argc, argv, "tiago_client");
    actionlib::SimpleActionClient<simple_navigation_goals::tiagoAction> ac("tiago", true);

    //Wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the action server to come up");

    //Registering the final pose as goal of the ActionClientServer
    simple_navigation_goals::tiagoGoal goal;
    for(int i = 0; i < argc - 1; i++)
        goal.final_position[i] = std::stof(argv[i + 1]);

    //Sending the goal to the server
    ROS_INFO("Sending goal");
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();
    return 0;
} /*main*/