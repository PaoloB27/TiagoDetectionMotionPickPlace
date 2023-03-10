//Written and revised by Paolo Bresolin, Giacomo Gonella and Pietro Picardi
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <simple_navigation_goals/tiagoAction.h>
#include <assignment2/BAction.h>
#include <assignment2/CAction.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

tf2_ros::Buffer tfBuffer;

std::vector<float> pick_blue{8.14544919863, -1.89761968751, 0.0, 0.0, 0.0, -0.718213826102, 0.695822462986};
std::vector<float> pick_red{7.50802563481, -1.87577911843, 0.0, 0.0, 0.0, -0.59819169173, 0.801353043262};
std::vector<float> pick_green{7.49690597563, -4.0758535066, 0.0, 0.0, 0.0, 0.66838804878, 0.743812756175};

std::vector<float> waypoint_1{8.23617838109, -0.935607984807, 0.0, 0.0, 0.0, -0.612424949177, 0.790528735483};
std::vector<float> waypoint_2{10.9055370847, -2.47566480838, 0.0, 0.0, 0.0, 0.706847002942, 0.707366464028};

//Called once when the movement goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const simple_navigation_goals::tiagoResultConstPtr& result){
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    /*ROS_INFO("%d obstacles detected at: ", (int) result->objs_locations_x_coord.size());
    for(int i = 0; i < result->objs_locations_x_coord.size(); i++)
        ROS_INFO("[%f, %f]", result->objs_locations_x_coord[i], result->objs_locations_y_coord[i]);*/
}/*doneCb*/

//Called once when the movement goal becomes active
void activeCb(){
    ROS_INFO("Movement goal just went active");
}/*activeCb*/

//Called every time a feedback is received from the server of simple_navigation_goals
void feedbackCb(const simple_navigation_goals::tiagoFeedbackConstPtr& feedback){
    ROS_INFO("Got Feedback: Tiago is %s", feedback->current_action.c_str());
}/*feedbackCb*/

//Called once when the detection goal completes
void doneNode_B(const actionlib::SimpleClientGoalState& state, const assignment2::BResultConstPtr& result){
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("\nPosition:\nx = %f \ny = %f \nz = %f \nOrientation:\nx = %f \ny = %f \nz = %f \nw = %f \n",
             result->obj_pose[0], result->obj_pose[1], result->obj_pose[2],
             result->obj_pose[3], result->obj_pose[4], result->obj_pose[5],
             result->obj_pose[6]);
}/*doneNode_B*/

//Called once when the detection goal becomes active
void activeNode_B(){
    ROS_INFO("Detection goal just went active");
}/*activeNode_B*/

//Called every time a feedback is received from node_B
void feedbackNode_B(const assignment2::BFeedbackConstPtr& feedback){
    ROS_INFO("Got Feedback: Tiago is %s", feedback->current_action.c_str());
}/*feedbackNode_B*/

//Called once when the picking routine completes
void doneNode_C(const actionlib::SimpleClientGoalState& state, const assignment2::CResultConstPtr& result){
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Picking done? %d", result->task_completed);
}/*doneNode_C*/

//Called once when the arm goal becomes active
void activeNode_C(){
    ROS_INFO("Arm goal just went active");
}/*activeNode_C*/

//Called every time a feedback is received from node_C
void feedbackNode_C(const assignment2::CFeedbackConstPtr& feedback){
    ROS_INFO("Got Feedback: Tiago is %s", feedback->current_action.c_str());
}/*feedbackNode_C*/

//Function that return the pose in the absolute reference frame given the corresponding pose in the robot reference frame
//base_link
geometry_msgs::Pose transformPoseToAbsoluteFrame(geometry_msgs::Pose pose){
    geometry_msgs::Pose transformedPose;

    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    }/*try*/
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }/*catch*/

    tf2::Transform transform;
    tf2::fromMsg(transformStamped.transform, transform);
    tf2::Vector3 pointVec(pose.position.x, pose.position.y, pose.position.z);
    tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Transform poseTransform(quat, pointVec);
    tf2::Transform transformedPoseTransform = transform * poseTransform;
    transformedPose.position.x = transformedPoseTransform.getOrigin().x();
    transformedPose.position.y = transformedPoseTransform.getOrigin().y();
    transformedPose.position.z = transformedPoseTransform.getOrigin().z();
    transformedPose.orientation.x = transformedPoseTransform.getRotation().x();
    transformedPose.orientation.y = transformedPoseTransform.getRotation().y();
    transformedPose.orientation.z = transformedPoseTransform.getRotation().z();
    transformedPose.orientation.w = transformedPoseTransform.getRotation().w();

    return transformedPose;
}/*transformPoseToAbsoluteFrame*/

//Function that computes the "place" poses for both the robot and the object to be placed
void findPlacePoses(const std::vector<float>& centers_x, const std::vector<float>& centers_y, 
    std::vector<geometry_msgs::Pose>& robot_place, std::vector<geometry_msgs::Pose>& object_place){
    for (int i = 0; i < 3; i++){
        geometry_msgs::Pose center;

        //In order to use transformPoseToAbsoluteFrame() we need to impose some arbitrary values for the orientation
        center.position.x = centers_x[i];
        center.position.y = centers_y[i];
        center.position.z = 0.0;
        center.orientation.x = 0.0;
        center.orientation.y = 0.0;
        center.orientation.z = 0.0;
        center.orientation.w = 1.0;
        center = transformPoseToAbsoluteFrame(center);

        //Place pose for the object
        center.position.z = 0.8;
        object_place.push_back(center);

        //Place pose for the robot
        center.position.y -= 0.7;
        center.position.z = 0;
        center.orientation.z = 0.7;
        center.orientation.w = 0.7;
        robot_place.push_back(center);
    }/*for*/
}/*findPlacesPoses*/

int main(int argc, char** argv) {
    //PHASE 1: initialization
    ros::init(argc, argv, "node_A");
    ros::NodeHandle nh;

    //Collection of pick poses
    std::vector<std::vector<float>> pick;
    pick.push_back(pick_blue);
    pick.push_back(pick_green);
    pick.push_back(pick_red);

    //MOVEMENT CLIENT
    actionlib::SimpleActionClient<simple_navigation_goals::tiagoAction> ac("tiago", true);
    //Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the action server to come up");
    //Movement goal
    simple_navigation_goals::tiagoGoal movement_goal;

    //DETECTION CLIENT
    actionlib::SimpleActionClient<assignment2::BAction> ac1("B", true);
    //Wait for the action server to come up
    while (!ac1.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the action server to come up");
    //Detection goal
    assignment2::BGoal detection_goal;
    //Detection result
    assignment2::BResultConstPtr detection_result;

    //ARM CLIENT
    actionlib::SimpleActionClient<assignment2::CAction> ac2("C", true);
    //Wait for the action server to come up
    while (!ac2.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the action server to come up");
    //Arm goal
    assignment2::CGoal arm_goal;
    //Arm result
    assignment2::CResultConstPtr arm_result;

    //PHASE 2: retrieving the picking order
    ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
    tiago_iaslab_simulation::Objs srv;
    srv.request.ready = true;
    srv.request.all_objs = true;
    if (client.call(srv))
        ROS_INFO("Order of visit: %d, %d, %d", srv.response.ids[0], srv.response.ids[1], srv.response.ids[2]);
    else{
        ROS_ERROR("Failed to call service /human_objects_srv");
        return 1;
    }/*else*/

    //PHASE 3: preparing for the pick and place

    //$robot_place_poses: vector of poses to be reached by the robot to start the place of the corresponding object
    //$object_place_poses: vector of poses where the corresponding objects have to be placed
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::vector<geometry_msgs::Pose> robot_place_poses;
    std::vector<geometry_msgs::Pose> object_place_poses;

    for (int j = 0; j < 7; j++)
        movement_goal.final_position[j] = waypoint_1[j];
    //CALL MOVEMENT SERVER: Tiago must go to waypoint_1, otherwise it may happen that it gets stuck
    ac.sendGoal(movement_goal, &doneCb, &activeCb, &feedbackCb);
    ac.waitForResult(ros::Duration(120.0));
    ac.cancelGoal();

    //PHASE 4: now we alternate the pick and place routines using the boolean flag $place_flag
    bool place_flag = false;
    for(int i = 0; i < 2*srv.response.ids.size(); i++) {
        //Update of the tag ID to be picked
        if(!place_flag)
            detection_goal.id = srv.response.ids[i / 2];

        //Update the pose to reach depending on the routine we are going to perform
        for (int j = 0; j < 7; j++) {
            if (!place_flag)
                movement_goal.final_position[j] = pick[srv.response.ids[i / 2] - 1][j];
            else
                movement_goal.final_position[j] = waypoint_2[j];
        }/*for*/

        //CALL MOVEMENT SERVER
        ac.sendGoal(movement_goal, &doneCb, &activeCb, &feedbackCb);
        ac.waitForResult(ros::Duration(120.0));

        //After the first pick, we reach waypoint_2 (as for the first one, we need it to safely travel back and forth
        //between pick and place poses): now we detect the place poses
        if (i == 1) {
            ROS_INFO("SCANNING TO GET THE PLACE POSES FOR BOTH ROBOT AND OBJECTS");
            simple_navigation_goals::tiagoResultConstPtr scan_result = ac.getResult();
            findPlacePoses(scan_result->objs_locations_x_coord, scan_result->objs_locations_y_coord, robot_place_poses, object_place_poses);
        }/*if*/
        ac.cancelGoal();

        if(place_flag){

            //Set the pose of the robot as the one detected by the laser scanner
            int index = srv.response.ids[(i - 1)/ 2] - 1;
            movement_goal.final_position[0] = robot_place_poses[index].position.x;
            movement_goal.final_position[1] = robot_place_poses[index].position.y;
            movement_goal.final_position[2] = robot_place_poses[index].position.z;
            movement_goal.final_position[3] = robot_place_poses[index].orientation.x;
            movement_goal.final_position[4] = robot_place_poses[index].orientation.y;
            movement_goal.final_position[5] = robot_place_poses[index].orientation.z;
            movement_goal.final_position[6] = robot_place_poses[index].orientation.w;
            //CALL MOVEMENT SERVER: we go to the current place pose
            ac.sendGoal(movement_goal, &doneCb, &activeCb, &feedbackCb);
            ac.waitForResult(ros::Duration(120.0));
            ac.cancelGoal();
        }/*if*/

        //PLACE ROUTINE
        if(place_flag){
            ROS_INFO("Placing %d", srv.response.ids[(i - 1)/ 2]);

            //CALL ARM SERVER
            arm_goal.tag_id = srv.response.ids[(i - 1)/ 2];
            arm_goal.pick_place_flag = true;

            //Set the target pose as the one detected by the laser scanner
            int index = srv.response.ids[(i - 1)/ 2] - 1;
            arm_goal.obj_pose[0] = object_place_poses[index].position.x;
            arm_goal.obj_pose[1] = object_place_poses[index].position.y;
            arm_goal.obj_pose[2] = object_place_poses[index].position.z;
            arm_goal.obj_pose[3] = object_place_poses[index].orientation.x;
            arm_goal.obj_pose[4] = object_place_poses[index].orientation.y;
            arm_goal.obj_pose[5] = object_place_poses[index].orientation.z;
            arm_goal.obj_pose[6] = object_place_poses[index].orientation.w;

            //Send the goal to node_C
            ac2.sendGoal(arm_goal, &doneNode_C, &activeNode_C, &feedbackNode_C);
            ac2.waitForResult(ros::Duration(120.0));
            arm_result = ac2.getResult();
            ac2.cancelGoal();

            for (int j = 0; j < 7; j++)
                movement_goal.final_position[j] = waypoint_2[j];
            //CALL MOVEMENT SERVER: we return to waypoint_2
            ac.sendGoal(movement_goal, &doneCb, &activeCb, &feedbackCb);
            ac.waitForResult(ros::Duration(120.0));
            ac.cancelGoal();
        }/*if*/

        //PICK ROUTINE
        else{
            //CALL DETECTION SERVER
            ROS_INFO("Picking %d", detection_goal.id);
            ac1.sendGoal(detection_goal, &doneNode_B, &activeNode_B, &feedbackNode_B);
            ac1.waitForResult(ros::Duration(120.0));
            ac1.cancelGoal();
        }/*else*/

        //Alternating pick and place
        place_flag = !place_flag;
    }/*for*/
    return 0;
}/*main*/