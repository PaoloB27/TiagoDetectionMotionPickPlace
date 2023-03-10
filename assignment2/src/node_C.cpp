//Written and revised by Paolo Bresolin, Giacomo Gonella and Pietro Picardi
#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment2/CAction.h>
#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <ros/service_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_control_client;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_control_client;

class CAction{
protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<assignment2::CAction> as;
    std::string action_name;
    assignment2::CResult result;
    assignment2::CFeedback feedback;
    ros::Publisher gripper_pub;
    ros::Publisher joint_pub;
    std::vector<double> closed_gripper_joint_values {0.00, 0.00};
    std::vector<double> open_gripper_joint_values {0.04, 0.04};
    const std::vector<std::vector<double>> waypoint_arm_joint_angles{
            {2.44, 1.02, -0.04, 0.41, -1.57, 1.37, 0.00}, //blue
            {1.30, 1.02, -0.14, 0.41, -1.57, 1.37, 0.00}, //green
            {0.70, 1.02, -0.04, 0.85, -1.57, 1.37, 0.00} //red
    };

    const std::vector<double> resting_arm_joint_angles{0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.00};
public:
    //Constructor and destructor
    CAction(std::string name) : as(nh, name, boost::bind(&CAction::executeCB, this, _1), false), action_name(name){as.start();}
    ~CAction(void) {}

    //Function to attach the object to be picked to Tiago arm_7_joint
    void attachRoutine(const assignment2::CGoalConstPtr &goal){

        //Create a service client for the attach service
        ros::ServiceClient attach_srv = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

        //Create an attach request
        gazebo_ros_link_attacher::AttachRequest req;
        if (goal->tag_id == 1){
            req.model_name_1 = "Hexagon";
            req.link_name_1 = "Hexagon_link";
        }/*if*/
        else if (goal->tag_id == 2){
            req.model_name_1 = "Triangle";
            req.link_name_1 = "Triangle_link";
        }/*else if*/
        else if (goal->tag_id == 3){
            req.model_name_1 = "cube";
            req.link_name_1 = "cube_link";
        }/*else if*/
        req.model_name_2 = "tiago";
        req.link_name_2 = "arm_7_link";

        //Send the request to the service
        gazebo_ros_link_attacher::AttachResponse resp;
        if (attach_srv.call(req, resp))
            ROS_INFO("Link attachment successful!");
        else
            ROS_ERROR("Failed to attach link");

    }/*attachRoutine*/

    //Function to detach the object to be placed from Tiago arm_7_joint
    void detachRoutine(const assignment2::CGoalConstPtr &goal){

        //Create a service client for the detach service
        ros::ServiceClient detach_srv = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

        //Create a detach request
        gazebo_ros_link_attacher::AttachRequest req;
        if (goal->tag_id == 1){
            req.model_name_1 = "Hexagon";
            req.link_name_1 = "Hexagon_link";
        }/*if*/
        else if (goal->tag_id == 2){
            req.model_name_1 = "Triangle";
            req.link_name_1 = "Triangle_link";
        }/*else if*/
        else if (goal->tag_id == 3) {
            req.model_name_1 = "cube";
            req.link_name_1 = "cube_link";
        }/*else if*/
        req.model_name_2 = "tiago";
        req.link_name_2 = "arm_7_link";

        //Send the request to the service
        gazebo_ros_link_attacher::AttachResponse resp;
        if (detach_srv.call(req, resp))
            ROS_INFO("Link detachment successful!");
        else
            ROS_ERROR("Failed to detach link");

    }/*detachRoutine*/

    //Function to open/close Tiago gripper
    void torso(const double height){
        //Create an action client to send trajectory goals to the gripper controller
        torso_control_client torso_client("/torso_controller/follow_joint_trajectory", true);

        //Wait for the action server to come up
        while(!torso_client.waitForServer(ros::Duration(5.0)))
            ROS_INFO("Waiting for the torso_controller action server to come up");

        //Create a FollowJointTrajectoryGoal message
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();

        //Set the joint names for the goal message
        goal.trajectory.joint_names.push_back("torso_lift_joint");

        goal.trajectory.points.resize(1);
        goal.trajectory.points[0].positions.push_back(height);
        goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

        //Send the goal to the action server
        torso_client.sendGoal(goal);

        // Wait for the action to return
        bool finished_before_timeout = torso_client.waitForResult(ros::Duration(5.0));

        if (finished_before_timeout){
            actionlib::SimpleClientGoalState state = torso_client.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }/*if*/
        else
            ROS_INFO("Action did not finish before the time out.");
    }/*torso*/

    //Function to open/close Tiago gripper
    void gripper(const std::vector<double>& joint_values){
        //Create an action client to send trajectory goals to the gripper controller
        gripper_control_client gripper_client("/gripper_controller/follow_joint_trajectory", true);

        //Wait for the action server to come up
        while(!gripper_client.waitForServer(ros::Duration(5.0)))
            ROS_INFO("Waiting for the gripper_controller action server to come up");

        //Create a FollowJointTrajectoryGoal message
        control_msgs::FollowJointTrajectoryGoal goal;

        //Set the joint names for the goal message
        goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
        goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

        //Create a JointTrajectoryPoint message
        trajectory_msgs::JointTrajectoryPoint point;

        //Set the joint values for the JointTrajectoryPoint message
        point.positions = joint_values;

        //Set the time stamp and duration for the JointTrajectoryPoint message
        point.time_from_start = ros::Duration(1.0);

        //Add the JointTrajectoryPoint message to the goal message
        goal.trajectory.points.push_back(point);

        //Set the goal tolerance
        goal.goal_tolerance.resize(2);
        for (unsigned int i = 0; i < 2; ++i){
            goal.goal_tolerance[i].name = goal.trajectory.joint_names[i];
            goal.goal_tolerance[i].position = 0.0;
            goal.goal_tolerance[i].velocity = 0.1;
            goal.goal_tolerance[i].acceleration = 0.1;
        }/*for*/

        //Send the goal to the action server
        gripper_client.sendGoal(goal);

        //Wait for the action to return
        bool finished_before_timeout = gripper_client.waitForResult(ros::Duration(5.0));

        if (finished_before_timeout){
            actionlib::SimpleClientGoalState state = gripper_client.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }/*if*/
        else
            ROS_INFO("Action did not finish before the time out.");
    }/*gripper*/

    //Function that make Tiago reach a pose with its arm moving in the cartesian space
    void moveArmTowardsPose(const assignment2::CGoalConstPtr &goal){
        //Create a move group for the Tiago robot
        moveit::planning_interface::MoveGroupInterface group_arm("arm");
        
        group_arm.setPlanningTime(1000.0);

        //Set the reference frame for the end-effector
        group_arm.setPlannerId("SBLkConfigDefault");
        group_arm.setPoseReferenceFrame("map");

        // Set the desired end-effector pose
        geometry_msgs::Pose target_pose;
        target_pose.position.x = goal->obj_pose[0];
        target_pose.position.y = goal->obj_pose[1];
        target_pose.position.z = goal->obj_pose[2];
        target_pose.orientation.x = 0.5;
        target_pose.orientation.y = 0.5;
        target_pose.orientation.z = 0.5;
        target_pose.orientation.w = 0.5;
        group_arm.setPoseTarget(target_pose);

        group_arm.setStartStateToCurrentState();
        group_arm.setMaxVelocityScalingFactor(0.2);

        //Define plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = static_cast<bool>(group_arm.plan(my_plan));

        if(!success)
            throw std::runtime_error("No plan found");
        //Plan and execute the motion
        group_arm.move();
    }/*moveArmTowardsPose*/

    //Function to open the gripper for a moveit_msgs::Grasp
    void openGripper(trajectory_msgs::JointTrajectory& posture){
        //Add both finger joints of panda robot
        posture.joint_names.resize(2);
        posture.joint_names[0] = "gripper_left_finger_joint";
        posture.joint_names[1] = "gripper_right_finger_joint";

        //Set them as open, wide enough for the object to fit
        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0.04;
        posture.points[0].positions[1] = 0.04;
        posture.points[0].time_from_start = ros::Duration(0.5);
    }/*openGripper*/

    //Routine to place an object with Tiago arm
    void placeRoutine(const assignment2::CGoalConstPtr &goal){
        //Manually found values to make Tiago arm achieve a waypoint configuration
        std::vector<double> waypoint_arm_joint_angles{1.30, 1.02, -0.14, 0.41, -1.57, 1.37, 0.00};

        //Move the arm to a waypoint configuration
        planJointSpace(waypoint_arm_joint_angles);

        //Move the arm to the final joint angles
        moveArmTowardsPose(goal);

        //Open the gripper
        gripper(open_gripper_joint_values);

        detachRoutine(goal);

        //Move the arm to a waypoint configuration
        planJointSpace(waypoint_arm_joint_angles);

        //Move the arm back to its initial configuration
        planJointSpace(resting_arm_joint_angles);

        result.task_completed = true;
    }/*placeRoutine*/

    //Function to reach, avoiding collision with the planning scene, an object to be picked with Tiago arm
    void planCartesianSpace(const assignment2::CGoalConstPtr &goal) {
        //Create a move group for the Tiago robot
        moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
        group_arm_torso.setEndEffectorLink("gripper_grasping_frame");

        //Prepare the grasp pose according to the pose of the object to pick
        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);
        tf2::Quaternion orientation(goal->obj_pose[3], goal->obj_pose[4], goal->obj_pose[5], goal->obj_pose[6]);
        float tau = 2 * M_PI;
        grasps[0].grasp_pose.header.frame_id = "map";


        tf2::Matrix3x3 mat(orientation);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        grasps[0].grasp_pose.pose.position.x = goal->obj_pose[0];
        grasps[0].grasp_pose.pose.position.y = goal->obj_pose[1];
        grasps[0].grasp_pose.pose.position.z = goal->obj_pose[2];

        //Slightly modify the pose given by the tag in order to correctly pick the object
        //One object is picked up laterally while the other two from above
        if(goal->tag_id == 1){
            grasps[0].grasp_pose.pose.position.y += 0.2;
            orientation.setRPY(roll -tau/4, pitch, -tau/4); //lateral pick
        }/*if*/
        else{
            grasps[0].grasp_pose.pose.position.z += 0.2;
            orientation.setRPY(roll, pitch + tau/4, yaw); //above pick
        }/*else*/

        grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

        //Setting pre-grasp approach
        grasps[0].pre_grasp_approach.direction.header.frame_id = "map";

        grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
        grasps[0].pre_grasp_approach.min_distance = 0.1;
        grasps[0].pre_grasp_approach.desired_distance = 0.15;

        //Setting posture before grasp
        openGripper(grasps[0].pre_grasp_posture);

        //Set support surface as table.
        group_arm_torso.setSupportSurfaceName("table");
        
        //Call pick to pick up the object using the grasps given
        std::string obj_name = "tag_";
        obj_name += std::to_string(goal->tag_id);
        group_arm_torso.pick(obj_name, grasps);

    }/*planCartesianSpace*/

    //Function to obtain a specific configuration for Tiago arm in the joint space
    void planJointSpace(std::vector<double> goal){

        //select group of joints
        moveit::planning_interface::MoveGroupInterface group_arm("arm");
        //choose your preferred planner
        group_arm.setPlannerId("SBLkConfigDefault");

        std::map<std::string, double> target_position;
        target_position["arm_1_joint"] = goal[0];
        target_position["arm_2_joint"] = goal[1];
        target_position["arm_3_joint"] = goal[2];
        target_position["arm_4_joint"] = goal[3];
        target_position["arm_5_joint"] = goal[4];
        target_position["arm_6_joint"] = goal[5];
        target_position["arm_7_joint"] = goal[6];

        std::vector<std::string> arm_joint_names;
        arm_joint_names = group_arm.getJoints();

        group_arm.setStartStateToCurrentState();
        group_arm.setMaxVelocityScalingFactor(0.2);
        for (unsigned int i = 0; i < arm_joint_names.size(); ++i)
            group_arm.setJointValueTarget(arm_joint_names[i], target_position[arm_joint_names[i]]);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        group_arm.setPlanningTime(1000.0);

        bool success = static_cast<bool>(group_arm.plan(my_plan));
        if (!success)
            throw std::runtime_error("No plan found");

        //Plan and execute the motion
        group_arm.move();
    }/*planJointSpace*/

    //Routine to pick an object with Tiago arm
    void pickRoutine(const assignment2::CGoalConstPtr &goal){

        //Reach a waypoint
        planJointSpace(waypoint_arm_joint_angles[goal->tag_id -1]);

        //Reach the object
        planCartesianSpace(goal);

        //Close the gripper
        gripper(closed_gripper_joint_values);

        //Make sure the object moves along with the arm
        attachRoutine(goal);

        //Return to the waypoint
        planJointSpace(waypoint_arm_joint_angles[goal->tag_id - 1]);

        //Safe arm configuration while moving
        planJointSpace(resting_arm_joint_angles);

        //Default value for Tiago torso joint
        torso(0.15);

        result.task_completed = true;
    }/*pickRoutine*/

    //Main callback that does all the work reserved to this action server
    void executeCB(const assignment2::CGoalConstPtr &goal){
        result.task_completed = false;
        ros::Rate loop_rate(1);
        while(!result.task_completed){
            if(as.isNewGoalAvailable()) 
                as.acceptNewGoal();
            if(goal->pick_place_flag){
                feedback.current_action = "PLACING OBJECT";
                as.publishFeedback(feedback);
                placeRoutine(goal);
            }/*if*/
            else{
                feedback.current_action = "PICKING OBJECT";
                as.publishFeedback(feedback);
                pickRoutine(goal);
            }/*else*/
            loop_rate.sleep();
        }/*while*/

        //Sending the result to the client
        ROS_INFO("%s: Succeeded", action_name.c_str());
        as.setSucceeded(result);

        result.task_completed = false;
    }/*executeCB*/
};

int main(int argc, char** argv){
    ros::init(argc, argv, "node_C");
    CAction C("C");
    ros::spin();
    return 0;
}/*main*/