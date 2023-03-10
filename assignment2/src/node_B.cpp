//Written and revised by Paolo Bresolin, Giacomo Gonella and Pietro Picardi
#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/shared_ptr.hpp>
#include <assignment2/BAction.h>
#include <assignment2/CAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_control_client;

class BAction{
    protected:
        ros::NodeHandle nh;
        actionlib::SimpleActionServer<assignment2::BAction> as;
        std::string action_name;
        assignment2::BResult result;
        assignment2::BFeedback feedback;
        bool found = false;
        bool detection_done = false;
        const int *target;
    private:
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
    public:
        //Constructor and destructor
        BAction(std::string name) : tfListener(tfBuffer), as(nh, name, boost::bind(&BAction::executeCB, this, _1), false), action_name(name){as.start();}
        ~BAction(void) {}

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

        //Function to create a box shape for the collision object depending on its shape
        shape_msgs::SolidPrimitive createBox(int id){
            shape_msgs::SolidPrimitive box;
            //Blue hexagon
            if(id == 1){
                box.type = box.CYLINDER;
                box.dimensions.resize(2);
                box.dimensions[0] = 0.1; //height
                box.dimensions[1] = 0.04; //radius
            }/*if*/

            //Green triangle
            else if(id == 2){
                box.type = box.CONE;
                box.dimensions.resize(2);
                box.dimensions[0] = 0.05; //height
                box.dimensions[1] = 0.04; //radius
            }/*else if*/

            //Red cube
            else if(id == 3){
                box.type = box.BOX;
                box.dimensions.resize(3);
                box.dimensions[0] = 0.05; //x
                box.dimensions[1] = 0.05; //y
                box.dimensions[2] = 0.05; //z
            }/*else if*/

            //Table
            else if(id == 0){
                box.type = box.BOX;
                box.dimensions.resize(3);
                box.dimensions[0] = 1.00;//x
                box.dimensions[1] = 1.00;//y
                box.dimensions[2] = 0.1; //z
            }

            //Gold hexagon
            else{
                box.type = box.CYLINDER;
                box.dimensions.resize(2);
                box.dimensions[0] = 0.3; //height
                box.dimensions[1] = 0.05; //radius
            }/*else*/

            return box;
        }/*createBox*/

        //Function to set the joint values of the head so that tags are properly visible by the camera
        void setHeadJointValues(const std::vector<double>& joint_values){
            //Create an action client to send trajectory goals to the head controller
            head_control_client head_client("/head_controller/follow_joint_trajectory", true);

            //Wait for the action server to come up
            while(!head_client.waitForServer(ros::Duration(5.0)))
                ROS_INFO("Waiting for the head_controller action server to come up");

            //Create a FollowJointTrajectoryGoal message
            control_msgs::FollowJointTrajectoryGoal goal;

            //Set the joint names for the goal message
            goal.trajectory.joint_names.push_back("head_1_joint");
            goal.trajectory.joint_names.push_back("head_2_joint");

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
                goal.goal_tolerance[i].position = 0.01;
                goal.goal_tolerance[i].velocity = 0.1;
                goal.goal_tolerance[i].acceleration = 0.1;
            }/*for*/

            //Send the goal to the action server
            head_client.sendGoal(goal);

            //Wait for the action to return
            bool finished_before_timeout = head_client.waitForResult(ros::Duration(5.0));

            if (finished_before_timeout){
                actionlib::SimpleClientGoalState state = head_client.getState();
                ROS_INFO("Action finished: %s", state.toString().c_str());
            }/*if*/
            else
                ROS_INFO("Action did not finish before the time out.");
        }/*setHeadJointValues*/

        //Callback function for the AprilTag detections topic: detects the visible tags and create an appropriate planning scene.
        //When the object to be picked is found, call node_C to pick it.
        void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg){
            //ARM CLIENT
            actionlib::SimpleActionClient<assignment2::CAction> ac("C", true);
            //Wait for the action server to come up
            while (!ac.waitForServer(ros::Duration(5.0)))
                ROS_INFO("Waiting for the action server to come up");
            //Arm goal
            assignment2::CGoal arm_goal;
            //Arm result
            assignment2::CResultConstPtr arm_result;

            //Connect to the planning scene
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            std::vector<moveit_msgs::CollisionObject> objects;
            //Table collision object
            moveit_msgs::CollisionObject table;
            table.header.frame_id = "map";
            table.id = "table";
            table.operation = table.ADD;
            geometry_msgs::Pose table_pose;
            table_pose.position.x = 7.701592163;
            table_pose.position.y = -2.940211463;
            table_pose.position.z = 0.75;
            table_pose.orientation.x = 0.0;
            table_pose.orientation.y = 0.0;
            table_pose.orientation.z = -0.693372783364;
            table_pose.orientation.w = 0.720579061096;
            table.primitive_poses.push_back(table_pose);
            table.primitives.push_back(createBox(0));
            objects.push_back(table);
            //Add the table to the planning scene
            planning_scene_interface.applyCollisionObjects({table});

            //Iterate over the detections: a message can contain multiple ones
            for (const auto& detection : msg->detections){
                if(detection_done)
                    break;

                //A detection contains several tags: we search the one corresponding to the object we want to pick
                for(int i = 0; i < detection.id.size(); i++) {
                    //Define a collision object for each tag and add it to the planning scene
                    moveit_msgs::CollisionObject collision_object;
                    collision_object.header.frame_id = "map";
                    collision_object.id = "tag_";
                    collision_object.operation = collision_object.ADD;
                    shape_msgs::SolidPrimitive box = createBox(detection.id[i]);
                    geometry_msgs::Pose obj_transformed_pose = transformPoseToAbsoluteFrame(detection.pose.pose.pose);

                    //The green triangle tag is not parallel to the ground: so to appropriately set its collision object
                    //we need to modify its orientation
                    if(detection.id[i] == 2) {
                        obj_transformed_pose.orientation.x = 0.0;
                        obj_transformed_pose.orientation.y = 0.0;
                    }/*if*/

                    collision_object.primitive_poses.push_back(obj_transformed_pose);
                    collision_object.id += std::to_string(detection.id[i]);
                    collision_object.primitives.push_back(box);
                    objects.push_back(collision_object);
                    //ROS_INFO("Tag detected: %d, Tag sought: %d", detection.id[index], *target);
                    if (detection.id[i] == *target) {

                        arm_goal.obj_pose[0] = obj_transformed_pose.position.x;
                        arm_goal.obj_pose[1] = obj_transformed_pose.position.y;
                        arm_goal.obj_pose[2] = obj_transformed_pose.position.z;
                        arm_goal.obj_pose[3] = obj_transformed_pose.orientation.x;
                        arm_goal.obj_pose[4] = obj_transformed_pose.orientation.y;
                        arm_goal.obj_pose[5] = obj_transformed_pose.orientation.z;
                        arm_goal.obj_pose[6] = obj_transformed_pose.orientation.w;

                        //Fill the detection result to send back to node_A
                        result.obj_pose = arm_goal.obj_pose;

                        arm_goal.tag_id = *target;
                        arm_goal.pick_place_flag = false;

                        found = true;
                    }/*if*/

                    //Add the collision object to the planning scene
                    planning_scene_interface.applyCollisionObjects({collision_object});
                }/*for*/
            }/*for*/

            //CALL ARM SERVER
            if(!detection_done) {
                ROS_INFO("Sending arm goal");
                ac.sendGoal(arm_goal, boost::bind(&BAction::doneNode_C, this, _1, _2),
                            boost::bind(&BAction::activeNode_C, this),
                            boost::bind(&BAction::feedbackNode_C, this, _1));
                ac.waitForResult(ros::Duration(120.0));
                arm_result = ac.getResult();
                ROS_INFO("Was tag_%d picked? %d", arm_goal.tag_id, arm_result->task_completed);
                ac.cancelGoal();
            }/*if*/

            //Clear the planning scene
            for (moveit_msgs::CollisionObject &obj: objects)
                obj.operation = obj.REMOVE;
            planning_scene_interface.applyCollisionObjects(objects);

            detection_done = true;
        }/*tagDetectionCallback*/

        //Main callback that does all the work reserved to this action server
        void executeCB(const assignment2::BGoalConstPtr &goal){
            //Adjust Tiago head orientation
            std::vector<double> head_joint_values {-0.15, -0.27};
            setHeadJointValues(head_joint_values);

            //Call for tag detection
            ros::Rate loop_rate(1);
            while(!found){
                if(as.isNewGoalAvailable())
                    as.acceptNewGoal();
                target = &(goal->id);
                feedback.current_action = "DETECTING OBJECTS";
                as.publishFeedback(feedback);
                //Subscribe to the AprilTag detection topic
                ros::Subscriber tag_detection_sub_ = nh.subscribe("/tag_detections", 1, &BAction::tagDetectionCallback, this);

                loop_rate.sleep();
            }/*while*/
            found = false;
            detection_done = false;

            //Sending the result to the client
            ROS_INFO("%s: Succeeded", action_name.c_str());
            as.setSucceeded(result);
        }/*executeCB*/

        //Function that returns the pose in the absolute reference frame given the corresponding pose in the camera reference frame
        geometry_msgs::Pose transformPoseToAbsoluteFrame(geometry_msgs::Pose pose) {
            geometry_msgs::Pose transformedPose;

            geometry_msgs::TransformStamped transformStamped;
            try {
                transformStamped = tfBuffer.lookupTransform("map", "xtion_rgb_optical_frame", ros::Time(0));
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
};

int main(int argc, char** argv){
    ros::init(argc, argv, "node_B");
    BAction B("B");
    ros::spin();
    return 0;
}/*main*/

