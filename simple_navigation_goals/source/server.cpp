//Written and revised by Paolo Bresolin, Giacomo Gonella and Pietro Picardi
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <simple_navigation_goals/tiagoAction.h>
#include "sensor_msgs/LaserScan.h"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <math.h>

//A simple class to represent a circular object
class Circle {
    public:
        Circle(double x, double y, double radius) : x_(x), y_(y), radius_(radius) {}

        double x_; //x coordinate of the center of the circle
        double y_; //y coordinate of the center of the circle
        double radius_; //radius of the circle
}; /*Circle*/

class tiagoAction {
    protected:
        bool finished = false;
        //Some variables used to control the robot in the narrow corridor
        float g_range_left = 0.0;
        float g_range_center = 0.0;
        float g_range_right = 0.0;
        //Other variables used by the action server
        ros::NodeHandle nh;
        actionlib::SimpleActionServer<simple_navigation_goals::tiagoAction> as;
        std::string action_name;
        simple_navigation_goals::tiagoFeedback feedback;
        simple_navigation_goals::tiagoResult result;
    public:
        //Constructor and destructor
        tiagoAction(std::string name) : as(nh, name, boost::bind(&tiagoAction::executeCB, this, _1), false), action_name(name){as.start();}
        ~tiagoAction(void) {}

        //Return the mean value of a vector of float
        float meanFloat(const std::vector<float>& vector) {
            float sum = 0;
            for (int i = 0; i < vector.size(); i++)
                sum += vector[i];
            return sum / vector.size();
        } /*meanFloat*/

        /*
        Given three points $p1, $p2 and $p3, it computes the center ($center) and the radius ($radius)
        of the interpolated circle to which they belong
        */
        void getCircle(const cv::Point2d p1, const cv::Point2d p2, const cv::Point2d p3, cv::Point2d& center, double& radius) {
            //Compute the lengths of the sides of the triangle formed by the three points
            double a = cv::norm(p1 - p2);
            double b = cv::norm(p2 - p3);
            double c = cv::norm(p3 - p1);

            //Compute the semi-perimeter of the triangle
            double s = (a + b + c) / 2;

            //Compute the area of the triangle
            double area = std::sqrt(s * (s - a) * (s - b) * (s - c));

            //Compute the radius of the circumcircle
            radius = a * b * c / (4 * area);

            //Compute the center of the circumcircle
            double x = (a * p1.x + b * p2.x + c * p3.x) / (a + b + c);
            double y = (a * p1.y + b * p2.y + c * p3.y) / (a + b + c);
            center = cv::Point2d(x, y);
        } /*getCircle*/

        //A callback function that processes incoming laser scan data to detect circular obstacles
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
            //The robot is not moving, the goal has been reached and the detection is running
            feedback.current_action = "DETECTING OBSTACLES";
            as.publishFeedback(feedback);

            //Create a vector of cartesian points by transforming polar coordinates into cartesian ones
            std::vector<cv::Point2d> points(scan->ranges.size());
            double current_angle = scan->angle_min;
            for (int i = 0; i < points.size(); i++) {
                points[i].x = scan->ranges[i] * cos(current_angle);
                points[i].y = scan->ranges[i] * sin(current_angle);
                current_angle += scan->angle_increment;
            } /*for*/

            //Clustering of points based on their distance from the robot (vector $scan->ranges)
            //Vector containing clusters, which are vectors of points
            std::vector<std::vector<cv::Point2d>> clusters;
            //Threshold for delta radius: we want a high delta to pass from cluster to cluster
            double max_dist = 0.8;
            std::vector<cv::Point2d> current_cluster;
            int index = 0;
            //We don't want points with a too low range
            while(scan->ranges[index] <= 0.3)
                index++;
            //Initialization of the first cluster with the first point that is not too close
            current_cluster.push_back(points[index]);
            //Clustering
            for (int i = index; i < scan->ranges.size(); i++) {
                //Check if we have reached a new cluster of points
                if (abs(scan->ranges[i] - scan->ranges[i - 1]) > max_dist) {
                    if (current_cluster.size() > 5) //we save only clusters with more than 5 points
                        clusters.push_back(current_cluster);
                    current_cluster.clear();
                } /*if*/
                //We don't want points with a too low range, so we check before adding to the current cluster
                if (scan->ranges[i] > 0.3) {
                    current_cluster.push_back(points[i]);
                } /*if*/
            } /*for*/

            //Try to fit a circle for each cluster by looking at three points per cluster
            //Vector of circles: it will contain the circles representing the detected obstacles
            std::vector<Circle> circles;
            //Analyze each cluster
            for (int i = 0; i < clusters.size(); i++){
                //Fit a circle to the cluster
                cv::Point2d center;
                double r;
                //To construct the circle we use the first, the middle and the last points of the cluster
                getCircle(clusters[i][0], clusters[i][floor(clusters[i].size() / 2)],
                          clusters[i][clusters[i].size() - 1], center, r);

                //Check if the radius is large enough, but also not too large
                if (r >= 0.1 && r <= 0.5) {
                    Circle circle = Circle(center.x,center.y,r);
                    //The radius is ok, so the cluster represents an obstacle: add the circle to the vector
                    circles.push_back(circle);
                } /*if*/
            } /*for*/

            //Publish the centers of the detected circles
            for (const auto& circle : circles) {
                result.objs_locations_x_coord.push_back(circle.x_);
                result.objs_locations_y_coord.push_back(circle.y_);
            } /*for*/

            finished = true;
        } /*scanCallback*/

        /*
        Callback function for the laser scan data used while in the narrow corridor to apply the control law
        The main reasons for doing the following computations are two:
            ~ find the ranges at the right desired angles;
            ~ collect more than one range per angle in order to be less sensitive to noise.
        */
        void laserNarrowCorridor(const sensor_msgs::LaserScan::ConstPtr& msg) {
            //We collect $n_mean points in each vector
            int n_mean = 5;
            std::vector<float> ranges_left; //vector of range values around 90° w.r.t. the x axis of the robot's r.f.
            std::vector<float> ranges_center; //vector of range values around 0° w.r.t. the x axis of the robot's r.f.
            std::vector<float> ranges_right; //vector of range values around -90° w.r.t. the x axis of the robot's r.f.

            //Define some constant angles
            float pi = (float) M_PI;
            float angle_90 = pi / 2;
            float angle_minus_90 = - angle_90;
            /*
            ~ right_center_left[0] states if we have already collected $n_mean points in $ranges_right;
            ~ right_center_left[1] states if we have already collected $n_mean points in $ranges_center;
            ~ right_center_left[2] states if we have already collected $n_mean points in $ranges_left.
            At the beginning they are false because the vectors are empty
            */
            std::vector<bool> right_center_left = {false, false, false};

            //Filling of the three vectors $ranges_right, $ranges_center, $ranges_left
            for (int i = 0; i < msg->ranges.size(); i++) {
                //Angle associated with the current range value $msg->ranges[i]
                float current_angle = msg->angle_min + i * msg->angle_increment;
                //Check whether the current angle is near to -90° and $ranges_right is still empty
                if ((abs(angle_minus_90 - current_angle) < (2 * msg->angle_increment)) && (!right_center_left[0])) {
                    //Fill $ranges_right with $n_mean range values around -90°
                    for (int j = 0; j < n_mean; j++)
                        ranges_right.push_back(msg->ranges[i + j]);
                    //Update the current index based on the number of already visited ranges
                    i += n_mean - 1;
                    //Set to true because $ranges_right is now completely full
                    right_center_left[0] = true;
                } /*if*/
                //Check whether the current angle is near to 0° and $ranges_center is still empty
                else if ((abs(current_angle) < (2 * msg->angle_increment)) && (!right_center_left[1])) {
                    //Fill $ranges_center with $n_mean range values around 0°
                    for (int j = - (n_mean / 2); j < (n_mean / 2); j++)
                        ranges_center.push_back(msg->ranges[i + j]);
                    //Update the current index based on the number of already visited ranges
                    i += (n_mean / 2) - 1;
                    //Set to true because $ranges_center is now completely full
                    right_center_left[1] = true;
                } /*else if*/
                //Check whether the current angle is near to 90° and $ranges_left is still empty
                else if ((abs(angle_90 - current_angle) < (2 * msg->angle_increment)) && (!right_center_left[2])) {
                    //Fill $ranges_left with $n_mean range values around 90°
                    for (int j = 0; j < n_mean; j++)
                        ranges_left.push_back(msg->ranges[i + j]);
                    //Update the current index based on the number of already visited ranges
                    i += n_mean - 1;
                    //Set to true because $ranges_center is now completely full
                    right_center_left[2] = true;
                    //All the three vectors are filled up, so it is useless to go on
                    break;
                } /*else if*/
            } /*for*/
            
            //Assign the three class variables with the mean value of each vector of ranges
            g_range_left = meanFloat(ranges_left);
            g_range_center = meanFloat(ranges_center);
            g_range_right = meanFloat(ranges_right);
        } /*laserNarrowCorridor*/

        //Function for passing through the initial narrow corridor
        int narrowCorridor() {
            //Update the status of the robot: the robot starts moving
            feedback.current_action = "MOVING IN A NARROW CORRIDOR";
            as.publishFeedback(feedback);

            //Create a publisher for the robot's velocity commands
            ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 100);

            //Create a subscriber to the laser scan data
            ros::Subscriber laser_sub = nh.subscribe("/scan", 10, &tiagoAction::laserNarrowCorridor, this);

            //Set the loop rate in Hz
            ros::Rate loop_rate(10);

            while (ros::ok()) {

                //Create a Twist message to store the velocity commands
                geometry_msgs::Twist cmd_vel;

                //Check if we are out from the narrow corridor: $g_range_left or g_range_right start becoming large
                if ((g_range_left > 3) || (g_range_right > 3)) {
                    //The corridor is finished, so we return to the next step of executeCB()
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    return 0;
                } /*if*/
                //We are navigating through the corridor
                else {
                    //Set the linear velocity to a constant value
                    cmd_vel.linear.x = 0.5;
                    //Set the angular velocity to zero
                    cmd_vel.angular.z = 0.0;
                } /*else*/

                //Publish the velocity commands
                cmd_vel_pub.publish(cmd_vel);
                //Spin and sleep to maintain the loop rate
                ros::spinOnce();
                loop_rate.sleep();
            } /*while*/
            return 0;
        } /*narrowCorridor*/

        //Function to make tiago reach the pose specified by the user
        void moveTiago(const simple_navigation_goals::tiagoGoalConstPtr &target) {
            //Act as an action client connected to move_base
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

            //Wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0)))
                ROS_INFO("Waiting for the move_base action server to come up");

            //Specification of the goal to be sent to the server
            move_base_msgs::MoveBaseGoal goal;
            //We specify the final pose w.r.t. the absolute reference frame
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = target->final_position[0];
            goal.target_pose.pose.position.y = target->final_position[1];
            goal.target_pose.pose.position.z = target->final_position[2];
            goal.target_pose.pose.orientation.x = target->final_position[3];
            goal.target_pose.pose.orientation.y = target->final_position[4];
            goal.target_pose.pose.orientation.z = target->final_position[5];
            goal.target_pose.pose.orientation.w = target->final_position[6];
            //Send the goal to the server
            ac.sendGoal(goal);


            //While moving, the server sends feedbacks to the client, who then prints them
            while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
                continue;
            //The robot reached the final pose
            //Update the status of the robot: the goal has been reached and the robot is now stopped
            feedback.current_action = "MOVING";
            as.publishFeedback(feedback);
        } /*moveTiago*/

        //Main callback that does all the work reserved to this action server
        void executeCB(const simple_navigation_goals::tiagoGoalConstPtr &goal) {
            ros::Rate loop_rate(1);
            if(as.isNewGoalAvailable())
                as.acceptNewGoal();
            while(!finished) {

                //Passing through the initial narrow corridor
                narrowCorridor();

                //Now tiago reaches the user defined pose
                moveTiago(goal);

                //Final pose reached: now we start the obstacle detection
                //Update the status of the robot: the goal has been reached and the detection has been started
                //Subscription to the laser scanner /scan topic and obstacle detection with scanCallback()
                ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &tiagoAction::scanCallback, this);

                loop_rate.sleep();
            }
            finished = false;
            //Sending the result to the client
            ROS_INFO("%s: Succeeded", action_name.c_str());
            as.setSucceeded(result);
        } /*executeCB*/
};

int main(int argc, char** argv){
    ros::init(argc, argv, "tiago_server");
    tiagoAction tiago("tiago");
    ros::spin();
    return 0;
} /*main*/