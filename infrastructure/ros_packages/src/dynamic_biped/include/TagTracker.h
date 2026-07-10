#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include "PoseTransformer.h"
#include "dynamic_biped/robotHeadMotionData.h"

namespace autoHeadChase {

class TagTracker {
public:
    TagTracker();
    void run();  // Runs the ROS node

private:
    // Callback functions for subscribers
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void tagInfoCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

    // Service handlers
    bool oneTimeTrackService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool continuousTrackService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    // Internal functions
    void updateTagWorldPose();
    void calculateHeadOrientation();
    void publishHeadOrientationCommand(double pitch, double yaw);

    // Node handles, subscribers, and publishers
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber tag_info_sub_;
    ros::Publisher head_orientation_pub_;
    ros::ServiceServer one_time_track_srv_;
    ros::ServiceServer continuous_track_srv_;
    ros::Publisher tag_world_pose_pub_;


    // State variables
    Eigen::VectorXd robot_pose_world_;  // Robot pose in the world frame (7-dim: pos + quat)
    Eigen::VectorXd tag_pose_robot_;    // Tag pose in the robot frame (7-dim: pos + quat)
    Eigen::VectorXd tag_pose_world_;    // Tag pose in the world frame (7-dim: pos + quat)

    bool is_continuous_tracking_;  // Flag to manage continuous tracking
    ros::Timer tracking_timer_;    // Timer for continuous tracking
    double pitch0  = 0.6105555;
};

}  // namespace autoHeadChase
