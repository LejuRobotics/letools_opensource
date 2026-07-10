#include "TagTracker.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace autoHeadChase {

TagTracker::TagTracker()
    : is_continuous_tracking_(false),
      robot_pose_world_(Eigen::VectorXd::Zero(7)),
      tag_pose_robot_(Eigen::VectorXd::Zero(7)),
      tag_pose_world_(Eigen::VectorXd::Zero(7)) {

    // Initialize subscribers
    odom_sub_ = nh_.subscribe("/odom", 10, &TagTracker::odomCallback, this);
    tag_info_sub_ = nh_.subscribe("/robot_tag_info", 10, &TagTracker::tagInfoCallback, this);

    // Initialize publishers
    head_orientation_pub_ = nh_.advertise<dynamic_biped::robotHeadMotionData>("/robot_head_motion_data", 10);
    tag_world_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/tag_world_pose", 10);  // New publisher for tag world pose

    // Initialize services
    one_time_track_srv_ = nh_.advertiseService("one_time_track", &TagTracker::oneTimeTrackService, this);
    continuous_track_srv_ = nh_.advertiseService("continuous_track", &TagTracker::continuousTrackService, this);
}

void TagTracker::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_pose_world_(0) = msg->pose.pose.position.x;
    robot_pose_world_(1) = msg->pose.pose.position.y;
    robot_pose_world_(2) = msg->pose.pose.position.z;
    robot_pose_world_(3) = msg->pose.pose.orientation.x;
    robot_pose_world_(4) = msg->pose.pose.orientation.y;
    robot_pose_world_(5) = msg->pose.pose.orientation.z;
    robot_pose_world_(6) = msg->pose.pose.orientation.w;

    updateTagWorldPose();
}

void TagTracker::tagInfoCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    if (!msg->detections.empty()) {
        const auto& tag_pose = msg->detections[0].pose.pose.pose;
        tag_pose_robot_(0) = tag_pose.position.x;
        tag_pose_robot_(1) = tag_pose.position.y;
        tag_pose_robot_(2) = tag_pose.position.z;
        tag_pose_robot_(3) = tag_pose.orientation.x;
        tag_pose_robot_(4) = tag_pose.orientation.y;
        tag_pose_robot_(5) = tag_pose.orientation.z;
        tag_pose_robot_(6) = tag_pose.orientation.w;

        // std::cout << "tag_pose_robot_ : " << tag_pose_robot_ << std::endl;

        updateTagWorldPose();
    }
}

void TagTracker::updateTagWorldPose() {
    tag_pose_world_ = PoseTransformer::transformPoseToWorld(tag_pose_robot_, robot_pose_world_);
    geometry_msgs::Pose tag_world_pose;
    tag_world_pose.position.x = tag_pose_world_(0);
    tag_world_pose.position.y = tag_pose_world_(1);
    tag_world_pose.position.z = tag_pose_world_(2);
    tag_world_pose.orientation.x = tag_pose_world_(3);
    tag_world_pose.orientation.y = tag_pose_world_(4);
    tag_world_pose.orientation.z = tag_pose_world_(5);
    tag_world_pose.orientation.w = tag_pose_world_(6);

    tag_world_pose_pub_.publish(tag_world_pose);

    // std::cout << "tag_pose_world_ : " << tag_pose_world_ << std::endl;
}

void TagTracker::calculateHeadOrientation() {
    double dx = tag_pose_world_(0) - robot_pose_world_(0);
    double dy = tag_pose_world_(1) - robot_pose_world_(1);
    double dz = tag_pose_world_(2) - robot_pose_world_(2);

    double yaw = atan2(dy, dx);

    double dl = sqrt(dx * dx + dy * dy + dz * dz);
    double dl_xy = sqrt(dx * dx + dy * dy);

    double head_link = 0.05449;
    double alpha1 = acos(head_link/dl);
    double alpha2 = asin(dl_xy/dl);
    // double pitch = atan2(dz, sqrt(dx * dx + dy * dy));
    double pitch = M_PI - pitch0 - alpha1 - alpha2;

    std::cout << "[Tag] Head pitch0 : " << pitch0 * 180 / M_PI << std::endl;
    std::cout << "[Tag] Head alpha1 : " << alpha1 * 180 / M_PI << std::endl;
    std::cout << "[Tag] Head alpha2 : " << alpha2 * 180 / M_PI << std::endl;


    // Normalize yaw and pitch to be within [-pi, pi]
    yaw = std::fmod(yaw + M_PI, 2 * M_PI) - M_PI;
    pitch = std::fmod(pitch + M_PI, 2 * M_PI) - M_PI;

    publishHeadOrientationCommand(pitch, yaw);
}

void TagTracker::publishHeadOrientationCommand(double pitch, double yaw) {
    // Ensure yaw is within the range [-30, 30]
    pitch *= 180 / M_PI;
    yaw *= 180 / M_PI;

    std::cout << "[Tag] Head pitch : " << pitch << std::endl;
    std::cout << "[Tag] Head yaw : " << yaw << std::endl;

    yaw = std::max(-30.0, std::min(30.0, yaw));

    // Ensure pitch is within the range [-25, 25]
    pitch = std::max(-25.0, std::min(25.0, pitch));

    // Create and populate the robotHeadMotionData message
    dynamic_biped::robotHeadMotionData head_cmd;
    head_cmd.joint_data.resize(2);  // Set size for two joints
    head_cmd.joint_data[0] = yaw;   // yaw in degrees
    head_cmd.joint_data[1] = pitch; // pitch in degrees

    // Publish the command
    head_orientation_pub_.publish(head_cmd);
}

bool TagTracker::oneTimeTrackService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    if (req.data) {
        calculateHeadOrientation();
        res.success = true;
        res.message = "One-time tracking executed.";
    } else {
        res.success = false;
        res.message = "Invalid request for one-time tracking.";
    }
    return true;
}

bool TagTracker::continuousTrackService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    if (req.data && !is_continuous_tracking_) {
        is_continuous_tracking_ = true;
        tracking_timer_ = nh_.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent&) {
            calculateHeadOrientation();
        });
        res.success = true;
        res.message = "Continuous tracking started.";
    } else if (!req.data && is_continuous_tracking_) {
        is_continuous_tracking_ = false;
        tracking_timer_.stop();
        res.success = true;
        res.message = "Continuous tracking stopped.";
    } else {
        res.success = false;
        res.message = "Invalid request for continuous tracking.";
    }
    return true;
}

// Run the ROS node
void TagTracker::run() {
    ros::spin();
}

}  // namespace autoHeadChase

// Main function to start the TagTracker node
int main(int argc, char** argv) {
    ros::init(argc, argv, "tag_tracker_node");
    autoHeadChase::TagTracker tag_tracker;
    tag_tracker.run();
    return 0;
}
