#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_biped/sensorsData.h>
#include <thread>
#include <mutex>
#include <cmath>

class JointStateRepublisher {
public:
    JointStateRepublisher(ros::NodeHandle& nh) {
        // 初始化 Publisher，发布到 /joint_states
        joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

        // 初始化 Subscriber，订阅 /head_sensors_data_raw
        head_sensor_data_sub_ = nh.subscribe("/sensors_data_raw", 10, &JointStateRepublisher::headSensorDataCallback, this);

        // 启动发布线程
        publish_thread_ = std::thread(&JointStateRepublisher::publishLoop, this);
    }

    ~JointStateRepublisher() {
        // 退出时确保线程结束
        if (publish_thread_.joinable()) {
            publish_thread_.join();
        }
    }

private:
    ros::Publisher joint_state_pub_;
    ros::Subscriber head_sensor_data_sub_;
    std::thread publish_thread_;
    std::mutex data_mutex_;

    sensor_msgs::JointState latest_joint_state_;
    bool data_received_ = false;

    // 角度转弧度的辅助函数
    double degreeToRadian(double degree) {
        return degree * (M_PI / 180.0);
    }

    // 回调函数，用于处理 /head_sensors_data_raw 数据
    void headSensorDataCallback(const dynamic_biped::sensorsData::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
    if (msg->joint_data.joint_q.size() == 28) {
        
        latest_joint_state_.header.stamp = ros::Time::now();

        // Keep only the last two elements in the position vector
        latest_joint_state_.position = {msg->joint_data.joint_q[26], msg->joint_data.joint_q[27]};
        latest_joint_state_.name = {"head_yaw", "head_pitch"};


        // Convert angles to radians and invert sign
        // for (double& position : latest_joint_state_.position) {
        //     position = degreeToRadian(position);
        //     position = -position;
        // }

        data_received_ = true;
    } else {
        ROS_WARN("Received data does not have 28 elements in the position vector. Ignoring message.");
    }
    }

    // 发布线程，持续发布数据
    void publishLoop() {
        ros::Rate rate(10);  // 发布频率为 10Hz
        while (ros::ok()) {
            sensor_msgs::JointState joint_state_msg;

            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (data_received_) {
                    joint_state_msg = latest_joint_state_;  // 使用接收到的最新数据（已转为弧度）
                } else {
                    // 未接收到数据时，使用默认值
                    joint_state_msg.header.stamp = ros::Time::now();
                    joint_state_msg.name = {"head_yaw", "head_pitch"};
                    joint_state_msg.position = {0.0, 0.0};  // 默认值为0.0弧度
                    joint_state_msg.velocity = {0.0, 0.0};
                    joint_state_msg.effort = {0.0, 0.0};
                }
            }

            joint_state_pub_.publish(joint_state_msg);
            // ROS_INFO("Published joint states to /joint_states");

            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "joint_state_republisher");
    ros::NodeHandle nh;

    // 创建 JointStateRepublisher 对象
    JointStateRepublisher republisher(nh);

    // 循环等待回调
    ros::spin();

    return 0;
}
