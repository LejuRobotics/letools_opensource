#! /usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, PoseStamped  
import tf2_ros
import tf2_geometry_msgs
import copy
from geometry_msgs.msg import TransformStamped, PoseStamped


class ARControlNode(object):
    def __init__(self, target_frame='base_link', source_frame='camera_color_optical_frame'):
        rospy.init_node('ar_control_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.publisher = rospy.Publisher('/robot_tag_info', AprilTagDetectionArray, queue_size=10)
        self.publisher_odom = rospy.Publisher('/robot_tag_info_odom', AprilTagDetectionArray, queue_size=10)
        self.subscription = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.target_frame = target_frame  # 指定目标坐标系
        self.source_frame = source_frame # 指定源坐标系

        rospy.loginfo('\033[32m' + f"[ARControlNode] target_frame: {self.target_frame}, source_frame: {self.source_frame}" + '\033[0m')

    def tag_callback(self, msg):
        new_msg = AprilTagDetectionArray()
        new_msg.header = msg.header  # 保持消息头一致
        new_msg.header.frame_id = self.target_frame # 使用设定的目标坐标系

        new_msg_odom = AprilTagDetectionArray()
        new_msg_odom.header.stamp = rospy.Time.now()
        new_msg_odom.header.frame_id = 'odom'

        for detection in msg.detections:
            # 使用 PoseStamped 简化转换
            pose_stamped = PoseStamped()
            pose_stamped.header = detection.pose.header
            if pose_stamped.header.frame_id == '':
                pose_stamped.header.frame_id = self.source_frame # 使用设定的源坐标系
            pose_stamped.pose = detection.pose.pose.pose

            try:
                # 等待转换可用，设置超时时间
                transform = self.tf_buffer.lookup_transform(self.target_frame, pose_stamped.header.frame_id, rospy.Time(0)) 
                transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

                # 更新 detection 的位姿
                detection.pose.pose.pose = transformed_pose.pose
                detection.pose.header.frame_id = self.target_frame

                new_msg.detections.append(detection)

                # 广播 tf 变换
                self.broadcast_transform(transformed_pose, detection.id)


                # 转换到odom坐标系
                try:
                    transform_to_odom = self.tf_buffer.lookup_transform(
                        'odom',
                        self.target_frame,
                        rospy.Time(0),
                        rospy.Duration(1.0)
                    )
                    
                    # 创建PoseStamped消息用于转换
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = self.target_frame
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.pose = transformed_pose.pose
                    
                    # 执行转换
                    transformed_pose_odom = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform_to_odom)
                    
                    # 创建新的检测消息
                    detection_odom = copy.deepcopy(detection)
                    detection_odom.pose.pose.pose = transformed_pose_odom.pose
                    detection_odom.pose.header.frame_id = 'odom'
                    
                    # 添加到odom消息中
                    new_msg_odom.detections.append(detection_odom)
                    
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(f"TF Error: {e} - Cannot transform from base_link to odom")
                    continue

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"无法转换 AprilTag ID {detection.id} 的位姿: {e}")


        self.publisher.publish(new_msg)
        self.publisher_odom.publish(new_msg_odom)

    def broadcast_transform(self, pose, tag_id):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = self.target_frame
        transform_stamped.child_frame_id = 'tag_origin_' + str(tag_id)
        transform_stamped.transform.translation.x = pose.pose.position.x
        transform_stamped.transform.translation.y = pose.pose.position.y
        transform_stamped.transform.translation.z = pose.pose.position.z
        transform_stamped.transform.rotation = pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform_stamped)

def main():
    try:
        # 添加命令行参数解析
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('--target_frame', type=str, default='base_link',
                          help='Target frame for coordinate transformation')
        parser.add_argument('--source_frame', type=str, default='camera_color_optical_frame',
                          help='Source frame for coordinate transformation')
        args, unknown = parser.parse_known_args()  # Changed from parse_args() to parse_known_args()

        ar_control_node = ARControlNode(target_frame=args.target_frame, source_frame=args.source_frame)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
