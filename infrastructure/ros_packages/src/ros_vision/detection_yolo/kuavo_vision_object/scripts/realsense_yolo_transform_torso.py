#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from vision_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_conjugate
from std_msgs.msg import Header
import math

class YoloTransform:
    def __init__(self):
        # 初始化节点
        rospy.init_node('yolo_transform', anonymous=True)

        # 订阅检测结果话题
        self.detection_sub = rospy.Subscriber('/object_yolo_segment_result', Detection2DArray, self.detection_callback)

        # 发布转换后的结果话题
        self.transformed_pub = rospy.Publisher('/object_yolo_tf2_torso_result', Detection2DArray, queue_size=10)

        # TF缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def normalize_quaternion(self, quat):
        length = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
        quat.x /= length
        quat.y /= length
        quat.z /= length
        quat.w /= length
        return quat

    def detection_callback(self, msg):
        try:
            # 查找坐标变换
            transform = self.tf_buffer.lookup_transform('torso', 'camera_color_optical_frame', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('TF lookup failed')
            return

        # 创建新的Detection2DArray消息
        transformed_msg = Detection2DArray()
        transformed_msg.header = msg.header
        transformed_msg.header.frame_id = 'torso'

        # 遍历所有检测结果并转换坐标
        for detection in msg.detections:
            if detection.results[0].pose.pose.position.x == 0 and \
               detection.results[0].pose.pose.position.y == 0 and \
               detection.results[0].pose.pose.position.z == 0:
                continue
            
            transformed_detection = Detection2D()
            transformed_detection.header = detection.header
            transformed_detection.results = detection.results
            transformed_detection.bbox = detection.bbox
            transformed_detection.source_img = detection.source_img

            # 变换位置坐标
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = detection.results[0].pose.pose

            # 使用TF变换坐标 -- 设置抓取位姿初始化，位置发送变换，姿态保持不变
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            transformed_detection.results[0].pose.pose = transformed_pose.pose  
            transformed_detection.results[0].pose.pose.orientation.x = 0.0
            transformed_detection.results[0].pose.pose.orientation.y = 0.0
            transformed_detection.results[0].pose.pose.orientation.z = 0.0
            transformed_detection.results[0].pose.pose.orientation.w = 1.0 

            # 创建TransformStamped消息用于TF广播
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = 'torso'
            transform_stamped.child_frame_id = f"torso_object_{detection.results[0].id}"
            transform_stamped.transform.translation.x = transformed_pose.pose.position.x
            transform_stamped.transform.translation.y = transformed_pose.pose.position.y
            transform_stamped.transform.translation.z = transformed_pose.pose.position.z
            transform_stamped.transform.rotation.w = 1  # 单位四元数
            # transform_stamped.transform.rotation = self.normalize_quaternion(transformed_pose.pose.orientation)

            # 广播转换
            self.tf_broadcaster.sendTransform(transform_stamped)

            transformed_msg.detections.append(transformed_detection)

        # 发布转换后的检测结果
        self.transformed_pub.publish(transformed_msg)

if __name__ == '__main__':
    try:
        yolo_transform = YoloTransform()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
