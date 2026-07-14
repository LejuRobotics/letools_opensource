#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, CameraInfo
from vision_msgs.msg import Detection2DArray
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from sensor_msgs.point_cloud2 import create_cloud
import threading

# 全局变量
detection_msg = None
point_cloud = None
camera_info = None
frame_lock = threading.Lock()

# YOLO目标检测结果的回调函数
def detection_callback(msg):
    global detection_msg
    detection_msg = msg

# 点云消息的回调函数
def point_cloud_callback(msg):
    global point_cloud
    point_cloud = msg

# 相机信息的回调函数
def camera_info_callback(msg):
    global camera_info
    camera_info = msg

def extract_sub_cloud(cloud, bbox, camera_info):
    u_min = int(bbox.center.x - bbox.size_x / 2.0)
    u_max = int(bbox.center.x + bbox.size_x / 2.0)
    v_min = int(bbox.center.y - bbox.size_y / 2.0)
    v_max = int(bbox.center.y + bbox.size_y / 2.0)

    points = []
    for p in point_cloud2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z", "rgb")):
        x, y, z = p[:3]
        u = int(camera_info.K[0] * x / z + camera_info.K[2])
        v = int(camera_info.K[4] * y / z + camera_info.K[5])
        if u_min <= u <= u_max and v_min <= v <= v_max:
            points.append([x, y, z, p[3]])
    
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = cloud.header.frame_id
    fields = cloud.fields
    sub_cloud = create_cloud(header, fields, points)
    return sub_cloud

def process_and_publish_cloud(pub):
    global detection_msg, point_cloud, camera_info
    while not rospy.is_shutdown():
        if detection_msg is None or point_cloud is None or camera_info is None:
            continue

        with frame_lock:
            input_detection_msg = detection_msg
            input_point_cloud = point_cloud

        for detection in input_detection_msg.detections:
            sub_cloud = extract_sub_cloud(input_point_cloud, detection.bbox, camera_info)
            pub.publish(sub_cloud)

        rospy.sleep(0.1)

def main():
    rospy.init_node('point_cloud_segmentation_node')

    # 创建发布者
    cloud_pub = rospy.Publisher('/segmented_object_cloud', PointCloud2, queue_size=10)

    # 创建订阅者
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, point_cloud_callback)
    rospy.Subscriber('/object_yolo_segment_result', Detection2DArray, detection_callback)
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_callback)

    # 启动处理线程
    process_thread = threading.Thread(target=process_and_publish_cloud, args=(cloud_pub,))
    process_thread.daemon = True
    process_thread.start()

    rospy.spin()

if __name__ == '__main__':
    main()
