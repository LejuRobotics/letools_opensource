#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.point_cloud2 import read_points, create_cloud
from std_msgs.msg import Header
import threading
import cv2

# 全局变量
point_cloud = None
mask_image = None
camera_info = None
frame_lock = threading.Lock()
bridge = CvBridge()

def point_cloud_callback(msg):
    global point_cloud
    point_cloud = msg

# 相机信息的回调函数
def camera_info_callback(msg):
    global camera_info
    camera_info = msg
    
def mask_callback(msg):
    global mask_image
    try:
        mask_image = bridge.imgmsg_to_cv2(msg, desired_encoding="64FC1")
    except CvBridgeError as e:
        rospy.logerr(f"Failed to convert mask image: {e}")

def extract_sub_cloud_using_mask(cloud, mask, camera_info):
    points = []
    for p in read_points(cloud, skip_nans=True, field_names=("x", "y", "z", "rgb")):
        x, y, z = p[:3]
        u = int(camera_info.K[0] * x / z + camera_info.K[2])
        v = int(camera_info.K[4] * y / z + camera_info.K[5])
        if u >= 0 and v >= 0 and u < mask.shape[1] and v < mask.shape[0] and mask[v, u] > 0:
            points.append([x, y, z, p[3]])

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = cloud.header.frame_id
    fields = cloud.fields
    sub_cloud = create_cloud(header, fields, points)
    return sub_cloud

def process_and_publish_cloud(pub):
    global point_cloud, mask_image, camera_info
    while not rospy.is_shutdown():
        if point_cloud is None or mask_image is None or camera_info is None:
            continue

        with frame_lock:
            input_point_cloud = point_cloud
            input_mask_image = mask_image
            input_camera_info = camera_info

        sub_cloud = extract_sub_cloud_using_mask(input_point_cloud, input_mask_image, input_camera_info)
        pub.publish(sub_cloud)

        rospy.sleep(0.1)

def main():
    rospy.init_node('point_cloud_mask_segmentation_node')

    # 创建发布者
    cloud_pub = rospy.Publisher('/segmented_object_cloud', PointCloud2, queue_size=10)

    # 创建订阅者
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, point_cloud_callback)
    rospy.Subscriber('/object_yolo_segment_mask', Image, mask_callback)
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_callback)

    # 启动处理线程
    process_thread = threading.Thread(target=process_and_publish_cloud, args=(cloud_pub,))
    process_thread.daemon = True
    process_thread.start()

    rospy.spin()

if __name__ == '__main__':
    main()
