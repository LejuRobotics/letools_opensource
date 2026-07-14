#!/usr/bin/env python
"""
本脚本将YOLO目标检测与RealSense相机数据集成，以在ROS（机器人操作系统）环境中执行对象检测。
它利用RGB和深度图像以及相机信息来检测对象，将像素坐标转换为3D坐标，并将结果作为TF（Transform）帧进行广播。

主要功能：
- 订阅RGB和深度图像话题以及相机信息话题。
- 对RGB图像执行YOLO目标检测。
- 使用深度信息和相机内参将像素坐标转换为3D坐标。
- 将检测结果发布为Detection2DArray消息。
- 为检测到的对象广播TF转换。

模块：
- rospy：ROS Python库。
- rospkg：ROS Python包库。
- pyrealsense2：Intel RealSense SDK的Python接口。
- numpy：用于数值计算的库。
- cv2：OpenCV库，用于计算机视觉任务。
- threading：多线程处理的库。
- time：时间相关函数。
- concurrent.futures：并行执行的库。
- sensor_msgs.msg：ROS传感器数据消息类型。
- vision_msgs.msg：ROS视觉相关数据消息类型。
- cv_bridge：用于在OpenCV和ROS图像格式之间转换的ROS库。
- tf2_ros：处理TF转换的ROS库。
- geometry_msgs：ROS几何数据消息类型。

全局变量：
- color_image：存储RGB图像数据。
- depth_image：存储深度图像数据。
- camera_info：存储相机内参。
- frame_lock：用于访问共享数据的线程锁。
- bridge：ROS CvBridge实例，用于图像转换。

函数：
- image_callback：用于RGB图像消息的ROS回调函数。
- depth_callback：用于深度图像消息的ROS回调函数。
- camera_info_callback：用于相机信息消息的ROS回调函数。
- broadcast_tf_transforms：为检测到的对象广播TF转换。
- convert_to_3d：使用相机内参和深度数据将像素坐标转换为3D坐标。
- process_frame：在输入图像上执行YOLO目标检测，转换坐标，并创建Detection2DArray消息。
- process_frames：持续处理帧，发布检测结果，并广播TF转换。
- main：主函数，用于初始化ROS节点，设置发布者/订阅者，并启动处理线程。
"""
import rospy
import rospkg
# import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import time
import yaml
from concurrent.futures import ThreadPoolExecutor
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs
import os
import sys
sys.path.append(os.path.join(rospkg.RosPack().get_path('kuavo_vision_object'), 'scripts'))
from yoloseg import YOLOSeg

# 初始化全局变量
color_image = None
depth_image = None
camera_info = None
frame_lock = threading.Lock()
bridge = CvBridge()

# 加载配置文件
rospack = rospkg.RosPack()
config_path = os.path.join(rospack.get_path('kuavo_vision_object'), 'config/yolo.yaml')
with open(config_path, 'r') as config_file:
    config = yaml.safe_load(config_file)
interested_classes = config['interested_classes']

# 图像回调函数
def image_callback(msg):
    global color_image
    try:
        color_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Failed to convert image: {e}")

# 深度图像回调函数
def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, "16UC1")
    except CvBridgeError as e:
        rospy.logerr(f"Failed to convert depth image: {e}")

# 相机信息回调函数
def camera_info_callback(msg):
    global camera_info
    camera_info = msg

def broadcast_tf_transforms(detection_msg, tf_broadcaster):
    for detection in detection_msg.detections:
        u = int(detection.bbox.center.x)
        v = int(detection.bbox.center.y)
        depth = depth_image[v, u]
        if depth > 0:  # 检查深度值是否有效
            x, y, z = convert_to_3d(u, v, depth, camera_info)
            # 创建 TF 转换
            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "head_camera_color_optical_frame"
            transform.child_frame_id = f"camera_object_{detection.results[0].id}"  # 使用检测到的对象 ID 作为子帧 ID
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.w = 1  # 单位四元数
            # 广播 TF 转换
            tf_broadcaster.sendTransform(transform)

# 将二维像素坐标转换为三维坐标
def convert_to_3d(u, v, depth, camera_info):
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]

    z = depth / 1000.0  # 深度值通常以毫米为单位，需要转换为米
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return x, y, z

# 预处理、推理、后处理的函数
def process_frame(yoloseg, input_image, depth_image, camera_info):
    start_time = time.time()
    boxes, scores, class_ids, masks = yoloseg(input_image)
    combined_img = yoloseg.draw_masks(input_image)
    end_time = time.time()
    inference_time = end_time - start_time
    fps = 1 / inference_time
    # rospy.loginfo(f"FPS : {fps:.4f} HZ")

    # 在图像左上角添加FPS信息
    cv2.putText(combined_img, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 创建Detection2DArray消息
    detection_msg = Detection2DArray()
    detection_msg.header.stamp = rospy.Time.now()
    detection_msg.header.frame_id = "head_camera_color_optical_frame"

    mask_msgs = []
    for box, score, class_id, mask in zip(boxes, scores, class_ids, masks):
        if class_id not in interested_classes:
            continue  # 忽略不感兴趣的类别

        detection = Detection2D()
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = int(class_id)
        hypothesis.score = score
        detection.results.append(hypothesis)
        detection.bbox.center.x = (box[0] + box[2]) / 2.0
        detection.bbox.center.y = (box[1] + box[3]) / 2.0
        detection.bbox.size_x = box[2] - box[0]
        detection.bbox.size_y = box[3] - box[1]

        # 获取中心点的深度值并转换为3D坐标
        u = int(detection.bbox.center.x)
        v = int(detection.bbox.center.y)
        if depth_image is not None:
            depth = depth_image[v, u]
            if depth > 0:  # 检查深度值是否有效
                x, y, z = convert_to_3d(u, v, depth, camera_info)
                rospy.loginfo(f"Depth at ({u}, {v}): {depth} mm, 3D position: ({x}, {y}, {z})")
                detection.results[0].pose.pose.position.x = x
                detection.results[0].pose.pose.position.y = y
                detection.results[0].pose.pose.position.z = z

                detection.results[0].pose.pose.orientation.x = 0.0 
                detection.results[0].pose.pose.orientation.y = 0.0 
                detection.results[0].pose.pose.orientation.z = 0.0 
                detection.results[0].pose.pose.orientation.w = 1.0 
            else:  # 如果深度无效，则将数值全赋值为0
                detection.results[0].pose.pose.position.x = 0
                detection.results[0].pose.pose.position.y = 0
                detection.results[0].pose.pose.position.z = 0

                detection.results[0].pose.pose.orientation.x = 0.0 
                detection.results[0].pose.pose.orientation.y = 0.0 
                detection.results[0].pose.pose.orientation.z = 0.0 
                detection.results[0].pose.pose.orientation.w = 1.0 

                rospy.logwarn(f"Invalid depth at ({u}, {v}): {depth}")
        else:
            rospy.logwarn(f"Depth image is None at ({u}, {v})")

        detection_msg.detections.append(detection)

        # 转换mask为ROS Image消息并添加到mask_msgs列表中
        try:
            # 将mask转换为8位单通道图像
            mask_8u = cv2.convertScaleAbs(mask, alpha=(255.0/np.max(mask)))
            mask_msg = bridge.cv2_to_imgmsg(mask_8u, "mono8")
            mask_msgs.append(mask_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert mask image: {e}")

    return combined_img, detection_msg, mask_msgs

# 持续处理帧并发布检测结果
def process_frames(yoloseg, executor, pub, image_pub, mask_pub, tf_broadcaster):
    global color_image, depth_image, combined_img, camera_info
    while not rospy.is_shutdown():
        if color_image is None or depth_image is None or camera_info is None:
            continue
        with frame_lock:
            input_image = color_image.copy()
            input_depth_image = depth_image.copy()

        # 使用线程池进行处理
        future = executor.submit(process_frame, yoloseg, input_image, input_depth_image, camera_info)
        combined_img, detection_msg, mask_msgs = future.result()

        # 发布Detection2DArray消息
        pub.publish(detection_msg)

        # 发布带有推理结果的图像消息
        try:
            image_msg = bridge.cv2_to_imgmsg(combined_img, "bgr8")
            image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert and publish image: {e}")

        # 发布mask消息
        for mask_msg in mask_msgs:
            mask_pub.publish(mask_msg)

        # 广播目标检测结果的 TF 转换
        broadcast_tf_transforms(detection_msg, tf_broadcaster)

        time.sleep(0.01)  # 模拟处理时间

# 主函数
def main():
    rospy.init_node('yolo_detection_node')

    # 创建发布者
    pub = rospy.Publisher('/object_yolo_segment_result', Detection2DArray, queue_size=10)
    image_pub = rospy.Publisher('/object_yolo_segment_image', Image, queue_size=10)
    mask_pub = rospy.Publisher('/object_yolo_segment_mask', Image, queue_size=10)  # 新增mask发布者

    # 创建订阅者
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback)
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_callback)

    # 初始化YOLOSeg对象
    model_path = os.path.join(rospkg.RosPack().get_path('kuavo_vision_object'), 'scripts/models/yolov5s-seg.onnx')
    yoloseg = YOLOSeg(model_path, conf_thres=0.3, iou_thres=0.3)

    # 初始化线程池
    executor = ThreadPoolExecutor(max_workers=4)

    # 初始化广播器
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # 启动处理线程
    thread2 = threading.Thread(target=process_frames, args=(yoloseg, executor, pub, image_pub, mask_pub, tf_broadcaster))

    # 设置守护线程，确保主线程退出时子线程也退出
    thread2.daemon = True

    # 启动线程
    thread2.start()

    rospy.spin()

    # 关闭线程池
    executor.shutdown()

if __name__ == '__main__':
    main()
