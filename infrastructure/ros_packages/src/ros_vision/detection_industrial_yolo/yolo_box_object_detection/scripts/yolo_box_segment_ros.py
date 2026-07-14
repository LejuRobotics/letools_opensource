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
- ultralytics: Ultralytics YOLO 库，用于目标检测和分割。

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
from ultralytics import YOLO

# 初始化全局变量
color_image = None
depth_image = None
camera_info = None
frame_lock = threading.Lock()
bridge = CvBridge()

# 加载配置文件
rospack = rospkg.RosPack()
config_path = os.path.join(rospack.get_path('yolo_box_object_detection'), 'config/yolo.yaml')
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
        if detection.results:
            # 获取3D坐标
            x = detection.results[0].pose.pose.position.x
            y = detection.results[0].pose.pose.position.y
            z = detection.results[0].pose.pose.position.z
            
            # 创建 TF 转换
            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "camera_color_optical_frame"
            transform.child_frame_id = f"box_object_{detection.results[0].id}"  # 使用检测到的对象 ID 作为子帧 ID
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.w = 1  # 单位四元数
            
            # 广播 TF 转换
            tf_broadcaster.sendTransform(transform)

# 将二维像素坐标转换为三维坐标
def convert_to_3d(u, v, depth_image, camera_info, box, region_factor=0.5):
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]

    # 计算区域范围
    box_width = box[2] - box[0]
    box_height = box[3] - box[1]
    region_width = int(box_width * region_factor)
    region_height = int(box_height * region_factor)

    # 提取区域内的深度值
    u_min = max(0, u - region_width // 2)
    u_max = min(depth_image.shape[1], u + region_width // 2)
    v_min = max(0, v - region_height // 2)
    v_max = min(depth_image.shape[0], v + region_height // 2)
    
    depth_region = depth_image[v_min:v_max, u_min:u_max]
    depth_values = depth_region[depth_region > 0]  # 过滤掉无效深度值

    if len(depth_values) == 0:
        return None  # 没有有效深度值

    median_depth = np.median(depth_values)

    z = median_depth / 1000.0  # 深度值通常以毫米为单位，需要转换为米
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return x, y, z

# 修改process_frame函数，使用Ultralytics进行目标检测
def process_frame(model, input_image, depth_image, camera_info):
    start_time = time.time()

    # 使用 Ultralytics 进行目标检测
    results = model(input_image)
    result = results[0]

    # 获取边界框、置信度和类别ID
    boxes = result.boxes.xyxy.cpu().numpy()
    scores = result.boxes.conf.cpu().numpy()
    class_ids = result.boxes.cls.cpu().numpy().astype(int)

    # 创建一个副本用于可视化
    combined_img = input_image.copy()
    
    # 计算FPS
    end_time = time.time()
    inference_time = end_time - start_time
    fps = 1 / inference_time
    cv2.putText(combined_img, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 创建Detection2DArray消息
    detection_msg = Detection2DArray()
    detection_msg.header.stamp = rospy.Time.now()
    detection_msg.header.frame_id = "camera_color_optical_frame"

    for box, score, class_id in zip(boxes, scores, class_ids):
        # 只处理ID为1的对象
        if class_id != 1:
            continue
            
        rospy.loginfo(f"Class ID: {class_id}, Score: {score}")
        if score < 0.6:
            rospy.loginfo("Detection filtered out due to low confidence.")
            continue

        # 在图像上绘制边界框和标签
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(combined_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"package: {score:.2f}"
        cv2.putText(combined_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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
            result = convert_to_3d(u, v, depth_image, camera_info, box)
            if result is not None:
                x, y, z = result
                rospy.loginfo(f"Depth at ({u}, {v}): Median depth, 3D position: ({x}, {y}, {z})")
                detection.results[0].pose.pose.position.x = x
                detection.results[0].pose.pose.position.y = y
                detection.results[0].pose.pose.position.z = z

                detection.results[0].pose.pose.orientation.x = 0.0 
                detection.results[0].pose.pose.orientation.y = 0.0 
                detection.results[0].pose.pose.orientation.z = 0.0 
                detection.results[0].pose.pose.orientation.w = 1.0 
            else:  # 如果没有有效深度值，则将数值全赋值为0
                detection.results[0].pose.pose.position.x = 0
                detection.results[0].pose.pose.position.y = 0
                detection.results[0].pose.pose.position.z = 0

                detection.results[0].pose.pose.orientation.x = 0.0 
                detection.results[0].pose.pose.orientation.y = 0.0 
                detection.results[0].pose.pose.orientation.z = 0.0 
                detection.results[0].pose.pose.orientation.w = 1.0 

                rospy.logwarn(f"No valid depth values in the region at ({u}, {v})")
        else:
            rospy.logwarn(f"Depth image is None at ({u}, {v})")

        detection_msg.detections.append(detection)

    return combined_img, detection_msg

# 持续处理帧并发布检测结果
def process_frames(model, executor, pub, image_pub, tf_broadcaster):
    global color_image, depth_image, combined_img, camera_info
    while not rospy.is_shutdown():
        if color_image is None or depth_image is None or camera_info is None:
            rospy.logwarn("Waiting for camera data...")
            continue
        with frame_lock:
            input_image = color_image.copy()
            input_depth_image = depth_image.copy()

        # 使用线程池进行处理
        future = executor.submit(process_frame, model, input_image, input_depth_image, camera_info)
        combined_img, detection_msg = future.result()

        # 发布Detection2DArray消息
        pub.publish(detection_msg)

        # 发布带有推理结果的图像消息
        try:
            image_msg = bridge.cv2_to_imgmsg(combined_img, "bgr8")
            image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert and publish image: {e}")

        # 广播目标检测结果的 TF 转换
        broadcast_tf_transforms(detection_msg, tf_broadcaster)

        time.sleep(0.01)  # 模拟处理时间

# 主函数
def main():
    rospy.init_node('yolo_box_detection_node')

    # 创建发布者
    pub = rospy.Publisher('/object_yolo_box_segment_result', Detection2DArray, queue_size=10)
    image_pub = rospy.Publisher('/object_yolo_box_segment_image', Image, queue_size=10)

    # 创建订阅者
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    use_orbbec = rospy.get_param('use_orbbec')
    if use_orbbec:
        rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)
    else:
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback)
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_callback)

    # 使用 Ultralytics 加载模型
    model_path = os.path.join(rospkg.RosPack().get_path('yolo_box_object_detection'), 'scripts/models/best.pt')
    model = YOLO(model_path)

    # 初始化线程池
    executor = ThreadPoolExecutor(max_workers=4)

    # 初始化广播器
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # 启动处理线程
    thread2 = threading.Thread(target=process_frames, args=(model, executor, pub, image_pub, tf_broadcaster))

    # 设置守护线程，确保主线程退出时子线程也退出
    thread2.daemon = True

    # 启动线程
    thread2.start()

    rospy.spin()

    # 关闭线程池
    executor.shutdown()

if __name__ == '__main__':
    main()
