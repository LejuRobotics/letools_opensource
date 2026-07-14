#!/usr/bin/env python3

import os
import rospy
import rospkg
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics import YOLO
import tf2_ros
import geometry_msgs
from geometry_msgs.msg import TransformStamped

import numpy as np
import tf2_ros
import tf.transformations
import rospy
from geometry_msgs.msg import TransformStamped

class YOLOv8Detector:
    def __init__(self):
        rospy.init_node('yolov8_detector', anonymous=True)
        
        # 获取功能包路径
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('detection_yolo_v8')
        
        # 获取参数并构建绝对路径
        model_path = rospy.get_param('/model_path', 'models/yolov8n.pt')
        if not os.path.isabs(model_path):
            model_path = os.path.join(pkg_path, model_path)
        
        # 检查文件是否存在
        if not os.path.isfile(model_path):
            rospy.logerr(f"Model file not found: {model_path}")
            rospy.signal_shutdown("Model file missing")
            return
        
        self.conf_threshold = rospy.get_param('/conf_threshold', 0.5)
        
        # 获取目标类别列表
        self.target_class = rospy.get_param('/target_class', ['cup'])
        rospy.loginfo(f"Target classes set to: {self.target_class}")
        
        # 加载YOLOv8模型
        rospy.loginfo(f"Loading model from: {model_path}")
        self.model = YOLO(model_path).to('cuda')
        self.class_names = self.model.names
        
        # 检查请求的类别是否在模型支持的类别中
        for cls in self.target_class:
            if cls not in self.class_names.values():
                rospy.logwarn(f"Requested class '{cls}' not found in model classes")
        
        # 加载高度参数
        self.height_table = rospy.get_param('/height_table', 0.864)
        self.height_bottle = rospy.get_param('/height_bottle', 0.22)
        self.height_cup = rospy.get_param('/height_cup', 0.16)
        self.height_banana = rospy.get_param('/height_banana', 0.04)
        self.height_apple = rospy.get_param('/height_apple', 0.071)
        self.height_orange = rospy.get_param('/height_orange', 0.071)
        self.height_carrot = rospy.get_param('/height_carrot', 0.04)
        rospy.loginfo(
            "[Height Params] "
            f"Table={self.height_table:.3f} m | "
            f"Bottle={self.height_bottle:.3f} m | "
            f"Cup={self.height_cup:.3f} m | "
            f"Banana={self.height_banana:.3f} m | "
            f"Apple={self.height_apple:.3f} m | "
            f"Orange={self.height_orange:.3f} m | "
            f"Carrot={self.height_carrot:.3f} m"
        )

        # 初始化CV桥
        self.bridge = CvBridge()
        
        # 订阅图像话题
        input_topic = rospy.get_param('~input_image', '/camera/color/image_raw')
        rospy.loginfo(f"Subscribing to image topic: {input_topic}")  
        rospy.Subscriber(input_topic, Image, self.image_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        # 发布检测结果
        self.detection_pub = rospy.Publisher('/yolov8_detections', Detection2DArray, queue_size=10)
        self.debug_pub = rospy.Publisher('/yolov8/output_image', Image, queue_size=10)
        
        # 初始化参数
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        # 初始化广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.loginfo("YOLOv8 Detector initialized")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    # 图像回调函数
    def image_callback(self, msg):
        try:
            # 转换ROS图像为OpenCV格式
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")

    # 深度图像回调函数
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            rospy.logerr(f"Failed to convert depth image: {e}")

    # 相机信息回调函数
    def camera_info_callback(self, msg):
        self.camera_info = msg

    # 将二维像素坐标转换为三维坐标(简易版)
    def convert_to_3d(self, u, v, depth):
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        z = depth / 1000.0  # 深度值通常以毫米为单位，需要转换为米
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y, z
    
    # 将二维像素坐标转换为三维坐标(准确版)
    def compute_3d_from_pixel(self, u, v, predef_h):
        """
        输入：
            u, v        : 图像像素坐标
            predef_h    : 预定义抓取点在 base_link 坐标系下的高度 (单位：米)

        返回：
            (x, y, z)   : 相机坐标系下的三维坐标
        """

        # === 1. 获取相机内参矩阵 ===
        K = np.array(self.camera_info.K).reshape(3, 3)

        # === 2. 将像素坐标转换为相机坐标系单位视线方向 ===
        pixel = np.array([u, v, 1.0])
        K_inv = np.linalg.inv(K)
        ray_dir_cam = K_inv @ pixel
        ray_dir_cam = ray_dir_cam / np.linalg.norm(ray_dir_cam)

        # === 3. 获取相机在 odom 坐标系中的 TF ===
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame="odom",
                source_frame="camera_color_optical_frame",
                time=rospy.Time(0),
                timeout=rospy.Duration(1.0)
            )
        except Exception as e:
            rospy.logwarn(f"TF 获取失败: {e}")
            return None

        # 相机在 odom 中的位置
        cam_pos = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        # 相机朝向（旋转矩阵）
        quat = transform.transform.rotation
        rot_matrix = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])[:3, :3]

        # === 4. 将射线方向从相机坐标系转换到 base_link 坐标系 ===
        ray_dir_world = rot_matrix @ ray_dir_cam

        # === 5. 计算射线与预定义抓取高度的交点 ===
        t = (predef_h - cam_pos[2]) / ray_dir_world[2]
        point_world = cam_pos + t * ray_dir_world

        # === 6. 将点从 base_link 坐标系变换到相机坐标系下 ===
        # P_camera = R^T * (P_world - cam_pos)
        point_cam = np.linalg.inv(rot_matrix) @ (point_world - cam_pos)

        return tuple(point_cam)


    # 广播 TF 转换
    def broadcast_tf_transforms(self, detection_msg):
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
                transform.child_frame_id = f"camera_yolov8_object_{detection.results[0].id}"  # 使用检测到的对象 ID 作为子帧 ID
                transform.transform.translation.x = x
                transform.transform.translation.y = y
                transform.transform.translation.z = z
                transform.transform.rotation.w = 1  # 单位四元数
                
                # 广播 TF 转换
                self.tf_broadcaster.sendTransform(transform)

    # YOLO检测函数
    def process_frame(self , color_image , depth_image ):
                # 使用YOLOv8进行检测
        results = self.model(color_image, conf=self.conf_threshold, verbose=False)
        
        # 创建检测结果消息
        detections_msg = Detection2DArray()
        # detections_msg.header.stamp = rospy.Time.now()
        # detections_msg.header.frame_id = "camera_color_optical_frame"
###########################################################################################
        # 处理检测结果
        for result in results:
            for box in result.boxes:
                # 提取边界框信息
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = box.conf[0].item()
                class_id = int(box.cls[0].item())
                class_name = self.class_names[class_id]
                
                # 只发布目标类别的检测结果（支持多个类别）
                if class_name not in self.target_class:
                    continue
                
                # 创建检测消息
                detection = Detection2D()
                # detection.header.stamp = rospy.Time.now()
                # detection.header.frame_id = "camera_color_optical_frame"
                # 设置边界框
                detection.bbox.center.x = (x1 + x2) / 2.0
                detection.bbox.center.y = (y1 + y2) / 2.0
                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1
                # 设置分类结果
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = class_id
                hypothesis.score = conf
                detection.results.append(hypothesis)
###########################################################################################
                # 获取中心点的深度值并转换为3D坐标
                u = int(detection.bbox.center.x)
                v = int(detection.bbox.center.y)
                if class_name == 'cup':
                    x, y, z = self.compute_3d_from_pixel( u, v, self.height_table + self.height_cup/2)
                    rospy.loginfo(f"Find cup at ({u}, {v}) mm, 3D position: ({x}, {y}, {z})")
                elif class_name == 'bottle':
                    x, y, z = self.compute_3d_from_pixel( u, v, self.height_table + self.height_bottle/2)
                    rospy.loginfo(f"Find bottle at ({u}, {v}) mm, 3D position: ({x}, {y}, {z})")
                elif class_name == 'banana':
                    x, y, z = self.compute_3d_from_pixel( u, v, self.height_table + self.height_banana/2)
                    rospy.loginfo(f"Find banana at ({u}, {v}) mm, 3D position: ({x}, {y}, {z})")
                elif class_name == 'apple':
                    x, y, z = self.compute_3d_from_pixel( u, v, self.height_table + self.height_apple/2)
                    rospy.loginfo(f"Find apple at ({u}, {v}) mm, 3D position: ({x}, {y}, {z})")
                elif class_name == 'orange':
                    x, y, z = self.compute_3d_from_pixel( u, v, self.height_table + self.height_orange/2)
                    rospy.loginfo(f"Find orange at ({u}, {v}) mm, 3D position: ({x}, {y}, {z})")
                elif class_name == 'carrot':
                    x, y, z = self.compute_3d_from_pixel( u, v, self.height_table + self.height_carrot/2)
                    rospy.loginfo(f"Find carrot at ({u}, {v}) mm, 3D position: ({x}, {y}, {z})")
                else : # 保留直接使用深度图的处理逻辑,正常情况不会使用
                    if depth_image is not None:
                        depth = depth_image[v, u]
                        if depth > 0:  # 检查深度值是否有效
                            x, y, z = self.convert_to_3d( u, v, depth)
                            rospy.loginfo(f"Depth at ({u}, {v}): {depth} mm, 3D position: ({x}, {y}, {z})")
                        else :
                            rospy.logwarn(f"Invalid depth at ({u}, {v}): {depth}")
                            x, y, z = 0.0, 0.0, 0.0
                    else:
                        rospy.logwarn(f"Depth image is None at ({u}, {v})")
                        x, y, z = 0.0, 0.0, 0.0
                
                detection.results[0].pose.pose.position.x = x
                detection.results[0].pose.pose.position.y = y
                detection.results[0].pose.pose.position.z = z
                detection.results[0].pose.pose.orientation.x = 0.0 
                detection.results[0].pose.pose.orientation.y = 0.0 
                detection.results[0].pose.pose.orientation.z = 0.0 
                detection.results[0].pose.pose.orientation.w = 1.0 

                # 添加到检测结果列表
                detections_msg.detections.append(detection)
###########################################################################################
                # 在图像上绘制边界框
                label = f"{class_name}: {conf:.2f}"
                color = (0, 255, 0)  # 绿色
                if class_name == "bottle":
                    color = (0, 0, 255)  # 红色瓶子
                elif class_name == "cup":
                    color = (255, 0, 0)  # 蓝色杯子
                elif class_name == "banana":
                    color = (0, 255, 255)  # 黄色香蕉
                elif class_name == "apple":
                    color = (30, 30, 220)  # 红色苹果
                elif class_name == "orange":
                    color = (0, 255, 0)  # 绿色橙子
                elif class_name == "carrot":
                    color = (0, 140, 255)  # 橙色胡萝卜
                
                cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(color_image, label, (int(x1), int(y1)-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
###########################################################################################
        # 发布检测结果
        self.detection_pub.publish(detections_msg)
        
        # 发布带有推理结果的图像消息
        debug_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        debug_msg.header = detections_msg.header
        self.debug_pub.publish(debug_msg)

        # 广播目标检测结果的 TF 转换
        self.broadcast_tf_transforms(detections_msg)
###########################################################################################
    def run(self):
        
        # 持续等待
        while not rospy.is_shutdown():
            # 确保彩色图和深度图都非空
            if self.color_image is None or self.depth_image is None or self.camera_info is None:
                continue
            # 使用局部参数进行传参, 防止在处理过程中图像被回调函数修改
            self.process_frame(self.color_image , self.depth_image)
            time.sleep(0.01)  # 模拟处理时间
                
if __name__ == '__main__':
    try:
        detector = YOLOv8Detector()

        detector.run()

    except rospy.ROSInterruptException:
        pass
    