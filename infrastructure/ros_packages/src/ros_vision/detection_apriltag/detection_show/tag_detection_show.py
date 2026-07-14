#!/usr/bin/env python3
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np

class TagProcessor:
    def __init__(self, is_init=False):
        """
        简化的标签检测处理器 - 只关注图像中的边框绘制
        :param is_init: 是否初始化ROS节点
        """
        if is_init:
            rospy.init_node('simple_tag_processor', anonymous=True)
        
        # 初始化CvBridge
        self.bridge = CvBridge()
        
        # 最新的检测结果
        self.latest_detections = []
        
        # 相机参数是否已获取
        self.camera_params_received = False
        
        # 使用固定的标签尺寸用于绘制（这个值不影响边框对齐）
        self.default_tag_size = 0.04  # 4cm，可以是任意合理值
        
        # 发布器
        self.image_pub = rospy.Publisher('/tag_detections_image', Image, queue_size=1)
        
        # 订阅器
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detection_callback)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

    def camera_info_callback(self, msg):
        """
        相机信息回调，更新相机参数
        """
        # 更新相机参数
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.image_width = msg.width
        self.image_height = msg.height
        
        if not self.camera_params_received:
            print(f"相机参数已更新: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")
            print(f"图像尺寸: {self.image_width}x{self.image_height}")
            self.camera_params_received = True

    def detection_callback(self, msg):
        """
        处理标签检测结果
        """
        self.latest_detections = []
        
        for detection in msg.detections:
            try:
                tag_id = detection.id[0] if detection.id else -1
                
                # 获取相机坐标系下的位置和姿态
                position = detection.pose.pose.pose.position
                orientation = detection.pose.pose.pose.orientation
                
                # 计算标签的四个角点（在相机坐标系中）
                # 注意：这里的tag_size只是用于保持和apriltag_ros一致的比例
                corners_3d = self.calculate_tag_corners_3d(position, orientation, self.default_tag_size)
                
                # 将3D角点投影到图像平面
                corners_2d = self.project_corners_to_image(corners_3d)
                
                tag_info = {
                    'id': tag_id,
                    'x': position.x,
                    'y': position.y, 
                    'z': position.z,
                    'corners_2d': corners_2d
                }
                
                self.latest_detections.append(tag_info)
                
            except Exception as e:
                rospy.logwarn(f"处理检测结果出错: {e}")

    def calculate_tag_corners_3d(self, position, orientation, tag_size):
        """
        根据标签的位置、姿态和尺寸计算四个角点的3D坐标
        """
        # 将四元数转换为旋转矩阵
        R = self.quaternion_to_rotation_matrix(orientation)
        
        # 标签的半尺寸
        half_size = tag_size / 2.0
        
        # 标签局部坐标系中的四个角点（逆时针顺序）
        local_corners = np.array([
            [-half_size, -half_size, 0],  # 左下
            [ half_size, -half_size, 0],  # 右下
            [ half_size,  half_size, 0],  # 右上
            [-half_size,  half_size, 0]   # 左上
        ])
        
        # 转换到相机坐标系
        world_corners = []
        for corner in local_corners:
            # 旋转
            rotated = R @ corner
            # 平移
            world_point = rotated + np.array([position.x, position.y, position.z])
            world_corners.append(world_point)
            
        return world_corners

    def quaternion_to_rotation_matrix(self, orientation):
        """
        将四元数转换为旋转矩阵
        """
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        
        # 旋转矩阵
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
        
        return R

    def project_corners_to_image(self, corners_3d):
        """
        将3D角点投影到图像平面
        """
        corners_2d = []
        
        for corner in corners_3d:
            x, y, z = corner
            if z > 0:  # 确保在相机前方
                # 透视投影
                pixel_x = int(self.fx * x / z + self.cx)
                pixel_y = int(self.fy * y / z + self.cy)
                corners_2d.append([pixel_x, pixel_y])
            else:
                corners_2d.append(None)  # 无效点
                
        return corners_2d

    def image_callback(self, msg):
        """
        图像回调，绘制检测结果
        """
        try:
            # 转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 绘制检测结果
            annotated_image = self.draw_detections(cv_image.copy())
            
            # 发布可视化图像
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
            
        except Exception as e:
            rospy.logerr(f"图像处理错误: {e}")

    def draw_detections(self, image):
        """
        在图像上绘制检测结果
        """
        if not self.latest_detections:
            return image
            
        height, width = image.shape[:2]
        
        for detection in self.latest_detections:
            corners_2d = detection['corners_2d']
            tag_id = detection['id']
            
            # 检查是否有有效的角点
            valid_corners = [corner for corner in corners_2d if corner is not None]
            
            if len(valid_corners) >= 4:
                # 将角点转换为numpy数组
                corners_array = np.array(valid_corners, dtype=np.int32)
                
                # 确保角点在图像范围内
                corners_array[:, 0] = np.clip(corners_array[:, 0], 0, width - 1)
                corners_array[:, 1] = np.clip(corners_array[:, 1], 0, height - 1)
                
                # 绘制标签轮廓（四边形）
                cv2.polylines(image, [corners_array], True, (0, 255, 0), 2)
                
                # 可选：填充半透明区域
                overlay = image.copy()
                cv2.fillPoly(overlay, [corners_array], (0, 255, 0))
                cv2.addWeighted(overlay, 0.2, image, 0.8, 0, image)
                
                # 计算中心点
                center_x = int(np.mean(corners_array[:, 0]))
                center_y = int(np.mean(corners_array[:, 1]))
                
                # 绘制ID
                text = str(tag_id)
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.4
                thickness = 1
                
                # 获取文本尺寸
                (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
                
                # 计算文本位置（居中）
                text_x = center_x - text_width // 2
                text_y = center_y + text_height // 2
                
                # 确保文本在图像范围内
                text_x = max(0, min(text_x, width - text_width))
                text_y = max(text_height, min(text_y, height))
                
                # 绘制ID（白色背景 + 红色文字）
                
                cv2.putText(image, text, (text_x, text_y), font, font_scale, (0, 0, 255), thickness)
        
        return image


def main():
    """
    主函数
    """
    try:
        # 创建简化的处理器
        processor = TagProcessor(is_init=True)
        
        print("=== 简化版标签检测器已启动 ===")
        print("等待相机参数...")
        print("发布可视化图像到: /tag_detections_image")
        print("使用 rqt_image_view 查看结果")
        
        
        # 保持节点运行
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print("节点已停止")
    except KeyboardInterrupt:
        print("用户中断")


if __name__ == '__main__':
    main()