#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        # 将ROS图像消息转换为OpenCV格式
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # 显示图像
    cv2.imshow("Camera Image", cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()
