#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import argparse

class ImageRotator180:
    def __init__(self, camera_name="head_camera"):
        rospy.init_node('image_rotator_180', anonymous=True)

        self.bridge = CvBridge()
        self.camera_name = camera_name
        self.depth_sub = rospy.Subscriber(f'/{self.camera_name}/depth/image_raw', Image, self.depth_callback)
        self.color_sub = rospy.Subscriber(f'/{self.camera_name}/color/image_raw', Image, self.color_callback)

        self.depth_pub = rospy.Publisher(f'/{self.camera_name}/depth/image_raw_rotate_180', Image, queue_size=1)
        self.color_pub = rospy.Publisher(f'/{self.camera_name}/color/image_raw_rotate_180', Image, queue_size=1)

    def rotate_and_publish(self, msg, publisher, encoding):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            rotated = cv2.rotate(cv_img, cv2.ROTATE_180)
            rotated_msg = self.bridge.cv2_to_imgmsg(rotated, encoding=encoding)
            rotated_msg.header = msg.header
            publisher.publish(rotated_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def depth_callback(self, msg):
        self.rotate_and_publish(msg, self.depth_pub, encoding="passthrough")

    def color_callback(self, msg):
        self.rotate_and_publish(msg, self.color_pub, encoding="bgr8")

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--camera_name", type=str, default="head_camera")
        args, unknown = parser.parse_known_args()
        ImageRotator180(args.camera_name)
        rospy.loginfo("Image stream color start!")
        rospy.loginfo("Image stream depth start!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass