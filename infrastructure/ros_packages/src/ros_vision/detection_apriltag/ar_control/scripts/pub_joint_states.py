#!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import JointState
# class ARControlNode(object):
# def publish_joint_states():
#     # 初始化 ROS 节点
#     rospy.init_node('joint_state_publisher_2', anonymous=True)
    
#     # 创建 Publisher
#     pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
#     # 设置发布频率
#     rate = rospy.Rate(10)  # 10 Hz

#     # 创建 JointState 消息
#     joint_state = JointState()
#     joint_state.header.seq = 0
#     joint_state.header.stamp = rospy.Time.now()
#     joint_state.header.frame_id = 'torso'
    
#     # 设置关节名称

#     joint_state.name = [ 'head_yaw', 
#         'head_pitch'
#     ]
    
#     # 设置关节位置
#     joint_state.position = [ 0.0, 0.0
#     ]
#     joint_state.velocity = []
#     joint_state.effort = []

#     # 发布消息
#     while not rospy.is_shutdown():
#         # 更新时间戳
#         joint_state.header.stamp = rospy.Time.now()

#         # 发布消息
#         pub.publish(joint_state)
        
#         rospy.loginfo("Published joint states: %s", joint_state)

#         # 按设定频率休眠
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         publish_joint_states()
#     except rospy.ROSInterruptException:
#         pass

import rospy
from sensor_msgs.msg import JointState

class JointStatePublisher:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('joint_state_publisher_node', anonymous=True)
        
        # 创建 Publisher
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # 设置发布频率
        self.rate = rospy.Rate(10)  # 10 Hz

    def publish_joint_states(self,angle_y,angle_p):
        # 创建 JointState 消息
        joint_state = JointState()
        joint_state.header.seq = 0
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = 'torso'
        
        # 设置关节名称
        joint_state.name = ['head_yaw', 'head_pitch']
        
        # 设置关节位置
        joint_state.position = [angle_y, angle_p]
        joint_state.velocity = []
        joint_state.effort = []

        # 发布消息
        while not rospy.is_shutdown():
            # 更新时间戳
            joint_state.header.stamp = rospy.Time.now()

            # 发布消息
            self.pub.publish(joint_state)
            
            rospy.loginfo("Published joint states: %s", joint_state)

            # 按设定频率休眠
            self.rate.sleep()
