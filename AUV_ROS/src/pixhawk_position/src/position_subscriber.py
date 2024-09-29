#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

class PixhawkPositionNode:
    def __init__(self):
        rospy.init_node('pixhawk_position_node', anonymous=True)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.current_pose = PoseStamped()

    def pose_callback(self, data):
        self.current_pose = data
        self.publish_position()

    def publish_position(self):
        rospy.loginfo("Current Position:\n"
                      "x: %f\n"
                      "y: %f\n"
                      "z: %f\n",
                      self.current_pose.pose.position.x,
                      self.current_pose.pose.position.y,
                      self.current_pose.pose.position.z)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.loginfo("connected")
        pixhawk_position_node = PixhawkPositionNode()
        pixhawk_position_node.run()
    except rospy.ROSInterruptException:
        pass
