#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_frames():
    rospy.init_node('camera_publisher', anonymous=True)
    pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()
    rate = rospy.Rate(10)  # 10 Hz

    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        return

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            break

        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        publish_frames()
    except rospy.ROSInterruptException:
        pass

