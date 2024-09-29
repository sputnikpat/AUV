#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(data):
    br = CvBridge()
    rospy.loginfo("Receiving video frame")
    current_frame = br.imgmsg_to_cv2(data)

    # Load the Haar cascade classifier for face detection
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    
    # Convert the frame to grayscale for face detection
    gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    
    # Detect faces in the grayscale frame
    faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.3, minNeighbors=5)
    
    # Draw rectangles around all detected faces
    for (x, y, w, h) in faces:
        # Convert the image to cv::UMat for drawing the rectangle
        current_frame_um = cv2.UMat(current_frame)
        cv2.rectangle(current_frame_um, (x, y), (x+w, y+h), (255, 255, 0), 2)
        rospy.loginfo(f"Face detected at ({x}, {y}) with width {w} and height {h}")

        # Display the annotated frame
        cv2.imshow("camera", current_frame_um.get())
        cv2.waitKey(1)

def receive_message():
    rospy.init_node('video_sub_py', anonymous=True)
    rospy.Subscriber('video_frames', Image, callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    receive_message()
