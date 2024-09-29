#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray

# Constants for known values
KNOWN_DISTANCE = 30.0  # Distance from camera to object (inches)
KNOWN_WIDTH = 5.7  # Known width of the object (inches)

# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)

# Fonts
fonts = cv2.FONT_HERSHEY_SIMPLEX

# Load the pre-trained Haar Cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Focal length finder function
def focal_length(measured_distance, real_width, width_in_rf_image):
    return (width_in_rf_image * measured_distance) / real_width

# Distance estimation function
def distance_finder(focal_length, real_face_width, face_width_in_frame):
    return (real_face_width * focal_length) / face_width_in_frame

# Face detection function
def face_data(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    if len(faces) > 0:
        # Get the largest face
        largest_face = max(faces, key=lambda rect: rect[2] * rect[3])
        (x, y, w, h) = largest_face
        face_center_x = x + w // 2
        face_center_y = y + h // 2
        return w, largest_face, face_center_x, face_center_y
    return 0, (), 0, 0

# Reading reference image from directory (this image should be captured at a known distance)
ref_image = cv2.imread("/home/prroid/Distance_measurement_using_single_camera/Ref_image.png")
ref_image_face_width, _, _, _ = face_data(ref_image)
focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_face_width)

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.callback)
        self.distance_pub = rospy.Publisher("distance_data", Float32MultiArray, queue_size=10)

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2

        face_width_in_frame, face_rect, face_center_x, face_center_y = face_data(frame)

        if face_width_in_frame != 0:
            (x, y, w, h) = face_rect

            distance_z = distance_finder(focal_length_found, KNOWN_WIDTH, face_width_in_frame)
            distance_z = round(distance_z, 2)

            cv2.rectangle(frame, (x, y), (x + w, y + h), GREEN, 2)
            cv2.circle(frame, (face_center_x, face_center_y), 5, GREEN, -1)

            distance_x = face_center_x - center_x
            distance_y = face_center_y - center_y

            distance_msg = Float32MultiArray()
            distance_msg.data = [distance_x, distance_y, distance_z]
            self.distance_pub.publish(distance_msg)

            cv2.putText(frame, f"Distance Z: {distance_z} inches", (10, 30), fonts, 0.6, WHITE, 2)
            cv2.putText(frame, f"Distance X: {distance_x} px", (10, 60), fonts, 0.6, WHITE, 2)
            cv2.putText(frame, f"Distance Y: {distance_y} px", (10, 90), fonts, 0.6, WHITE, 2)

        cv2.circle(frame, (center_x, center_y), 5, RED, -1)
        cv2.imshow('Frame', frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=True)
    ImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

