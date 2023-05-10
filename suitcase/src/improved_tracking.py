#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rospy  # Python library for ROS
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import Int16  # Image is the message type
import cv2  # OpenCV library
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import numpy as np
import math


def publish_message():

    # Node is publishing to the video_frames topic using
    # the message type Image
    pub = rospy.Publisher('video_frames', Image, queue_size=10)
    pub2 = rospy.Publisher('x_pixel', Int16, queue_size=10)
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node('video_pub_py', anonymous=True)

    # Go through the loop 10 times per second
    rate = rospy.Rate(10)  # 10hz

    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    cap = cv2.VideoCapture(0)
    WIDTH = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    HEIGHT = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    # print(width, height)
    fgbg = cv2.createBackgroundSubtractorMOG2()

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # While ROS is still running.
    while not rospy.is_shutdown():

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = cap.read()

        if ret == True:
            # Print debugging information to the terminal+
            rospy.loginfo('publishing video frame')

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define the range of green color in HSV color space
            h, s, v = 164, 89, 86
            lower_color = np.array([h-15, s-70, v-70])
            upper_color = np.array([h+15, s+70, v+70])

            # Threshold the HSV image to get only green color
            mask = cv2.inRange(hsv, lower_color, upper_color)

            # Find the contours of the object in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                # Get the bounding box coordinates of the largest contour
                x, y, w, h = cv2.boundingRect(largest_contour)
                # Draw the bounding box around the object in the original frame
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                print(int(WIDTH/2)-int(x+w/2), int(HEIGHT/2)-int(y+h/2))

              # Publish the image.
              # The 'cv2_to_imgmsg' method converts an OpenCV
              # image to a ROS image message
                center = (x + w//2, y + h//2)
                # Draw a line from the center of the rectangle to the center of the camera
                camera_center = (frame.shape[1]//2, frame.shape[0]//2)
                cv2.line(frame, center, camera_center, (255, 0, 0), 2)
              # Calculate the angle between the line and the vertical line through the middle of the camera
                dx = camera_center[0] - center[0]
                dy = camera_center[1] - center[1]
                angle = math.degrees(math.atan2(dy, dx))
                cv2.putText(frame, f"Angle: {angle:.2f}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                # Calculate the x-coordinate of the center of the rectangle relative to the center of the screen
                x_offset = center[0] - camera_center[0]
                cv2.putText(frame, f"X-Offset: {x_offset:.2f}", (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        pub.publish(br.cv2_to_imgmsg(frame, encoding='rgb8'))
        pub2.publish(x_offset)

        # Sleep just enough to maintain the desired rate
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
