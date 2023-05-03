#!/usr/bin/env python3
# Basic ROS program to publish real-time streaming
# video from your built-in webcam and detect an ArUco tag with the ID 667
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

# Import ArUco module
import cv2.aruco as aruco


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

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Create ArUco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    aruco_parameters = aruco.DetectorParameters_create()

    # While ROS is still running.
    while not rospy.is_shutdown():

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = cap.read()

        if ret == True:
            # Print debugging information to the terminal
            rospy.loginfo('publishing video frame')

            # Detect ArUco markers
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_parameters)

            if ids is not None:
                for i in range(len(ids)):
                    if ids[i] == 667:
                        # Draw a rectangle around the ArUco marker
                        cv2.polylines(frame, [np.int32(corners[i])], True, (0, 255, 0), 3)

                        # Calculate the center of the ArUco marker
                        center = tuple(np.mean(corners[i][0], axis=0).astype(int))
                        cv2.circle(frame, center, 1, (255, 0, 0), 2)

                        # Draw a line from the center of the ArUco marker to the center of the camera
                        camera_center = (frame.shape[1]//2, frame.shape[0]//2)
                        cv2.line(frame, center, camera_center, (255, 0, 0), 2)

                        # Calculate the angle between the line connecting the center of the ArUco marker and the center of the camera and the vertical line through the middle of the camera
                        dx = camera_center[0] - center[0]
                        dy = camera_center[1] - center[1]
                        angle = math.degrees(math.atan2(dy, dx))
                        cv2.putText(frame, f"Angle: {angle:.2f}", (20, 40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                        # Calculate the x-coordinate of the center of the ArUco marker relative to the center of the screen
                        x_offset = center[0] - camera_center[0]
                        cv2.putText(frame, f"X-Offset: {x_offset:.2f}", (20, 80),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS image message
            pub.publish(br.cv2_to_imgmsg(frame, encoding='rgb8'))
            pub2.publish(x_offset)

            # Sleep just enough to maintain the desired rate
            rate.sleep()


if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass

