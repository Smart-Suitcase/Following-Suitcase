#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
# Modified to detect ArUco tag with ID 667 and stream video over the network

# Import the necessary libraries
import rospy  # Python library for ROS
from sensor_msgs.msg import CompressedImage  # CompressedImage is the message type
from std_msgs.msg import Int16  # Image is the message type
import cv2  # OpenCV library
import numpy as np
import math

def publish_message():

    # Node is publishing to the video_frames topic using
    # the message type CompressedImage
    pub = rospy.Publisher('video_frames/compressed', CompressedImage, queue_size=10)
    pub2 = rospy.Publisher('pixel', Int16, queue_size=10)
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
    
    # Load the ArUco dictionary and parameters
    # aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    aruco_params = cv2.aruco.DetectorParameters_create()

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
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

            # Draw detected markers
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            if ids is not None:
                # Find the ArUco tag with ID 667
                for i, tag_id in enumerate(ids.flatten()):
                    if tag_id == 667:
                        # Get the center of the detected ArUco tag
                        center = tuple(np.mean(corners[i][0], axis=0).astype(int))

                        # Draw a circle at the center of the ArUco tag
                        cv2.circle(frame, center, 3, (0, 0, 255), 2)

                        # Calculate the x-coordinate of the center of the rectangle relative to the center of the screen
                        camera_center = (frame.shape[1] // 2, frame.shape[0] // 2)
                        x_offset = center[0] - camera_center[0]

                        # Publish the x_offset
                        pub2.publish(x_offset)

            # Encode the frame as a compressed JPEG image
            compressed_frame = cv2.imencode('.jpg', frame)[1].tobytes()

            # Create a CompressedImage message and publish the encoded frame
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = compressed_frame
            pub.publish(msg)

        # Sleep just enough to maintain the desired rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
