#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
from lib.Image import Image_ROS
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

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    image = Image_ROS(0)
    # While ROS is still running.
    while not rospy.is_shutdown():

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        # ret, frame = cap.read()
        image.update_frame()

        if image.ret == True:
            image.image_processing()

        pub.publish(br.cv2_to_imgmsg(image.frame, encoding='rgb8'))
        pub2.publish(image.x_offset)

        # Sleep just enough to maintain the desired rate
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.loginfo('publishing video frame')
        publish_message()
    except rospy.ROSInterruptException:
        pass
