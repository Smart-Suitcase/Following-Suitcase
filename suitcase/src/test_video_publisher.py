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
from pyzbar.pyzbar import decode


def publish_message():

    # Node is publishing to the video_frames topic using
    # the message type Image
    pub = rospy.Publisher('video_frames', Image, queue_size=10)
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
    # print(width, height)
    fgbg = cv2.createBackgroundSubtractorMOG2()

    # Used to convert between ROS and OpenCV images
    br = CvBridge()


    while not rospy.is_shutdown():

        # Capture frame-by-frame
        ret, frame = cap.read()

        if ret == True:
            # Print debugging information to the terminal
            rospy.loginfo('publishing video frame')

            # Decode QR codes in the frame
            decoded_qr_codes = decode(frame)

            for d in decoded_qr_codes:
                if d.data.decode('utf-8') == 'suitcase':
                    x, y, w, h = d.rect
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    center = (x + w//2, y + h//2)
                    camera_center = (frame.shape[1]//2, frame.shape[0]//2)
                    cv2.line(frame, center, camera_center, (255, 0, 0), 2)

                    dx = camera_center[0] - center[0]
                    dy = camera_center[1] - center[1]
                    angle = math.degrees(math.atan2(dy, dx))
                    cv2.putText(frame, f"Angle: {angle:.2f}", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

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
