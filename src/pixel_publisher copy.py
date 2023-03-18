#!/usr/bin/env python3

# Import required ROS libraries
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class PixelPublisher:
    def __init__(self):
        # Set up the video publisher
        self.video_pub = rospy.Publisher('video_frames', Image, queue_size=10)
        self.x_pixel_pub = rospy.Publisher('x_pixel', Int16, queue_size=10)

        # Initialize the ROS node
        rospy.init_node('pixel_pub', anonymous=True)

        # Set the loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

        # Set up the background subtractor
        self.fgbg = cv2.createBackgroundSubtractorMOG2()

        # Set up the ROS image converter
        self.br = CvBridge()

        self.ret, self.frame = cv2.VideoCapture(0).read()

    def run(self):

        while not rospy.is_shutdown():

            if self.ret == True:

                # Print debugging information to the terminal
                rospy.loginfo('publishing video frame')
                # Process the frame
                processed_frame, x_pixel = self.process_frame(self.frame)

                # Publish the processed frame
                self.publish_frame(processed_frame, x_pixel)

            # Wait until the next loop iteration
            self.rate.sleep()

    def image_processing(self):
        pass

    def __apply_filter(self, frame, lower_color=np.array([45, 50, 50]), upper_color=np.array([75, 255, 255])):

        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # # Define the range of green color in the HSV color space
        # lower_color = np.array([45, 50, 50])
        # upper_color = np.array([75, 255, 255])

        # Threshold the HSV image to get only green color
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Apply background subtraction
        fgmask = self.fgbg.apply(mask)

        return fgmask

    def __find_countours(self, fgmask):
        # Find contours
        contours, _ = cv2.findContours(
            fgmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        return contours

    def __find_largest_contour(self, contours, min_area):
        # Find the largest contour
        largest_contour = None
        largest_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > largest_area and area > min_area:
                largest_area = area
                largest_contour = cnt
        return largest_contour

    def __draw_lines(self, frame, largest_contour, cap):
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        # Draw the circle at the center of the image
        cv2.circle(frame,
                   (int(width/2),
                    int(height/2)),
                   1,
                   (255, 0, 0), 2)

        # Draw a rectangle around the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        x_offset = 0

        # Draw a rectangle around the largest contour
        if largest_contour is not None:
            # print("le centre de l'objet est à ", int(x+w/2), int(y+h/2))
            cv2.circle(frame, (int(x+w/2), int(y+h/2)), 1, (255, 0, 0), 2)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
            cv2.line(frame, (int(x+w/2), int(y+h/2)),
                     (int(width/2), int(height/2)), (255, 0, 0), 1)
            print(int(width/2)-int(x+w/2), int(height/2)-int(y+h/2))

            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS image message
            center = (x + w//2, y + h//2)
            # Draw a line from the center of the rectangle to the center of the camera
            camera_center = (frame.shape[1]//2, frame.shape[0]//2)
            cv2.line(frame, center, camera_center, (255, 0, 0), 2)
        return camera_center, center

    def __compute_x_offset(self, frame, camera_center, center):
        x_offset = center[0] - camera_center[0]
        cv2.putText(frame, f"X-Offset: {x_offset:.2f}", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        return x_offset

    def __publish_frame(self, frame, x_pixel):
        # Publish the frame
        self.video_pub.publish(self.br.cv2_to_imgmsg(frame, encoding='rgb8'))
        self.x_pixel_pub.publish(x_pixel)


if __name__ == '__main__':

    try:
        pixelpublisher = PixelPublisher()
        pixelpublisher.run()
    except rospy.ROSInterruptException:
        pass
