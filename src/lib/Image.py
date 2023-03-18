#!/usr/bin/env python3
import cv2
import numpy as np


class Image_ROS:
    def __init__(self, port_camera):
        self.cap = cv2.VideoCapture(port_camera)
        self.ret, self.frame = self.cap.read()
        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.camera_center = (self.frame.shape[1]//2, self.frame.shape[0]//2)
        self.x_offset = 0

    def image_processing(self):
        mask_image = self.__apply_filter(
            np.array([45, 50, 50]), np.array([45, 50, 60]))
        contours = self.__find_countours(mask_image)
        largest_contour = self.__find_largest_contour(contours, 700)
        center_contour, x_rectangle, y_rectangle, w_rectangle, h_rectangle = self.__find_center_contour(
            largest_contour)
        self.__draw_lines(largest_contour, x_rectangle,
                          y_rectangle, w_rectangle, h_rectangle, center_contour)
        self.x_offset = self.__compute_x_offset(center_contour)

    def update_frame(self):
        self.ret, self.frame = self.cap.read()

    def __apply_filter(self, lower_color, upper_color):

        # Set up the background subtractor
        fgbg = cv2.createBackgroundSubtractorMOG2()

        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only green color
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Apply background subtraction
        fgmask = fgbg.apply(mask)

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
                print(area)
                largest_contour = cnt
        return largest_contour

    def __find_center_contour(self, largest_contour):

        # Draw a rectangle around the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)

        # image to a ROS image message
        center_contour = (x + w//2, y + h//2)
        print(center_contour)
        return center_contour, x, y, w, h

    def __draw_lines(self, largest_contour, x, y, w, h, center_contour):

        # Draw the circle at the center of the image
        cv2.circle(self.frame,
                   (int(self.width/2),
                    int(self.height/2)),
                   1,
                   (255, 0, 0), 2)

        # Draw a rectangle around the largest contour
        if largest_contour is not None:
            # print("le centre de l'objet est à ", int(x+w/2), int(y+h/2))
            cv2.circle(self.frame, (int(x+w/2), int(y+h/2)), 1, (255, 0, 0), 2)
            cv2.rectangle(self.frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
            cv2.line(self.frame, (int(x+w/2), int(y+h/2)),
                     (int(self.width/2), int(self.height/2)), (255, 0, 0), 1)
            print(int(self.width/2)-int(x+w/2), int(self.height/2)-int(y+h/2))

            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV

            # Draw a line from the center of the rectangle to the center of the camera

            cv2.line(self.frame, center_contour,
                     self.camera_center, (255, 0, 0), 2)

    def __compute_x_offset(self, center_contour):
        self.x_offset = center_contour[0] - self.camera_center[0]
        cv2.putText(self.frame, f"X-Offset: {self.x_offset:.2f}", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        return self.x_offset
