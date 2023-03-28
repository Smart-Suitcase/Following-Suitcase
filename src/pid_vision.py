#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
from PID import PIDController

# Function to track green object
def track_green_object(frame):
    # Convert frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color range for green
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour
    max_area = 0
    max_contour = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            max_contour = cnt

    # Return the center of the largest green object
    if max_contour is not None:
        M = cv2.moments(max_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return cx, cy
    return None

# Initialize camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# Allow the camera to warm up
time.sleep(0.1)

# Initialize ROS node and publishers
rospy.init_node("motor_control_node")
motor1_pub = rospy.Publisher("motor1", Int16, queue_size=1)
motor2_pub = rospy.Publisher("motor2", Int16, queue_size=1)

# Set up the PID controllers
pid1 = PIDController()
pid2 = PIDController()

pid1.set_Kp(0.2)
pid1.set_Ki(0.0)
pid1.set_Kd(0.0)

pid2.set_Kp(0.2)
pid2.set_Ki(0.0)
pid2.set_Kd(0.0)

pid1.set_sample_time(0.01)
pid2.set_sample_time(0.01)

# Main loop
try:
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array

        # Track green object
        center = track_green_object(image)

        # Calculate error based on object position
        if center is not None:
            error_x = center[0] - image.shape[1] / 2
        else:
            error_x = 0

        # Update PID controllers
        motor1_speed = int(pid1.update(error_x))
        motor2_speed = int(pid2.update(-error_x))

        # Send motor speeds to Arduino
        motor1_pub.publish(motor1_speed)
        motor2_pub.publish(motor2_speed)

        # Clear the stream for the next frame
        rawCapture.truncate(0)

        # Break the loop on KeyboardInterrupt
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except rospy.ROSInterruptException:
    pass

finally:
    # Release resources
    camera.close()
    cv2.destroyAllWindows()