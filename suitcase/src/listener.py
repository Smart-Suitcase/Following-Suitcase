#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Float64
import requests
import cv2
import base64
import socketio

flag = True

sio = socketio.Client()
sio.connect('http://192.168.4.5:5000')

def capture_and_upload_image():
    file_path = 'image.jpg'

    # initialize the camera
    camera = cv2.VideoCapture(0)

    # take a photo
    _, frame = camera.read()

    cv2.imwrite(file_path, frame)

    with open(file_path, 'rb') as file:
        encoded_image = base64.b64encode(file.read()).decode('utf-8')
        sio.emit('image', encoded_image)

    # release the camera
    camera.release()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

    startTime = 100000

    threshold = 4.0  # Set the threshold value here
    if time.time()-startTime > 10:
        flag = True

    # Check if the threshold has been reached
    if float(data.data) >= threshold and flag == True:
        capture_and_upload_image()
        flag = False
        startTime = time.time()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('range', Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()