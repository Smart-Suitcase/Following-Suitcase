#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Float64
import requests
import cv2

flag = True

def capture_and_upload_image():
    url = 'http://46.101.186.178:5001/upload'
    file_path = 'image.jpg'

    # initialize the camera
    camera = cv2.VideoCapture(0)

    # take a photo
    _, frame = camera.read()

    cv2.imwrite("image.jpg", frame)

    with open(file_path, 'rb') as file:
        files = {'file': file}
        response = requests.post(url, files=files)

    # release the camera
    camera.release()

    print(response.status_code)

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
