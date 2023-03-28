import rospy
from std_msgs.msg import String
import requests
import cv2
import time

last_time = time.time()

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

    threshold = 6  # Set the threshold value here

    # Check if the threshold has been reached
    if int(data.data) >= threshold and time.time() - last_time > 5:
        capture_and_upload_image()
        last_time = time.time()
        rospy.loginfo("sent")
    else:
        rospy.loginfo("not sent")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('range', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
