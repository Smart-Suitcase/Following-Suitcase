#!/usr/bin/env python3

PKG = 'suitcase'
import roslib; roslib.load_manifest(PKG)
import rospy
import unittest
from std_msgs.msg import Float64

class TestMyPackage(unittest.TestCase):
    def __init__(self, *args):
        super(TestMyPackage, self).__init__(*args)
        self.image_captured = False

    def setUp(self):
        rospy.init_node('test_listener', anonymous=True)
        self.publisher = rospy.Publisher('range', Float64, queue_size=1)
        self.rate = rospy.Rate(1)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

        # Set a flag if image is captured
        self.image_captured = True

        # Check that image is not captured
        self.assertFalse(self.image_captured)

    # def tearDown(self):
    #     rospy.signal_shutdown('Test complete')

    def test_callback(self):
        # Set the threshold value
        threshold = 4.0

        # Publish sensor data below threshold
        self.publisher.publish(Float64(threshold-1))
        self.rate.sleep()



        # Publish sensor data above threshold
        self.publisher.publish(Float64(threshold+1))
        self.rate.sleep()

        # # Check that image is captured
        # self.assertTrue(self.image_captured)

    def listener(self):
        rospy.Subscriber('range', Float64, self.callback)
        rospy.spin()

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'image_notification', TestMyPackage)
