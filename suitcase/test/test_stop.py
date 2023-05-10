#!/usr/bin/env python

import unittest
import rospy
import rostest
import sys
from std_msgs.msg import Float64

class TestMyNode(unittest.TestCase):
    def __init__(self, *args):
        super(TestMyNode, self).__init__(*args)
        self.left_speed_value = 0
        self.right_speed_value = 0

    def left_speed_callback(self, msg):
        print('leftspeed received',msg.data)
        self.left_speed_value = msg.data
        self.assertEqual(self.left_speed_value, 1.0)

    def right_speed_callback(self, msg):
        print('rightspeed received',msg.data)
        self.right_speed_value = msg.data
        self.assertEqual(self.right_speed_value, 2.0)

    def setUp(self):
        rospy.init_node('test_my_node')
        rospy.Subscriber('/left_speed', Float64, self.left_speed_callback)
        rospy.Subscriber('/right_speed', Float64, self.right_speed_callback)
        self.left_speed = rospy.Publisher('/left_speed', Float64, queue_size=10)
        self.right_speed = rospy.Publisher('/right_speed', Float64, queue_size=10)

    def test_my_node(self):
        # Vérifiez que les vitesses de gauche et de droite sont initialement à zéro
        self.assertEqual(self.left_speed_value, 0.0)
        self.assertEqual(self.right_speed_value, 0.0)

        # Publiez une vitesse de gauche de 1,0 et une vitesse de droite de 2,0
        self.left_speed.publish(1.0)
        self.right_speed.publish(2.0)



if __name__ == '__main__':
    rostest.rosrun('suitcase', 'test_following_robot', TestMyNode, sysargs=None)
