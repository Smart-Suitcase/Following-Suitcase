#!/usr/bin/env python

import unittest
import rospy
import rostest
import sys
from std_msgs.msg import Float64
from std_msgs.msg import UInt16




class TestMyNode(unittest.TestCase):
    """
    Classe de test pour le nœud 'MyNode'.

    Cette classe de test vérifie si la valeur de la distance est de 2 mètres, 
    ce qui entraîne une valeur PWM de 0 car nous souhaitons que la valise ne bouge pas si nous sommes à 2 mètres de celle-ci.
    """
    def __init__(self, *args):
        super(TestMyNode, self).__init__(*args)
        self.left_speed_value = 0
        self.right_speed_value = 0

    def left_speed_callback(self, msg):
        print('leftspeed received',msg.data)
        self.left_speed_value = msg.data
        self.assertEqual(self.left_speed_value, 0.0)
	 

    def right_direction_callback(self, msg):
        print('right_direction received',msg.data)
        self.right_direction_value = msg.data
        self.assertEqual(self.right_direction_value, 0)

    def left_direction_callback(self, msg):
        print('left_direction received',msg.data)
        self.left_direction_value = msg.data
        self.assertEqual(self.left_direction_value, 0)
	 

    def right_speed_callback(self, msg):
        print('rightspeed received',msg.data)
        self.right_speed_value = msg.data
        self.assertEqual(self.right_speed_value, 0.0)
    
    def range_callback(self, msg):
        print('Range',msg.data)
        self.range_value = msg.data
        self.assertEqual(self.range_value, 2.0)

    def setUp(self):
        rospy.init_node('test_my_node')
        rospy.Subscriber('/left_speed', Float64, self.left_speed_callback)
        rospy.Subscriber('/right_speed', Float64, self.right_speed_callback)
        rospy.Subscriber('/left_direction', UInt16, self.left_direction_callback)
        rospy.Subscriber('/right_direction', UInt16, self.right_direction_callback)
        rospy.Publisher('/range', Float64, self.range_callback)
        self.left_speed = rospy.Publisher('/left_speed', Float64, queue_size=10)
        self.right_speed = rospy.Publisher('/right_speed', Float64, queue_size=10)
        self.left_direction = rospy.Publisher('/left_direction', UInt16, queue_size=10)
        self.right_direction = rospy.Publisher('/right_direction', UInt16, queue_size=10)

    def test_my_node(self):
        # Vérifiez que les vitesses de gauche et de droite sont à zéro quand la distance est de 2 mètres
        self.range.publish(2.0)
        self.assertEqual(self.left_speed_value, 0.0)
        self.assertEqual(self.right_speed_value, 0.0)
        self.assertEqual(self.left_direction_value, 0)
        self.assertEqual(self.right_direction_value, 0)




if __name__ == '__main__':
    rostest.rosrun('suitcase', 'test_following_robot', TestMyNode, sysargs=None)