#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import UInt16

LeftSpeed = rospy.Publisher('left_speed', Float64, queue_size=10)
RightSpeed = rospy.Publisher('right_speed', Float64, queue_size=10)

LeftSpeed.publish(0)
LeftSpeed.publish(0)