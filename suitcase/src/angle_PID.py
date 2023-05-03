#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from simple_pid import PID

##### PID variables #####
Kp = 0.5
Ki = 0
Kd = 0.01
SP = 0
#########################

class FollowingRobot:
    
    def __init__(self):
        self.ls = rospy.Publisher('left_speed', Float64, queue_size=10)
        self.rs = rospy.Publisher('right_speed', Float64, queue_size=10)
        self.ld = rospy.Publisher('left_direction', UInt16, queue_size=10)
        self.rd = rospy.Publisher('right_direction', UInt16, queue_size=10)
        rospy.Subscriber('x_pixel', Int16, self.callback)
        rospy.init_node('pid', anonymous=True)
        self.pwm_value = 0
        self.rate = rospy.Rate(20)
        self.pid = PID(Kp, Ki, Kd, setpoint=SP, output_limits=(-255,255))
        

    def callback(self, data):
        #received data
        rospy.loginfo("Received : %s", data.data)
        #sent data
        self.pwm_value = self.pid(data.data)
        if self.pwm_value >= 0:
            self.ld.publish(2)
            self.rd.publish(1)
            self.ls.publish(self.pwm_value)
            self.rs.publish(self.pwm_value)
        else:
            self.ld.publish(1)
            self.rd.publish(2)
            self.ls.publish(- self.pwm_value)
            self.rs.publish(- self.pwm_value)
        #sleep
        rospy.loginfo("Sent : %s", self.pwm_value)
        self.rate.sleep()

if __name__ == '__main__':
    robot = FollowingRobot()
    while not rospy.is_shutdown():
        rospy.spin()
