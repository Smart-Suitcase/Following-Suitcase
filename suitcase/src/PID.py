#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64, Int16, UInt16
from simple_pid import PID

##### PID variables #####
Kp_d = 500.0
Ki_d = 0.1
Kd_d = 0.05
SP_d = 2.0

Kp_a = 0.5
Ki_a = 0
Kd_a = 0.01
SP_a = 0
#########################

class FollowingRobot:
    def __init__(self):
        self.LeftSpeed = rospy.Publisher('left_speed', Float64, queue_size=10)
        self.RightSpeed = rospy.Publisher('right_speed', Float64, queue_size=10)
        self.LeftDirection = rospy.Publisher('left_direction', UInt16, queue_size=10)
        self.RightDirection = rospy.Publisher('right_direction', UInt16, queue_size=10)
        rospy.Subscriber('range', Float64, self.range_callback)
        rospy.Subscriber('x_pixel', Int16, self.pixel_callback)
        rospy.init_node('pid', anonymous=True)
        self.average_pwm = 0
        self.rotation_pwm = 0
        self.left_pwm = 0
        self.right_pwm = 0
        self.rate = rospy.Rate(25)
        self.distance_pid = PID(Kp_d, Ki_d, Kd_d, setpoint=SP_d, output_limits=(-235,235))
        self.angle_pid = PID(Kp_a, Ki_a, Kd_a, setpoint=SP_a, output_limits=(-100,100))
    
    def update_speed(self):
        self.left_pwm = self.average_pwm + self.rotation_pwm
        self.right_pwm = self.average_pwm - self.rotation_pwm
        if self.left_pwm >= 0:
            self.LeftDirection.publish(2)
            self.LeftSpeed.publish(self.left_pwm)
        else:
            self.LeftDirection.publish(1)
            self.LeftSpeed.publish(- self.left_pwm)
        if self.right_pwm >= 0:
            self.RightDirection.publish(2)
            self.RightSpeed.publish(self.right_pwm)
        else:
            self.RightDirection.publish(1)
            self.RightSpeed.publish(- self.right_pwm)

    def range_callback(self, data):
        #received data
        rospy.loginfo("Received: %s", data.data)
        #sent data
        self.average_pwm = - self.distance_pid(data.data)
        self.update_speed()
        rospy.loginfo("Average pwm: {} ; Rotation pwml: {}".format(self.average_pwm, self.rotation_pwm))
        rospy.loginfo("Sent: {} {}".format(self.left_pwm, self.right_pwm))
        #sleep
        self.rate.sleep()
    
    def pixel_callback(self, data):
        #received data
        rospy.loginfo("Received: %s", data.data)
        #sent data
        self.rotation_pwm = self.angle_pid(data.data)
        self.update_speed()
        rospy.loginfo("Average pwm: {} ; Rotation pwml: {}".format(self.average_pwm, self.rotation_pwm))
        rospy.loginfo("Sent: {} {}".format(self.left_pwm, self.right_pwm))
        #sleep
        self.rate.sleep()

if __name__ == '__main__':
    robot = FollowingRobot()
    while not rospy.is_shutdown():
        rospy.spin()