#!/usr/bin/env python
# -*- coding: utf-8 -*-
from random import uniform

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class PID:
    error = 0.0
    error_deriv = 0.0
    dt = 1e-3
    previous_error = 0.0
    previous_error_deriv = 0.0
    sum_error = 0.0

    def __init__(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD

    def __steering_angle__(self):
        """
        Calculates steering angle that's define by the sum of the
        gains of p, i and d.
        """
        p_gain = self.kP * self.error
        self.sum_error += self.kI * self.error * self.dt
        i_gain = self.sum_error
        d_gain = self.kD * self.error_deriv
        print('Steering', p_gain, i_gain, d_gain)

        return p_gain + i_gain + d_gain

    def __call__(self, error):
        """
        Updates the values for the errors and the deriv, as well as returns the
        current steering angle for the robot
        """
        self.error_deriv = (self.error - self.previous_error) / self.dt

        # Calculates steering angle
        output = self.__steering_angle__()

        # Update error
        self.previous_error = self.error
        self.error = error
        self.previous_error_deriv = self.error_deriv

        return output

class PIDController:
    def __init__(self):
        rospy.init_node('follower', anonymous=True)

        self.forward_speed = 0.30
        self.desired_distance_from_wall = 0.1
        self.hertz = 5

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.right_wall_pid = PID(0.75, 2e-4, 0.2)

    def __move__(self, angle):
        print('Speed', self.forward_speed)
        print('Angle', angle)
        linear = Vector3(self.forward_speed, 0, 0)
        angular = Vector3(0, 0, angle)
        twist = Twist(linear, angular)
        self.cmd_pub.publish(twist)

    def __right_wall_following__(self, closest_right):
        print('Doing right wall following', closest_right)
        cte_r = closest_right - self.desired_distance_from_wall
        angle = self.right_wall_pid(cte_r) * -1
        self.__move__(angle)

    def __wander__(self):
        angle = uniform(-1, 1) * 30
        self.__move__(angle)

    def laser_callback(self, msg):
        # closest_front = min(msg.ranges[690:] + msg.ranges[:30])
        # closest_front = min(closest_front, 3)
        closest_right = min(msg.ranges[480:660])
        closest_right = min(closest_right, 3)

        if closest_right < 3:
            self.__right_wall_following__(closest_right)
        else:
            self.__wander__()

    def run(self):
        rate = rospy.Rate(self.hertz)
        rospy.spin()
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    follower = PIDController()
    follower.run()
