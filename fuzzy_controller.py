#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

def frange(start, stop, step):
    """
    Frange exists because range is dumb and can't handle
    float steps
    """
    while start < stop:
        yield start
        start += step

def centroid(function, low, high, step=0.1):
    """Calculates the centroid of a function"""
    numerator, denominator = 0, 0
    for value in frange(low, high, step):
        output = function(value)
        numerator += value * output
        denominator += output
    return numerator / denominator

def trapezoid(x, a, b, c=None, d=None):
    """Representation of a trapezoidal membership function"""
    if a is not None:
        if x <= a:
            return 0
        if a < x < b:
            return (x - a) / (b - a)
    if c is None or b <= x <= c:
        return 1
    if d is None or x <= d:
        c, d = (c, d) if d is not None else (b, c)
        return (x - c) / (d - c)
    return 0

def triangle(x, a, b, c):
    """Representation of a triangular membership function"""
    return 0 if x < a else \
        (x - a) / (b - a) if a <= x <= b else \
        (x - b) / (c - b) if b < x < c else 0

class FuzzyDistance:
    """Class for managing the fuzzy logic of distance"""
    labels = ['close', 'far']

    @staticmethod
    def close(value):
        """Returns close membership function"""
        return trapezoid(value, None, 0, 0.5, 1.0)

    @staticmethod
    def far(value):
        """Returns far membership function"""
        return trapezoid(value, 1.0, 1.5)

    @staticmethod
    def get_fuzzy_set(value):
        """Get fuzzy sets"""
        return FuzzyDistance.close(value), FuzzyDistance.far(value)

class FuzzyDirection:
    """Class for managing the fuzzy logic of direction"""
    @staticmethod
    def left(value):
        """Returns left membership function"""
        return 0 if value <= 0 else value if value <= 1 else 1

    @staticmethod
    def front(value):
        """Returns left membership function"""
        return triangle(value, -1, 0, 1)

    @staticmethod
    def right(value):
        """Returns right membership function"""
        return trapezoid(value, None, -1, -1, 0)

direction_centroids = [
    centroid(FuzzyDirection.left, 0, 1, 0.001),
    centroid(FuzzyDirection.front, -1, 1, 0.001),
    centroid(FuzzyDirection.right, -1, 0, 0.001)
]
get_direction_centroid = lambda i: direction_centroids[i]

class FuzzySpeed:
    """Class for managing the fuzzy logic of speed"""
    @staticmethod
    def slow(value):
        """Returns slow membership function"""
        return trapezoid(value, None, 0, 0, 0.2)

    @staticmethod
    def fast(value):
        """Returns slow membership function"""
        return trapezoid(value, 0.2, 0.3)

speed_centroids = [
    centroid(FuzzySpeed.slow, 0, 0.2, 0.001),
    centroid(FuzzySpeed.fast, 0.2, 0.3, 0.001)
]
get_speed_centroid = lambda i: speed_centroids[i]

def centroid_deffuzifier(fire_strengths, centroids):
    """Defuzzify centroid based on the fire_strengths and the centroids"""
    numerator, denominator = 0, 0
    for index, fire_strength in enumerate(fire_strengths):
        numerator += fire_strength * centroids[index]
        denominator += fire_strength
    return numerator / denominator

def get_behavior_vel_and_dir_crisp(fire_s, vel_consequence, dir_consequence):
    """Get the vel and dir crisp outputs"""
    vel_centroids = list(map(get_speed_centroid, vel_consequence))
    vel_crisp_output = centroid_deffuzifier(fire_s, vel_centroids)

    dir_centroids = list(map(get_direction_centroid, dir_consequence))
    dir_crisp_output = centroid_deffuzifier(fire_s, dir_centroids)

    return vel_crisp_output, dir_crisp_output

def ref_behavior(rs_set, fs_set):
    """
    Follows the right edge following behavior

    @arg rs_set: Fuzzy set of the right sensor
    @arg fs_set: Fuzzy set of the front sensor
    """
    def get_fire_strength(rule):
        return min(rs_set[rule[0]], fs_set[rule[1]])

    rule_set = [(0, 0), (0, 1), (1, 0), (1, 1)]
    fire_strengths = list(map(get_fire_strength, rule_set))
    vel_consequence = [0, 0, 0, 1]
    dir_consequence = [0, 1, 0, 2]

    return get_behavior_vel_and_dir_crisp(fire_strengths, vel_consequence, dir_consequence)

def oa_behavior(rs_set, fs_set, ls_set):
    """
    Follows the obstacle avoidance behavior

    @arg rs_set: Fuzzy set of the right sensor
    @arg fs_set: Fuzzy set of the front sensor
    @arg ls_set: Fuzzy set of the left sensor
    """
    def get_fire_strength(rule):
        return min(rs_set[rule[0]], fs_set[rule[1]], ls_set[rule[2]])

    rule_set = [
        (0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1),
        (1, 0, 0), (1, 0, 1), (1, 1, 0), (1, 1, 1)
    ]
    fire_strengths = list(map(get_fire_strength, rule_set))
    vel_consequence = [0, 0, 1, 1, 0, 0, 1, 1]
    dir_consequence = [0, 2, 1, 1, 0, 0, 1, 1]

    return get_behavior_vel_and_dir_crisp(fire_strengths, vel_consequence, dir_consequence)

class FuzzyContext:
    @staticmethod
    def oa(value):
        """OA behavior membership function"""
        return trapezoid(value, None, 0, 1, 5)

    @staticmethod
    def ref(value):
        """REF behavior membership function"""
        return trapezoid(value, 0, 2)

def context_blending(closest_distance, ref_out, oa_out):
    """
    For context blending

    @arg closest_distance: Closest distance to the robot
    @arg ref_out: Tuple containing the vel and dir output of ref
    @arg oa_out: Tuple containing the vel and dir output of oa
    """
    ref_fs = FuzzyContext.ref(closest_distance)
    oa_fs = FuzzyContext.oa(closest_distance)
    fs_sum = ref_fs + oa_fs

    vel_crisp_output = (ref_out[0] * ref_fs + oa_out[0] * oa_fs) / fs_sum
    dir_crisp_output = (ref_out[1] * ref_fs + oa_out[1] * oa_fs) / fs_sum

    return vel_crisp_output, dir_crisp_output

class FuzzyController:
    def __init__(self):
        rospy.init_node('follower', anonymous=True)

        self.hertz = 5

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def __move__(self, angle, speed):
        linear = Vector3(speed * 2, 0, 0)
        angular = Vector3(0, 0, angle)
        twist = Twist(linear, angular)
        self.cmd_pub.publish(twist)

    def laser_callback(self, msg):
        min_front = min(msg.ranges[:90] + msg.ranges[630:])
        min_left = min(msg.ranges[90:270])
        min_right = min(msg.ranges[450:630])

        values = [min_front, min_left, min_right]
        closest_distance = min(values)

        fs_set, ls_set, rs_set = list(map(FuzzyDistance.get_fuzzy_set, values))
        ref_out = ref_behavior(rs_set, fs_set)
        oa_out = oa_behavior(rs_set, fs_set, ls_set)

        speed, angle = context_blending(closest_distance, ref_out, oa_out)
        self.__move__(angle, speed)

    def run(self):
        rate = rospy.Rate(self.hertz)
        rospy.spin()
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    follower = FuzzyController()
    follower.run()
