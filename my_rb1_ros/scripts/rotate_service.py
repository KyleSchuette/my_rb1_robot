#! /usr/bin/env python

import rospy
from my_rb1_ros.srv import Rotate, RotateResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time

current_yaw = None  # global to track robot's orientation

def on_shutdown():
    stop = Twist()
    for _ in range(5):
        my_pub.publish(stop)
        time.sleep(0.05)

def odom_callback(msg):
    global current_yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_yaw = yaw

def rotate_callback(request):
    global current_yaw
    rospy.loginfo("The service 'rotate_robot' has been called")
    target_degrees = request.degrees
    target_radians = math.radians(target_degrees)

    # Wait until we have odom data
    while current_yaw is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    start_yaw = current_yaw
    desired_yaw = start_yaw + target_radians

    twist = Twist()
    twist.angular.z = 0.3 if target_radians > 0 else -0.3  # direction of turn

    # Normalize angles into [-pi, pi]
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    desired_yaw = normalize_angle(desired_yaw)

    rate = rospy.Rate(10)
    rospy.loginfo(f"Rotating robot by {target_degrees} degrees...")
    try:
        target_degrees = request.degrees
        target_radians = math.radians(target_degrees)

        # Wait until we have odom data
        while current_yaw is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        start_yaw = current_yaw
        desired_yaw = start_yaw + target_radians

        twist = Twist()
        twist.angular.z = 0.3 if target_radians > 0 else -0.3

        def normalize_angle(angle):
            return math.atan2(math.sin(angle), math.cos(angle))

        desired_yaw = normalize_angle(desired_yaw)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            error = normalize_angle(desired_yaw - current_yaw)
            if abs(error) < 0.05:  # tolerance ~3°
                break
            my_pub.publish(twist)
            rate.sleep()

        # Stop the robot
        twist.angular.z = 0.0
        my_pub.publish(twist)

        rospy.loginfo("Finished service 'rotate_robot'")
        return RotateResponse(result="success")

    except Exception as e:
        error_msg = f"failure: {str(e)}"
        rospy.logerr(f"Rotation failed: {error_msg}")
        return RotateResponse(result=error_msg)

rospy.init_node('rotate_service_node')
my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
my_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
my_service = rospy.Service('/rotate_robot', Rotate, rotate_callback)
rospy.on_shutdown(on_shutdown)
rospy.loginfo("Service /rotate_robot Ready")
rospy.spin()