#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
tt_pose = Pose()

def pose_cb(pose_msg):
    tt_pose.x = pose_msg.x
    tt_pose.y = pose_msg.y
    tt_pose.theta = pose_msg.theta

def rotate(angular_vel, degrees): # rotate 'degrees' counterclockwise using fblc
    velocity_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.angular.z = angular_vel

    goal_theta = degrees * np.pi / 180.0 # in radians
    rospy.loginfo(f"Goal theta = {goal_theta:.2f}     Current theta = {tt_pose.theta:.2f}")
    while(tt_pose.theta <= goal_theta): # Loop to rotate the turtle in an specified angle
        velocity_pub.publish(vel_msg)
        # ROS Python does not need spinOnce(). No spinOnce() in ROS Python
    vel_msg.angular.z = 0 # After the loop, stops the robot
    velocity_pub.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('spin45fb_py', anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, pose_cb)
    while(tt_pose.x == 0.0): # Need some time to get pose data from the turtle
        pass # do nothing
    try: rotate(1,45) # angular velocity, degrees to rotate
    except rospy.ROSInterruptException: pass
    rospy.spin() 