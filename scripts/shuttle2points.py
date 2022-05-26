#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
tt_pose = Pose()

def pose_cb(pose_msg):
    tt_pose.x = pose_msg.x
    tt_pose.y = pose_msg.y
    tt_pose.theta = pose_msg.theta

def go_fb(vel, distance):
    velocity_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = vel

    if vel > 0:
        goal_x = tt_pose.x + distance
        rospy.loginfo(f"Goal x = {goal_x:.2f}     Current x = {tt_pose.x:.2f}")
        while(tt_pose.x <= goal_x): # Loop to move the turtle in an specified location
            velocity_pub.publish(vel_msg)
    else:
        goal_x = tt_pose.x + (distance * -1)
        rospy.loginfo(f"Goal x = {goal_x:.2f}     Current x = {tt_pose.x:.2f}")
        while(tt_pose.x >= goal_x): # Loop to move the turtle in an specified location
            velocity_pub.publish(vel_msg)


    vel_msg.linear.x = 0 # After the loop, stops the robot
    velocity_pub.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('shuttle2points', anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, pose_cb)
    rospy.loginfo(f"({tt_pose.x}, {tt_pose.y})")
    while(tt_pose.x == 0.0): # Need some time to get pose data from the turtle
        pass # do nothing
    
    go_fb(3,5)
    
    for _ in range(3):
        go_fb(-3,10)
        go_fb(+3,10)
