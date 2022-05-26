#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from dynamic_reconfigure.server import Server
from turtlesim_tst_pkg.cfg import move2xy_pyConfig
tt_pose = Pose()

def callback(config, level):
    global kv, kw, Tolerance
    kv = config.kv
    kw = config.kw
    Tolerance = config.Tolerance
    return config


def pose_cb(pose_msg):
    tt_pose.x = pose_msg.x
    tt_pose.y = pose_msg.y                   
    tt_pose.theta = pose_msg.theta               

def dist(tt_pose:Point, p:Point):
    return math.sqrt((p.x-tt_pose.x)**2 + (p.y - tt_pose.y)**2)      # <====

def move2xy(p: Point): # p: goal point
    velocity_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    err_distance = dist(tt_pose, p)
    while(err_distance > Tolerance):
        vel_msg.linear.x = kv * err_distance # P control
        y_diff = p.y - tt_pose.y
        x_diff = p.x - tt_pose.x
        g_theta = math.atan2(y_diff, x_diff)  # <====
        vel_msg.angular.z = kw * (g_theta - tt_pose.theta)                           # <====
        velocity_pub.publish(vel_msg)                    # <====
        err_distance = dist(tt_pose, p)
    print(f"--- Stopped at ({tt_pose.x:.2f}, {tt_pose.y:.2f})")
    vel_msg.linear.x  = 0 # stops the robot
    #vel_msg.angular.z = g_theta[0]                                     # <====
    velocity_pub.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('move2xy_py', anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, pose_cb)
    srv = Server(move2xy_pyConfig, callback)
    while(tt_pose.x == 0.0): # Need some time to get pose data from the turtle
        pass # do nothing

    p = Point()
    while True:
        try:
            p.x = float(input("\nGoal x: "))
            p.y = float(input("Goal y: "))
            
        except: 
            print("Try again...")
        try: move2xy(p)
        except rospy.ROSInterruptException: 
            print("ROS Exception")
            break
