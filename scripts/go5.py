#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def go5():
    rospy.init_node('go5.py', anonymous = True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    speed = input("Input your speed:")
    vel_msg.linear.x = abs(float(speed))

    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    while(current_distance < 5):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = float (speed) * (t1-t0)
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try: go5()
    except rospy.ROSInterruptException: pass