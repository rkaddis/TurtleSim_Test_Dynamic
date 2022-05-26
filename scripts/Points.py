#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point
if __name__ == '__main__':
    rospy.init_node('Points_py', anonymous=True)
    p1 = Point()
    p1.x = float(input("Input p1 x: "))
    p1.y = float(input("Input p1 y: "))
    p2 = Point()
    p2.x = float(input("Input p2 x: "))
    p2.y = float(input("Input p2 y: "))
    d = math.sqrt( (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) )
    print(f"distance = {d}")