#!/usr/bin/env python3

import rospy
import numpy as np
from rbe500_project.srv import ik, ikResponse
from math import cos, sin, atan2, acos, sqrt,pi



def solveik(req):
    x=req.x
    y=req.y
    z=req.z
    d3= 0.39-z
    theta_2= acos(((x**2+y**2-0.29965)/(0.29325)))
    theta_1=atan2(y,x)-atan2((0.345*sin(theta_2)),((0.345*cos(theta_2))+0.425))
    if x<0 and y<0 :
        theta_1=theta_1+2*pi
    return ikResponse(theta_1, theta_2, d3)

def main():
    rospy.init_node("Ik_calculate", anonymous=True)
    s=rospy.Service("ik_solver",ik, solveik)
    rospy.spin()


if __name__=="__main__":
    main()	
	
