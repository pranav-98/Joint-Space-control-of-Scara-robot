#! usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
#from rbe500_project.srv import *
import math
from std_msgs.msg import String, Float64


class Controller:
    def __init__(self, P=0.0, I=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.Ki
        self.set_point = set_point 
        self.previous_error = 0
        self.total_error=0

    def update(self, current_value):
        error = self.set_point - current_value
        P_term = self.Kp*error
        D_term = self.Kd*(error - self.previous_error)
        self.previous_error = error
        self.total_error+=error
        I_term+=self.Ki*(self.total_error)
        return P_term + D_term+I_term
    
    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D
        self.Ki=I


class RRP():
    def __init__(self):
        
        rospy.init_node("Scara_move", anonymous=True)
        rospy.loginfo("Press Ctrl + C to terminate")
        #rospy.sleep(5)
        self.joint_angle_pub = rospy.Publisher("/rrp/joint1_effort_controller/command", Float64, queue_size=10)
        self.rate = rospy.Rate(10)
                

        self.jointstate=JointState()
        self.logging_counter = 0
        self.trajectory = list()
        # print("HERE")
        self.joint_angle_sub = rospy.Subscriber("/rrp/joint_states", JointState, self.joint_callback, queue_size=10)
        print("THERE")
        
        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt("trajectory.csv", np.array(self.trajectory), fmt="%f", delimiter=",")
            rospy.loginfo("Trajectory Saved")

    
            
        
    def run(self):
        
        print("Reached the run fn")
        print(self.jointstate.position)        
        rospy.loginfo("Run successfully completed")

    
    
    def joint_callback(self, msg):
        
        self.jointstate.position=msg.position
        print("Callback")
        print(self.jointstate.position)
        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0     
        
    
    
	
	

#def calculateik(x,y,z):
#	rospy.wait_for_service("ik_solver")
#	try:
#		inverse=rospy.ServiceProxy("ik_solver",ik)
#		out=inverse(x,y,z)
#		return out
#	except rospy.ServiceException as e:
#	        print("Service call failed: %s"%e)
		
if __name__=="__main__":
    obj=RRP()
		
	
	
		
