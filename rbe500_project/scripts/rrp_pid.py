#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from rbe500_project.srv import ik, ikResponse
from std_msgs.msg import Float64
 

# PID Controller Class 
class Controller:
    def __init__(self, P=0.0,I=0.0,D=0.0, set_point=0,j_t=0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.set_point = set_point 
        self.previous_error = 0
        self.error_sum=0
        self.jt=j_t #Joint type-0 revolute ;1 -Prismatic

    def update(self, current_value):
        error = self.set_point - current_value
        #to avoid Integral wind up problem.Resetting integral amount 
        if self.jt==0:
            if abs(error)>0.15:
                #accumulation of error
                self.error_sum=self.error_sum+error
            else:
                self.error_sum=0
        else:
            
            if abs(error)>0.002:
                self.error_sum=self.error_sum+error
            else:
                self.error_sum=0
        P_term = self.Kp*error
        I_term = self.Ki*self.error_sum
        D_term = self.Kd*(error - self.previous_error)
        self.previous_error = error
        return P_term + I_term + D_term
    
    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0,I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

class RRP():
    def __init__(self):
        rospy.init_node("RRP_run")
        self.theta1 = 0
        self.theta2 = 0
        self.d3 = 0
        # Joint torque and joint force
        self.jt_tq_1=Float64()
        self.jt_tq_2=Float64()
        self.jt_F=Float64()
        #Accessing joint effort Publisher topic 
        self.theta1_pub= rospy.Publisher("/rrp/joint1_effort_controller/command", Float64, queue_size=10)
        self.theta2_pub= rospy.Publisher("/rrp/joint2_effort_controller/command", Float64, queue_size=10)
        self.d_3_pub= rospy.Publisher("/rrp/joint3_effort_controller/command", Float64, queue_size=10)

        # subscribe to joint_states
        self.joint_value=rospy.Subscriber("/rrp/joint_states", JointState, self.joint_states_callback,queue_size=10)
        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        
    def run(self):
        
        waypoints=[[0,0.77, 0.34],[-0.345, 0.425, 0.24], [-0.67,-0.245, 0.14], [0.77,0.0,0.39]]
        for x,y,z in waypoints:
            #Joint angles = inverse kinematics output
            joint_set=self.calculateik(x,y,z)
            print("Joints Setpoint-",joint_set)            
           
           #Joint-1 and Joint-2 controllers
            joint1_set_point=joint_set.theta1
            joint1_controller=Controller(P=1,I=0.00,D=275,set_point=joint1_set_point,j_t=0)
            joint2_set_point=joint_set.theta2
            joint2_controller=Controller(P=1,I=0.00,D=245,set_point=joint2_set_point,j_t=0)
            threshold = 0 
            while True:
                               
                if abs(joint1_set_point-self.theta1) < 0.15 and abs(joint2_set_point-self.theta2) < 0.15:
                    threshold = threshold + 1
                    if threshold > 40:
                        break
                self.jt_tq_1=joint1_controller.update(self.theta1)
                #Setting limit to Controller amount- to avoid unstability
                if self.jt_tq_1>0:
                	self.jt_tq_1=min(self.jt_tq_1,1.5)
                else:
                	self.jt_tq_1=max(self.jt_tq_1,-1.5)
                 #joint-2 
                self.jt_tq_2=joint2_controller.update(self.theta2)
                if self.jt_tq_2>0:
                	self.jt_tq_2=min(self.jt_tq_2,1.5)
                else:
                	self.jt_tq_2=max(self.jt_tq_2,-1.5)
                
                self.theta1_pub.publish(self.jt_tq_1)
                self.theta2_pub.publish(self.jt_tq_2)
                rospy.sleep(0.01)
                
            self.jt_tq_1=0
            self.theta1_pub.publish(self.jt_tq_1)
            rospy.loginfo("Joint 1 setpoint reached")
            print("Joint angle -1 = ",self.theta1)
            self.jt_tq_2=0
            self.theta2_pub.publish(self.jt_tq_2)
            rospy.loginfo("Joint 2 setpoint reached")
            print("Joint angle -2 = ",self.theta2)
        
            #JOINT3
            joint3_set_point=joint_set.d3
            
            joint3_controller=Controller(P=100,I=10,D=50,set_point=joint3_set_point, j_t=1)
    
            while(abs(joint3_set_point-self.d3)>0.002):

                self.jt_F=joint3_controller.update(self.d3)
                if self.jt_F>0:
                	self.jt_F=min(self.jt_F,25)
                else:
                	self.jt_F=max(self.jt_F,-25)
        
                self.d_3_pub.publish(self.jt_F)
                rospy.sleep(0.01)
            self.jt_F=0
            self.d_3_pub.publish(self.jt_F)
            rospy.loginfo("Joint 3 setpoint reached")
            print("Joint position -3 = ",self.d3)
            rospy.sleep(3)
        
        
    def calculateik(self, x,y,z):
        rospy.wait_for_service("ik_solver")
        try:
            inverse=rospy.ServiceProxy("ik_solver",ik)
            out=inverse(x,y,z)
            return out
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        
    def joint_states_callback(self,data):
        # get joint_state = (angle1, angle2,distance) 
        self.theta1 = data.position[0]
        self.theta2 = data.position[1]
        self.d3 = data.position[2]


if __name__ == "__main__":
    whatever = RRP()


		
	
	
		
