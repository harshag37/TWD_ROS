#!/usr/bin/env python2.7
from cmath import sqrt
from inspect import trace
from logging import shutdown
from traceback import print_tb
from xmlrpc.client import MAXINT, MININT
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from math import pi
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from time import sleep
from datetime import datetime
#from tf.transformations import euler_from_quaternion
#Controller
class PI():
    def __init__(self, Kp, Ki,Kh,Kpv,Kiv):  # Kp,Ki,Kh are tuning parameters for vel and yaw cmd
        #self.t_last = t_now                # Kpv,Kih are tuning parameters for Throttle cmd
        self.Kp= Kp
        self.Ki= Ki
        self.Kh= Kh
        self.Kpv= Kpv
        self.Kiv= Kiv
        #self.last_error=0
        #self.x=1
        #self.y=0
        #self.theta=pi/2
        #self.v=1
        self.e_q=deque()
        self.t_q=deque()
        self.velo=2
        self.ang_vel=0.5
    def ctrl2(self,d,theta_a):
        e_d=self.Kp*d
        theta_a=e_d/100
        v=self.velo
        if theta_a>1.57:
            theta_a=1.57
        elif theta_a<-1.57:
            theta_a=-1.57
        
        print("theta ",theta_a)
        return (theta_a,v)

    def ctrl(self,x_d,y_d,x,y,theta_a):
        print("xd: ",x_d," yd: ",y_d," x: ",x," y: ",y)
        e_x= x - x_d 
        e_y= y - y_d
        d_str= 1
        e= np.sqrt(e_x*e_x + e_y*e_y) - d_str

        v_str=self.Kp*e #+ self.Ki*(e+sum(self.e_q))   # linear velocity

        theta_d= np.arctan2(e_y,e_x)
        e_theta= theta_a - theta_d

        yaw= self.Kh*e_theta    # yaw angle

        # e_v= v - v_str
        # Th= self.Kpv*e_v - self.Kiv*(e_v+sum(self.t_q))   #Throttle 
        
        #self.t_last = t_now
        #self.last_error = e
        if len(self.e_q) > 10:
            self.e_q.popleft()
        if len(self.t_q)>10:
            self.t_q.popleft()
        # self.t_q.append(e_v)
        self.e_q.append(e)
        print("v: ",v_str," yaw ",yaw)
        #self.v=v_str
        #theta_a=yaw
        return(v_str,yaw)
class Control():
    def __init__(self):
        print("yes")
        self.yaw=Float64(0.0)
        self.i=0
        self.j=0
        self.vel=0
        self.v_ctrl= PI(Kp=1,Ki=0.06,Kh=0.3 ,Kpv=0.1,Kiv=0.1)
        self.traj= np.load("traj.npy")
        rospy.Subscriber('/e_rick/fork_position_controller/command',Float64,self.callback2)
        rospy.Subscriber("/gazebo/model_states",ModelStates, self.callback)
        self.pub1 = rospy.Publisher('/cmd_vel',Float64, queue_size=10)
        self.pub2 = rospy.Publisher('/e_rick/fork_position_controller/command',Float64, queue_size=10)
        
    def callback2(self,data):
        self.yaw=data
    def callback(self,data):
        odom=data
        # d=MAXINT
        # x=odom.pose[1].position.x
        # y=odom.pose[1].position.y
        # for l,m in self.traj:
        #     if d>np.sqrt((l-x)**2+(m-y)**2):
        #         d=np.sqrt((l-x)**2+(m-y)**2)
        # d_c=np.sqrt(x**2+y**2)
        # if d_c<10:
        #     d=-d
        # print(d)
        x_d,y_d=self.traj[self.i]
        x=odom.pose[1].position.x
        y=odom.pose[1].position.y
        data=self.v_ctrl.ctrl(x_d=x_d,y_d=y_d,x=x,y=y,theta_a=self.yaw.data)
        velocity=Float64(data[0])
        self.pub1.publish(velocity)
        angl=Float64(data[1])
        self.pub2.publish(angl)
        if(self.j==1000):
            self.j=0
            if self.i==149:
                self.i=0
            self.i+=1
        self.j+=1
        


if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous=True)
        control=Control()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass