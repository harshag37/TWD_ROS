#!/usr/bin/env python2.7
from cmath import sqrt
from logging import shutdown
from re import M
from traceback import print_tb
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from math import pi
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
#from tf.transformations import euler_from_quaternion

i=0
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

    def ctrl(self,x_d,y_d,x,y,theta_a,v):

        e_x= x - x_d 
        e_y= y - y_d
        d_str= -1
        e= np.sqrt(e_x*e_x + e_y*e_y) - d_str

        v_str= self.Kp*e + self.Ki*(e+sum(self.e_q))   # linear velocity

        theta_d= np.arctan2(e_y,e_x)
        e_theta= theta_a - theta_d

        yaw= self.Kh*e_theta    # yaw angle

        e_v= v - v_str
        Th= self.Kpv*e_v - self.Kiv*(e_v+sum(self.t_q))   #Throttle 
        
        #self.t_last = t_now
        #self.last_error = e
        if len(self.e_q) > 10:
            self.e_q.popleft()
        if len(self.t_q)>10:
            self.t_q.popleft()
        self.t_q.append(e_v)
        self.e_q.append(e)
        
        #self.v=v_str
        #theta_a=yaw
        return(v_str,yaw,Th)
odom=ModelStates()
yaw=Float64(0.0)
v_ctrl= PI(Kp=0.35,Ki=0.06,Kh=0.0851 ,Kpv=0.1,Kiv=0.1)
traj= np.load("traj.npy")
def callback(data):
    global odom
    odom=data
def callback2(data):
    global yaw
    yaw=data
vel=Float64(1)
def run():
    global i,vel,odom,yaw
    global traj
    rospy.Subscriber("/gazebo/model_states",ModelStates, callback)
    rospy.Subscriber('/e_rick/fork_position_controller/command',Float64,callback2)
    pub1 = rospy.Publisher('/cmd_vel',Float64, queue_size=10)
    pub2 = rospy.Publisher('/e_rick/fork_position_controller/command',Float64, queue_size=10)
    rate=rospy.Rate(1)
    while not rospy.is_shutdown() and i<150:
        data= v_ctrl.ctrl(x_d=traj[i-1][0],y_d=traj[i-1][1],x=odom.pose[1].position.x,y=odom.pose[1].position.y,theta_a=yaw.data,v=vel.data)
        velocity=Float64(data[0])
        pub1.publish(velocity)
        pub2.publish(data[1])
        #print(yaw)
        #print(data[1])
        now = rospy.get_rostime()
         
    #print(type(now))
        plt.plot(yaw.data,i,"^")
        if i%20==0:
            plt.show()
        
        i+=1
        vel=velocity
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous=True)
        run()
    except rospy.ROSInterruptException:
        pass