from tkinter import Y
import numpy as np
from math import cos, sin, tan, log


class model():
    def __init__(self,x,y,theta,L,v=0):
        self.L=L
        self.theta= theta
        self.x=x
        self.y=y
        self.v=v
        
    
    def kine(self,vel,yaw,Th):
        
        dt=0.1
        K=1
        a= K*Th
        
        x= self.x -vel*cos(self.theta)*dt
        y= self.y-vel*sin(self.theta)*dt
        theta_o= self.theta - (vel*tan(yaw))/self.L*dt

        """x=  vel*sin(self.theta)
        y=  -vel*cos(self.theta)
        theta_o=  (vel*(-log(abs(cos(yaw)))))/self.L"""

        self.x=x
        self.y=y
        self.theta=theta_o
        self.v= vel + a*dt
        

        #self.theta=theta_o 

        return (x,y,theta_o)
