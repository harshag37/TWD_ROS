from time import time
import numpy as np
import PI_ctrl
import sys_model
#import test_cmd
import matplotlib.pyplot as plt
from math import pi

dt=0.1
time_d=np.arange(0,15,0.1)

class sim():
    def __init__(self,radius,Kp,Ki,Kh,Kpv,Kiv):
        self.radius=radius
        self.Kp=Kp
        self.Ki=Ki
        self.Kh=Kh
        self.Kpv=Kpv
        self.Kiv=Kiv

    def run(self):
        T= 15
        curr_time=0

        car= sys_model.model(10,0,pi/2,0.1)
        traj= np.load("traj.npy")       # (x,y)
        traj= np.column_stack((traj,time_d))
        v_ctrl= PI_ctrl.PI(self.Kp,self.Ki,self.Kh,self.Kpv,self.Kiv) # v,y,Th
        x,y,t= [10],[0],[pi/2]
        v,yaw,th= [0],[0],[0]
        c_time=[0]
        #xyt=np.empty((150,3))
        #vyth= np.empty((150,3))
        
        """
        xyt[0][0],xyt[0][1],xyt[0][2]= 10,0,pi/2
        vyth[0][0],vyth[0][1],vyth[0][2]= 0,0,0"""

        """
        for i in range(1,len(traj)):
            #print("x = ",xyt[i-1][0])
            #print("xtraj = ",traj[i][0])
            #print("y = ",xyt[i-1][1])
            #print("ytraj = ",traj[i][1])
            vyth[i][0],vyth[i][1],vyth[i][2]= v_ctrl.ctrl(traj[i-1][0],traj[i-1][1],xyt[i-1][0],xyt[i-1][1],xyt[i-1][2],vyth[i-1][0])
            xyt[i][0],xyt[i][1],xyt[i][2]=  car.kine(vyth[i-1][0],vyth[i-1][1],vyth[i-1][1]) """
        i=1
        while T > curr_time:
            
            

            vyth= v_ctrl.ctrl(traj[i-1][0],traj[i-1][1],x[i-1],y[i-1],t[i-1],v[i-1])
            xyt=  car.kine(v[i-1],yaw[i-1],th[i-1])
            
            if i <150:
                i+=1
            curr_time= round(curr_time,1)+ dt

            x.append(xyt[0])
            y.append(xyt[1])
            t.append(xyt[2])
            v.append(vyth[0])
            yaw.append(vyth[1])
            th.append(vyth[2])
            c_time.append(curr_time)
            #print(c_time)
        return x,y,t,c_time

car1=sim(1,0.35,0.06,0.0851 ,0.1,0.1)
#car1=sim(1,Kp=0.24,Ki=0.0025,Kh=0.499 ,Kpv=0.1,Kiv=0.1)   #car1=sim(1,0.35,0.06,0.0851 ,0.1,0.1) for circle
#print(car1.run())


traj= np.load("traj.npy")
#trip= np.column_stack((traj,time_d))
#print(trip)
x_d=np.hsplit(traj,2)[0]
y_d=np.hsplit(traj,2)[1]
#t_d=np.hsplit(traj,2)[2]
out=car1.run()
#tt= np.arange(0,15.2,0.1)

x,y,theta,tt= out
#x=np.hsplit(out,3)[0]
#y=np.hsplit(out,3)[1]
#theta=np.hsplit(out,3)[2]
# print(len(time_d))

#y_d =np.load("traj.npy")
#print(theta)
#plt.subplot(1,2,1)
plt.plot(x_d,y_d,"-r",label="input")
plt.plot(x,y,"--",label="output")
#plt.subplot(1,2,2)
#plt.plot(tt,theta,label="yaw")
plt.legend()

#plt.plot()
plt.show()

