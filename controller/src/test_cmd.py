from cmath import sqrt
import numpy as np
import matplotlib.pyplot as plt
import PI_ctrl
import sys_model

# Reference trajectory generation (Circle)
def circular_traj(radius):
    theta= np.linspace(0,2*np.pi,150)
    r= radius

    a= r*np.cos(theta)
    b= r*np.sin(theta)

    figure,axes= plt.subplots(1)
    curve=axes.plot(a,b)
    axes.set_aspect(1)

    #Extract x,y data from plot
    x_data= curve[0].get_xdata()    
    y_data= curve[0].get_ydata()
    #print(np.shape(y_data))

    return (x_data,y_data)

#plt.plot(a,b)
#plt.show()

def test_out(radius,Kp,Ki,Kh,Kpv,Kiv):     # Function to test output, takes in radius of traj and tuning parameters
    traj_x,traj_y = circular_traj(radius)  # generate x,y coordinates for a circle
    v_ctrlr= PI_ctrl.PI(Kp,Ki,Kh,Kpv,Kiv)  # Initialise controller

    for i in range(len(traj_x)):
        res= np.empty((150,3))
        res[i][0],res[i][1],res[i][2]= v_ctrlr.ctrl(traj_x[i],traj_y[i])

    return res                             # Returns vel,yaw and throttle cmds

#print(test_out(1,0.2,3,0.1,0.2,0.1))

car= sys_model.model(0,0.1)
g= test_out(1,0.2,3,0.1,0.2,0.1)
for i in range(150):
    res= np.empty((150,3))
    res[i][0],res[i][1],res[i][2]= car.kine(g[i][0],g[i][1],g[i][2])
print(res)