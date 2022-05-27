import numpy as np
import matplotlib.pyplot as plt

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

def gen_inf():
    t= np.linspace(0,np.pi*2,150)
    x= np.sin(t)
    y=np.sin(t)*np.cos(t)
    curve=plt.plot(x,y)

    x_data= curve[0].get_xdata()    
    y_data= curve[0].get_ydata()
    
    return (x_data,y_data)

x_data,y_data= circular_traj(10)
#x_data,y_data= gen_inf()
data= np.c_[x_data,y_data]
np.save("traj.npy",data)
np.savetxt("circle",data)
print(data)
#x_data= np.save('x_data',data[0])
#y_data= np.save
plt.show()