#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
vel1=Float64()
vel2=Float64()

def callback(data):
    global vel1
    global vel2 
    vel1=data
    vel2.data=-data.data
# def callback2(data):
#     print(data.pose[1].position)
    
def vel_pub():
    global vel1
    global vel2
    rospy.Subscriber("/cmd_vel",Float64, callback)
    # rospy.Subscriber("/gazebo/model_states",ModelStates, callback2)
    rate=rospy.Rate(10)
    pub1 = rospy.Publisher('/e_rick/left_velocity_controller/command',Float64, queue_size=10)
    pub2 = rospy.Publisher('/e_rick/right_velocity_controller/command',Float64, queue_size=10)
    while not rospy.is_shutdown():
        pub1.publish(vel1)
        pub2.publish(vel2)
        rate.sleep()
    rospy.spin()
if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous=True)
        vel_pub()
    except rospy.ROSInterruptException:
        pass