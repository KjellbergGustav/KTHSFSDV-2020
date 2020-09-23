#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int32
from name_frequency_broadcast.msg import Incremented

def callback(data):
    pub = rospy.Publisher('/kthfs/result', Float64, queue_size=10)
    #rospy.loginfo(rospy.get_caller_id() + "Incrmentor valus is %d and muliplier is %d", data.incrementor_value, data.multiplier)
    rate = rospy.Rate(20)
    #incrementor_value = data.incrementor_value
    #multiplier = data.multiplier
    q = 1/0.15
    result = data.data*q
    rospy.loginfo(result)
    pub.publish(result)
    rate.sleep()

 
def received_number():
    rospy.init_node('name_frequency_receiver_node', anonymous=True) #Here we init the node before we subscribe.
    rospy.Subscriber("kjellberg", Int32, callback)
    rospy.spin() # Spin keeps us from exiting without ctlr-c prombt

if __name__ == '__main__':
    received_number()
    
