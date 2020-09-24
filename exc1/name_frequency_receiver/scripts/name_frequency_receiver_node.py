#!/usr/bin/env python

import rospy
from name_frequency_broadcast.msg import Incremented
from std_msgs.msg import Float64, Int32


def callback(data):
    """[summary]
        The callback function for the receiver. It tranforms the data sent by the publisher and then publishes the received 
        data to the topic /kthfs/result.
    Args:
        data ([Int32]): [The received data]
    """    
    pub = rospy.Publisher('/kthfs/result', Float64, queue_size=10)
    #rospy.loginfo(rospy.get_caller_id() + "Incrmentor valus is %d and muliplier is %d", data.incrementor_value, data.multiplier)
    rate = rospy.Rate(20) # Frequency 20 Hz
    #incrementor_value = data.incrementor_value # Keeping these lines for discussion at interview.
    #multiplier = data.multiplier
    q = 1/0.15
    result = data.data*q
    rospy.loginfo(result)
    pub.publish(result)
    rate.sleep()

 
def received_number():
    """[summary]
        This function subscribers to the topic kjellberg and reveives the data.
    """
    rospy.init_node('name_frequency_receiver_node', anonymous=True) #Here we init the node before we subscribe.
    rospy.Subscriber("kjellberg", Int32, callback)
    rospy.spin() # Spin keeps us from exiting without ctlr-c prombt

if __name__ == '__main__':
    received_number()
    
