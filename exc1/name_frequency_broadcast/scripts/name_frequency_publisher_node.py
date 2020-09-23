#!/usr/bin/env python

import rospy # import rospy as that is what we'll be working with
from std_msgs.msg import Int32 #import the message type. To minimize the size I'm satisfied with 32 bit as it will let me send an int up to 2.1M
from name_frequency_broadcast.msg import Incremented


def incrementor():
    pub = rospy.Publisher('kjellberg', Int32, queue_size=10)
    rospy.init_node('name_frequency_boradcast_node', anonymous=True) # Anonymous ensures us that the node name is unique.
    rate = rospy.Rate(20) # The instr says that we should be sending at 20 Hz.
    #data = Incremented()
    set_of_ints = []
    interator = 0
    k =1 
    incrementor = 4
    while not rospy.is_shutdown():
        value_to_publish = k+(interator*incrementor)
        #data.incrementor_value = incrementor
        #data.multiplier = index_of_int
        #rospy.loginfo(data) # We might just as well log everything so that it's in core for debugging.
        interator+=1
        pub.publish(value_to_publish)
        rate.sleep() # Force the loop to sleep with the appropriate time so that we are operating at 20 Hz.

if __name__ == '__main__':
    try:
        incrementor()
    except rospy.ROSInterruptException:
        pass


