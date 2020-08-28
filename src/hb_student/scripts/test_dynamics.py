#!/usr/bin/env python3

import rospy
import std_msgs.msg
import hb_msgs.msg
import numpy as np

from hb_common.datatypes import Params, Command, ROSParameterException
from hb_common.helpers import saturate
from dynamics import Dynamics

# this is just the dynamic parameters from the ROS param server
params = Params._from_rosparam()

#this is artifically setting all our states and inputs to 0.5 to test our dynamics function
state = np.ones((6,1))*0.5
command = Command(left=0.5, right=0.5)

#making a dynamics object
d = Dynamics()

deriv_of_state = d.dynamics(params, state, command)


print("\n\n\n\nThis is the derivative of our state!!!")
print(deriv_of_state)
print("This is the derivative of our state!!!\n\n\n\n")




## for this input:

#state = np.ones((6,1))*0.5
#command = Command(left=0.5, right=0.5)



## I got this output:

#This is the derivative of our state!!!
#[[  0.5       ]
# [  0.5       ]
# [  0.5       ]
# [  6.96828413]
# [ 18.8517092 ]
# [ 14.16995885]]
#This is the derivative of our state!!!
