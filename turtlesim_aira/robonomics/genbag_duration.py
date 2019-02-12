# -*- coding: utf-8 -*-

import rospy
import rosbag
from std_msgs.msg import String, Duration


bag = rosbag.Bag('./objective.bag', 'w')
msg = Duration(data=rospy.Duration(24*60*60))

try:
    bag.write('/duration', msg)
finally:
    bag.close()
