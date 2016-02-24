# simple wrapper for sonars 0,1,2 (forward, left & right) of an stdr pandora_robot

import rospy
from sensor_msgs.msg import Range
import numpy as np
import re
import time
import sys

class Sonars(object):

    def __init__(self, robot_id, num_sonars=3):
        self.robot_id = robot_id
        self.num_sonars = num_sonars
        self.reset()
        for idx in range(0, num_sonars):
            # sonar 0, 1 & 2 => forward, left & right
            rospy.Subscriber("/robot%s/sonar_%s" % (self.robot_id, idx), Range, self.sonar_callback)

    def reset(self):
        # dft noop value for case of sonars not publishing yet
        self.ranges = [0] * self.num_sonars

    def sonar_callback(self, msg):
        # record callback from one sonar into ranges []
        m = re.match(".*sonar_(.*)", msg.header.frame_id)
        idx = int(m.group(1))  # or die in a regex non match hellfire
        r = msg.range
        if r > msg.max_range: r = msg.max_range
        if r < msg.min_range: r = 0
        self.ranges[idx] = int(r * 100)  # for ease of reading

    
    
            

        
