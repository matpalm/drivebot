#!/usr/bin/env python
import json
from geometry_msgs.msg import Pose2D
import math
import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import Range
from stdr_msgs.srv import MoveRobot
import sys
import time
from collections import Counter

robot_id = 0
srv_str = "/robot%s/replace" % robot_id
rospy.wait_for_service(srv_str)
move = rospy.ServiceProxy(srv_str, MoveRobot)

def range_at(x, y, theta):
    pose = Pose2D()
    pose.x = x
    pose.y = y
    pose.theta = theta
    global r
    r = None
    move(pose)  # will throw rospy.service.ServiceException on attempting to hit a wall.
    attempts = 0
    while r is None:
        time.sleep(0.1)
        attempts += 1
        if attempts > 10:
            print >>sys.stderr, "lock up on range?"
            exit(0)
    return r

r = None
def sonar_callback(msg):
    rr = msg.range
    if rr > msg.max_range: rr = msg.max_range
    if rr < msg.min_range: rr = 0
    global r
    r = rr
rospy.Subscriber("/robot%s/sonar_0" % robot_id, Range, sonar_callback)
rospy.init_node('vector_field_data')

def on_track(x, y):
    if x == 10 or y == 10:
        return False  # some weird crash causing corner case?
    if x <= 4 and y <= 4:
        return False
    if x >= 6 and x <= 8 and y >= 2 and y <= 8:
        return False
    if x >= 2 and x <= 6 and y >= 6 and y <= 8:
        return False
    return True

num_arcs = 10
Y, X = np.mgrid[1:10:50j,1:10:50j]

def log(x, y, theta=0, srange=0, msg="reading"):
    print "\t".join(map(str, ["LOG", x, y, theta, srange, msg]))

for i in range(0, X.shape[0]):
    for j in range(0, X.shape[1]):
        x = X[i][j]
        y = Y[i][j]
        if not on_track(x, y):
            log(x=x, y=y, msg="off track")
            continue
        try:
            max_range = max_theta = None
            for theta in np.arange(0.0, math.pi*2, math.pi*2/num_arcs):
                forward_range = range_at(x, y, theta)
                log(x=x, y=y, theta=theta, srange=forward_range)
                if forward_range >= max_range:
                    max_range, max_theta = forward_range, theta
            print "MAX\t%s" % json.dumps({"x": x, "y": y, "theta": max_theta, "srange": max_range, "i": i, "j": j})
        except rospy.service.ServiceException:
            # tried to move into a wall. regardless of theta that invalidates
            # this pose
            log(x=x, y=y, msg="wall?")
