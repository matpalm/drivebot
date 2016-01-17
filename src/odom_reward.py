# reward progress around track by breaking it into larg(ish) grid squares
# and giving +1 if we progress to the next square & -5 if we back a square.
# the baseline track was designed to "fit" into simple coarse squares to
# make this reward calc simpler.

# HARDCODED TO TRACK1

import rospy
from nav_msgs.msg import Odometry
import sys

class OdomReward(object):

    # clockwise around track1 
    GRID_ORDER = [22, 23, 24, 19, 14, 9, 4, 3, 2, 7, 12, 11, 10, 15, 20, 21]
    LEN = len(GRID_ORDER)

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.last_grid_pt = None  # grid point at last reward() call
        self.latest_grid_pt = None  # constantly updated from callback
        rospy.Subscriber("/robot%s/odom" % self.robot_id, Odometry, self.odom_callback)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        gx, gy = int(pos.x / 2), int(pos.y / 2)  # map ref -> grid ref is scale of 2
        self.latest_grid_pt = gx + 5 * gy  # bottom left (0, 0) -> 0 & top right (4, 4) -> 24

    def reward(self):
        if self.latest_grid_pt == None:
            # no callbacks yet
            return 0

        if self.last_grid_pt == None:
            # just started
            self.last_grid_pt = self.latest_grid_pt
            return 0

        latest = self.latest_grid_pt
        latest_idx = OdomReward.GRID_ORDER.index(latest)
        last_idx = OdomReward.GRID_ORDER.index(self.last_grid_pt)

        r = None
        wrap = len(OdomReward.GRID_ORDER)
        if latest_idx == last_idx:
            r = 0
        elif latest_idx == (last_idx + 1) % wrap:
            r = 1
        elif latest_idx == (last_idx - 1) % wrap:
            r = -5
        else:
            # robot has been reset?
            print >>sys.stderr, "odom reward; reset?"
            self.last_grid_pt = None
            return 0
    
        self.last_grid_pt = latest
        return r
    
