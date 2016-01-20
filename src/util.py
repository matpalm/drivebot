# various utils for dealing with robots

import rospy
from geometry_msgs.msg import Pose2D
from stdr_msgs.srv import MoveRobot
import random

starting_poses = [(3,9,0), (7,9,0), (9,7,4.71),
                  (9,3,4.71), (7,1,3.14), (5,3,1.57),
                  (3,5,3.14), (1,7,1.57)]

def reset_robot(robot_id):
    srv = "/robot%s/replace" % robot_id
    rospy.wait_for_service('/robot0/replace')
    move = rospy.ServiceProxy('/robot0/replace', MoveRobot)
    start_x, start_y, start_theta = random.choice(starting_poses)
    new_pose = Pose2D()
    new_pose.x = start_x
    new_pose.y = start_y
    new_pose.theta = start_theta
    move(new_pose)


