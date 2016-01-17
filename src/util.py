# various utils for dealing with robots

import rospy
from geometry_msgs.msg import Pose2D
from stdr_msgs.srv import MoveRobot

def reset_robot(robot_id):
    srv = "/robot%s/replace" % robot_id
    rospy.wait_for_service('/robot0/replace')
    move = rospy.ServiceProxy('/robot0/replace', MoveRobot)
    new_pose = Pose2D()
    new_pose.x = 3
    new_pose.y = 9
    new_pose.theta = 0
    move(new_pose)


