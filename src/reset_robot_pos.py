# various utils for dealing with reseting robots to a random position

import rospy
from geometry_msgs.msg import Pose2D
from stdr_msgs.srv import MoveRobot
import random
import math

class BotPosition(object):
    def __init__(self, robot_id):
        srv_str = "/robot%s/replace" % robot_id
        rospy.wait_for_service(srv_str)
        self.move = rospy.ServiceProxy(srv_str, MoveRobot)
        # TODO: handle rospy.service.ServiceException which can thrown from this
        self.starting_random_positions = None
        self.straight_section_poses = None

    # reset bot to a random position 
    def reset_robot_random_pose(self):
        if self.starting_random_positions is None:
            self.starting_random_positions = []
            # top straight
            for x in range(1, 10):
               self.starting_random_positions.append((x, 9))
            # rhs straight
            for y in range(1, 9):  
                self.starting_random_positions.append((9, y))
            # lhs zip / zag, top to bottom
            for y in range(5, 9):
                self.starting_random_positions.append((1, y))
            for x in range(2, 5):
                self.starting_random_positions.append((x, 5))
            for y in range(1, 5):
                self.starting_random_positions.append((5, y))
            for x in range(6, 9):
                self.starting_random_positions.append((x, 1))
            # check no dups
            assert len(self.starting_random_positions) == len(set(self.starting_random_positions)),\
                ("%s" % self.starting_random_positions)

        # pick a random starting pose
        start_x, start_y = random.choice(self.starting_random_positions)
        new_pose = Pose2D()
        new_pose.x = start_x
        new_pose.y = start_y
        new_pose.theta = random.random() * 2 * math.pi
        self.move(new_pose)

    def reset_robot_on_straight_section(self):
        if self.straight_section_poses is None:
            self.straight_section_poses = [(3,9,0), (7,9,0), (9,7,4.71),
                                           (9,3,4.71), (7,1,3.14), (5,3,1.57),
                                           (3,5,3.14), (1,7,1.57)]
        start_x, start_y, start_theta = random.choice(self.straight_section_poses)
        new_pose = Pose2D()
        new_pose.x = start_x
        new_pose.y = start_y
        new_pose.theta = start_theta
        self.move(new_pose)


