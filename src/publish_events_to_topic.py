#!/usr/bin/env python
import argparse
from drivebot.msg import TrainingExample
import json
import rospy
import sys
import time
import util as u

parser = argparse.ArgumentParser()
parser.add_argument('--training-eg-topic', default="/drivebot/training_egs", 
                    help="ros topic to publish TrainingExamples to")
parser.add_argument('--rate', type=float, default=0.0,
                    help="rate (hz) to publish. 0 => fast as possible")
opts = parser.parse_args()

training = rospy.Publisher(opts.training_eg_topic, TrainingExample, queue_size=200)
rospy.init_node('publish_events_to_topic')
rate = None if opts.rate <= 0 else rospy.Rate(opts.rate)

for line in sys.stdin:
    if rospy.is_shutdown():
        break
    event = json.loads(line)
    eg = u.training_eg_msg(event['state_1'], event['action'], event['reward'], event['state_2'])
    training.publish(eg)
    if rate is not None:
        rate.sleep()
                     
    
