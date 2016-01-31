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
parser.add_argument('--delay', type=float, default=0.0,
                    help="delay (ms) between publishing messages")
opts = parser.parse_args()
delay_ms = 0 if opts.delay <= 0 else opts.delay / 1000

training = rospy.Publisher(opts.training_eg_topic, TrainingExample, queue_size=200)

rospy.init_node('publish_events_to_topic')

for line in sys.stdin:
    event = json.loads(line)
    training.publish(u.training_eg_msg(event['state_1'], event['action'], event['reward'], event['state_2']))
    if delay_ms > 0:
        time.sleep(delay_ms)
                     
    
