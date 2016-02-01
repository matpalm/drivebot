#!/usr/bin/env python
import json
import sys

episode_id = 0 
for line in sys.stdin:
    episode = json.loads(line)
    rewards = [event['reward'] for event in episode]
    print "\t".join(map(str, [episode_id, len(episode), sum(rewards)]))
    episode_id += 1
