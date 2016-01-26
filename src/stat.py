#!/usr/bin/env python
import json
import sys

episode_id = 0 
for line in sys.stdin:
    if line.startswith("EPISODE"):
        line = line.replace("EPISODE\t","")
        episode = json.loads(line)
        rewards = [event['r'] for event in episode]
        print "\t".join(map(str, [episode_id, len(episode), sum(rewards)]))
        episode_id += 1
