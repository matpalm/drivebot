#!/usr/bin/env python
import json
import sys
for line in sys.stdin:
    if line.startswith("EPISODE"):
        line = line.replace("EPISODE\t","")
        d = json.loads(line)
        print len(d)

#    rewards = [r['reward'] for r in d]
#    print sum(rewards), "\t", rewards

