#!/usr/bin/env python
# given a stream of episodes (1/line); extract events, shuffle and emit them (1/line)
import sys, json, random
for line in sys.stdin:
    episode = json.loads(line)
    for event in episode:
        print json.dumps(event)

