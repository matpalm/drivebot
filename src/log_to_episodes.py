#!/usr/bin/env python
import sys, json
for line in sys.stdin:
    if line.startswith("EPISODE"):
        tag, jsonl = line.split("\t")
        assert tag == "EPISODE"
        episode = json.loads(jsonl)
        for event in episode:
            if 't' in event:
                del event['t']
        print json.dumps(episode)

