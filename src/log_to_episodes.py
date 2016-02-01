#!/usr/bin/env python
import sys, json
num_decode_errors = 0

for line in sys.stdin:
    if not line.startswith("EPISODE"):
        continue
    if line.startswith("EPISODE STAT"):
        continue

    try:
        tag, jsonl = line.split("\t")
        assert tag == "EPISODE"
        episode = json.loads(jsonl)
        for event in episode:
            if 't' in event:
                del event['t']
        print json.dumps(episode)
    except:
        num_decode_errors += 1

print >>sys.stderr, "num_decode_errors", num_decode_errors

