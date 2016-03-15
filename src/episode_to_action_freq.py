#!/usr/bin/env python                                                                                                    
from collections import defaultdict, Counter
import json
import sys

BUCKET_WIDTH = 50 if len(sys.argv)==1 else int(sys.argv[1])

def action_factor(e):
    return {0: "forward", 1: "left", 2: "right", 3:"back"}[e['action']]

# {bucket: {action: action_freq, ...}                                                                                    
counts = defaultdict(Counter)
for n, episode_json in enumerate(sys.stdin):
    for event in json.loads(episode_json):
        # set bucket to be upper bound of bucket range
        bucket = ((n / BUCKET_WIDTH)+1) * BUCKET_WIDTH
        counts[bucket][action_factor(event)] += 1

print "\t".join(['bucket', 'action', 'freq'])
for bucket, action_freqs in counts.iteritems():
    for action, freq in action_freqs.iteritems():
        print "\t".join(map(str, [bucket, action, freq]))
