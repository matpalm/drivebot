#!/usr/bin/env python
import argparse
import datetime
import rospy
import sys
import time

# 0 < n < burn-in => min-value
# burn-in < n < burn-in+ramp-up => value annealed from min to max
# burn-in+ramp-up < n => max-value
parser = argparse.ArgumentParser()
parser.add_argument('--param', type=str, help="rospy param to set")
parser.add_argument('--burn-in', type=float, default=0, help="burn in (sec) to keep at min-value")
parser.add_argument('--ramp-up', type=float, help="ramp-up time (sec) after burn-in to move from min to max")
parser.add_argument('--step', type=float, help="granularity of steps (sec). i.e. emit each step sec")
parser.add_argument('--start-value', type=float, help="start value to emit <= burn-in sec")
parser.add_argument('--end-value', type=float, help="final value to emit >= burn-in + ramp-up sec")
o = parser.parse_args()
print "OPTS", o
assert o.burn_in >= 0
assert o.ramp_up > 0
assert o.step > 0

def emit(n, v):
    print datetime.datetime.now().strftime('%c'), v
    rospy.set_param(o.param, v)

n = 0.0
while n < o.burn_in + o.ramp_up + o.step:
    if n <= o.burn_in:
        emit(n, o.start_value)
    elif n >= o.burn_in + o.ramp_up:
        emit(n, o.end_value)
    else:
        normalised = float(n - o.burn_in) / o.ramp_up
        rescaled = (normalised * (o.end_value - o.start_value)) + o.start_value
        emit(n, rescaled)
    n += 1
    # assume everything else is instant (including set_param call)
    time.sleep(o.step)
