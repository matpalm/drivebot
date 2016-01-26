#!/usr/bin/env python
import sys, math
import numpy as np
import random

# anneal v between min and max over n steps
# for 0 < i < n return linear interpolation between min_v and max_b
# for i > n return max_v

def annealed_value(min_v, max_v, i, n):
    return min(max_v, min_v + (max_v-min_v)*(i*1./n))

def weighted_choice(values):  
    # assume values normalised
    c = random.random()
    i = 0
    while c > values[i]:
        c -= values[i]
        i += 1
    return i


p = 0.1
for i in range(100):
    x = [ 18.46289668, 18.14081887, 1.1899614 ]
    x_min = [max(v, 0.1) for v in x]
    x_pow = [math.pow(v, p) for v in x_min]
    x_pow_sum = sum(x_pow)
    x_pow_norm = [v / x_pow_sum for v in x_pow]
    print "\t".join(map(str, [i, p, x_pow_norm]))
    if p < 5: p *= 1.1

from collections import Counter
f = Counter()
for i in range(1000):
    f[weighted_choice([0.3,0.2,0.5])] += 1
print f
