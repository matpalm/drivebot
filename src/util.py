import numpy as np
import random

def raised(values, p):
    # shift min value up to 1e-3 if there are any negative
    # use some epsilon rather than 0 other v**p will be 0 even for small p (eg during explore)
    m = min(values)
    if m < 0.0:
        values = [v -m + 1e-3 for v in values]
    # raise all
    return [v**p for v in values]

def normalised(values):
    s = float(sum(values))
    return [v/s for v in values]

def weighted_choice(values):
    # assumes values normalised
    c = random.random()
    i = 0
    while c > values[i]:
        c -= values[i]
        i += 1
    return i
