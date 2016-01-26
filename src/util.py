import random

def normalised(values):
    s = float(sum(values))
    return [v/s for v in values]

def weighted_choice(values):
    # assume values normalised                                                  
    c = random.random()
    i = 0
    while c > values[i]:
        c -= values[i]
        i += 1
    return i
