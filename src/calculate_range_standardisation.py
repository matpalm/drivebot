#!/usr/bin/env python
import sys, json, math

# given a stream of EPISODE log data fit a mean/std dev across all three
# readings. note: could fit one per F, L, R but after initial eyeball it's
# not a huge thing.
def fit_mean_sd_from_stream(stream):
    # streaming mean calc
    total = 0.0
    # streaming std dev calc
    m = 0.0
    s = 0.0
    k = 1

    for line in stream:
        episode = json.loads(line)
        for e in episode:
            ranges = e['ranges_1']  # ignore ranges_2; it'll just be ranges_1 from previous event in episode...
            for r in ranges:
                r = float(r)
                total += r
                m_ = m
                m += (r-m_) / k
                s += (r-m_) * (r-m)
                k += 1

    mean = total / (k-1)
    std = math.sqrt(s/(k-2))
    return (mean, std)

def standardise_ranges(ranges, mean, std):
    return [(r-m)/std for r in ranges]

print fit_mean_sd_from_stream(sys.stdin)
