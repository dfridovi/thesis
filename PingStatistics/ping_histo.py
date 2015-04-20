#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from: https://raw.githubusercontent.com/pklaus/ping_histo/master/ping_histo.py
# edited to use matplotlib

import subprocess
import re
import sys
import matplotlib.pyplot as plt
import numpy as np
import cPickle as pickle

def mean(values):
    # starting with Python 3.4 there is the module statistics
    # http://docs.python.org/3.4/library/statistics.html
    return float(sum(values))/len(values) if len(values) > 0 else float('nan')

single_matcher = re.compile("(?P<bytes>\d+) bytes from (?P<IP>\d+.\d+.\d+.\d+): icmp_seq=(?P<sequence>\d+) ttl=(?P<ttl>\d+) time=(?P<time>\d+(.\d+)?) ms")
# should match lines like this:
# 64 bytes from 192.168.178.45: icmp_seq=2 ttl=64 time=103 ms
end_matcher = re.compile("rtt min/avg/max/mdev = (?P<min>\d+.\d+)/(?P<avg>\d+.\d+)/(?P<max>\d+.\d+)/(?P<mdev>\d+.\d+)")
# should match lines like this:
# rtt min/avg/max/mdev = 0.234/0.234/0.234/0.000 ms
# alternative matcher for a different ping utility:
#end_matcher = re.compile("round-trip min/avg/max/stddev = (\d+.\d+)/(\d+.\d+)/(\d+.\d+)/(\d+.\d+)")

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Ping a host and create histogram.')
    parser.add_argument('host', help='The host to ping')
    parser.add_argument('--count', '-c', type=int, default=4, help='Number of times the host should be pinged')
    parser.add_argument('--interval', '-i', type=float, default=1.0, help='Interval between individual pings.')
    parser.add_argument('--debug', '-d', action='store_true', help='Enable debug output for this script')
    args = parser.parse_args()

    ping = subprocess.Popen(
        ["ping", "-c", str(args.count), "-i", str(args.interval), args.host],
        stdout = subprocess.PIPE,
        stderr = subprocess.STDOUT
    )

    try:
        times = []
        sentinel = b"" if sys.version_info[0] >= 3 else ""
        for line in iter(ping.stdout.readline, sentinel):

            line = line.decode('ascii')
            if args.debug: print("Analyzing line: " + line)
            if line == u"\n": continue
            line = line.replace('\n', '')

            match = single_matcher.match(line)
            if match:
                if args.debug: print(match.groups())
                time = float(match.group('time'))
                times.append(time)
                print("%d: %f" % (len(times), time))
                continue
            match = end_matcher.match(line)
            if match:
                if args.debug: print(match.groups())
                continue
            if args.debug: print("Didn't understand this line: " + line)
    except KeyboardInterrupt:
        file = open("ping_times.pkl", 'wb')
        pickle.dump(np.asarray(times, dtype=np.float), file)
        file.close()

        n, bins, patches = plt.hist(np.asarray(times, dtype=np.float), 20, facecolor='g', alpha=0.75)
        plt.xlabel('Ping times (ms)')
        plt.ylabel('Frequency')
        plt.title('Ping Time Distribution: 10.9.160.238')
        plt.grid(True)
        plt.show()
        sys.exit()

    exitCode = ping.returncode
    if args.debug: print("Exit Code of the ping command: " + str(ping.returncode))

    #print(mean(times))

if __name__ == "__main__":
    main()
