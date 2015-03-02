B"""
Simple program that spits out numbers. Intended to test stdout with subprocess
(1;2cor other process-spawning module).
"""

import sys, time

if __name__ == "__main__":
    try:
        i = 0
        while True:
            print i
#            time.sleep(0.5)
            i = i + 1

    except KeyboardInterrupt:
        sys.exit()
