"""
Simple program that spits out numbers. Intended to test stdout with subprocess
(or other process-spawning module).
"""

import sys

if __name__ == "__main__":
    try:
        i = 0
        while True:
            print i
            i = i + 1

    except KeyboardInterrupt:
        sys.exit()
