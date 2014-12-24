"""
Author: David Fridovich-Keil
Senior thesis, Fall 2014.

Implements a simple low-pass IIR filter (with only a single tap), to track
a 2D position over time.
"""

import numpy as np

class Filter2D:
    
    # constructor
    def __init__(self, _X, _tap=0.5):
        """
        X = position vector as list
        tap = weighting of prior filter value
        """

        self.X = np.float32(_X)
        self.tap = _tap

    # process next position
    def update(self, pos):
        pos = np.float32(pos)
        self.X = self.tap * self.X + (1 - self.tap) * pos

    # return current position
    def position(self):
        return self.X.tolist()
