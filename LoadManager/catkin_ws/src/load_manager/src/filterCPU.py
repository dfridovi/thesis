"""
Implements an exponential filter.
"""

from collections import deque

class FilterCPU:
    
    # constructor
    def __init__(self, _X=0.0, _tap=0.5):
        """
        X = position vector as list
        tap = weighting of prior filter value
        """

        self.X = float(X)
        self.tap = _tap

    # process next position
    def update(self, cpu):
        cpu = float(cpu)
        self.X = self.tap * self.X + (1 - self.tap) * cpu

    # return current position
    def position(self):
        return self.X
