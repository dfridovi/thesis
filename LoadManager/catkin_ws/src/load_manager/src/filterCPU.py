"""
Implements an exponential filter.
"""

class FilterCPU:
    
    # constructor
    def __init__(self, _tap=0.5):
        """
        X = current filter value
        tap = weighting of prior filter value
        """

        self.X = None
        self.tap = _tap

    # process next position
    def update(self, cpu):
        cpu = float(cpu)
        if not self.X: 
            self.X = cpu
        else:
            self.X = self.tap * self.X + (1 - self.tap) * cpu

    # return current position
    def output(self):
        return self.X
