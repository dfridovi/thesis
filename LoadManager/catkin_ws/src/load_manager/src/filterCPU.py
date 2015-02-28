"""
Implements an exponential filter. Step response = 1 - tap^n.
"""

class FilterCPU:
    
    # constructor
    def __init__(self, _X=None, _tap=0.5):
        """
        X = current filter value -- initialize to 0
        tap = weighting of prior filter value
        """

        self.X = _X
        self.tap = _tap

    # process next position
    def update(self, cpu):
        cpu = float(cpu)
        if self.X is not None: 
            self.X = self.tap * self.X + (1 - self.tap) * cpu
        else:
            self.X = cpu


    # return current position
    def output(self):
        return self.X
