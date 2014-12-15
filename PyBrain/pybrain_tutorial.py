"""
PyBrain tutorial. Creates a small neural network to model the XOR function.
"""

from pybrain.tools.shortcuts import buildNetwork
from pybrain.structure import TanhLayer, SoftmaxLayer
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer

net = buildNetwork(2, 3, 1, 
                   bias=True, hiddenclass=TanhLayer)
print "Activating initial network: " + str(net.activate([0, 1]))

ds = SupervisedDataSet(2, 1)
ds.addSample((0, 0), (0,))
ds.addSample((0, 1), (1,))
ds.addSample((1, 0), (1,))
ds.addSample((1, 1), (0,))

trainer = BackpropTrainer(net, ds, 
                          learningrate=0.1, momentum=0.9, verbose=True)
trainer.trainUntilConvergence(maxEpochs=500)
