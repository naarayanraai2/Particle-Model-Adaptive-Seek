import random
import matplotlib.pyplot as plt

from Configuration import config
from Vehicle import State, Vehicle
import simulation
import graphing
import numpy as np
from Methods import *

class Grapher:
    def __init__(self):
        # Generate Data and Statistics
        #TODO: We can be much smarter. Put all the data into a dictionary, like we have done with the stats
        self.simulation_step, self.positions_history, self.velocities_history = simulation.run_simulation()
        self.stats = simulation.calculate_statistics(self.simulation_step, self.positions_history, self.velocities_history)

    def graph_x_vs_t(self):
        graphing.graph_x_vs_t(self.simulation_step, self.positions_history)

    def graph_stats(self):
        graphing.graph_stats(self.simulation_step, self.stats)


if __name__ == '__main__':
    data = []
    grapher = Grapher()
    grapher.graph_stats()
