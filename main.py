import simulation
import animate
import graphing
import pickle

class Grapher:
    """
    Choose what to graph using the following functions

    Optional input to load your own .pkl file using SimulationData class
    """
    def __init__(self, data_filepath=None):
        if data_filepath:
            self.data = self.load_data(data_filepath)
        else:
            self.data = simulation.run_simulation()

    def graph_x_vs_t(self):
        graphing.graph_x_vs_t(self.data)

    def graph_statistics(self):
        graphing.graph_statistics(self.data)
    
    def animate_circular(self):
        animate.circular(self.data)

    def save_data(self, filepath):         
        with open(filepath, 'wb') as f:
            pickle.dump(self.data, f)

    def load_data(self, filepath):
        with open(filepath, 'rb') as f:
            return pickle.load(f)

if __name__ == '__main__':
    filepath = "./data.pkl"
    grapher = Grapher()
    grapher.graph_x_vs_t()
    grapher.animate_circular()

