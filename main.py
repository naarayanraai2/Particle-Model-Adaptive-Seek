import simulation
import graphing

#TODO: Remove irrelevant include paths

class Grapher:
    def __init__(self):
        self.data = simulation.run_simulation()

    def graph_x_vs_t(self):
        graphing.graph_x_vs_t(self.data)

    def graph_statistics(self):
        graphing.graph_statistics(self.data)

if __name__ == '__main__':
    data = []
    grapher = Grapher()
    grapher.graph_x_vs_t()
    # grapher.graph_statistics()

