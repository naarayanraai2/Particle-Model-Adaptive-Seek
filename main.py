import simulation
import animate
import graphing

class Grapher:
    def __init__(self):
        self.data = simulation.run_simulation()

    def graph_x_vs_t(self):
        graphing.graph_x_vs_t(self.data)

    def graph_statistics(self):
        graphing.graph_statistics(self.data)
    
    def animate(self):
        animate.circular(self.data)

if __name__ == '__main__':
    data = []
    grapher = Grapher()
    grapher.graph_x_vs_t()

