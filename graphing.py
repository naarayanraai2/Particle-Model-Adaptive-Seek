import random
import matplotlib.pyplot as plt

from Configuration import config
from Vehicle import State, Vehicle
from simulation import adaptive_seek
import numpy as np
from Methods import *

def graph_x_vs_t(simulation_step, positions_history):
    """Plot positions of all cars over time"""
    plt.figure(figsize=(10, 6))
    for vid in range(config["num_vehicles"]):
        plt.scatter(simulation_step, positions_history[:, vid], label=f'Car {vid}', s=5)
    plt.xlabel('Simulation Step')
    plt.ylabel('Position (state.x)')
    plt.title('Car Positions Over Time')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()

#TODO: Make it so we can choose which graphs to show

def graph_stats(simulation_step, stats):
    """graph all stats"""
    plt.figure(figsize=(10, 6))
    for key in stats:
        #TODO: line plot?
        plt.scatter(simulation_step, stats[f"{key}"], label=f"{key}", s=10)
    plt.xlabel('Simulation Step')
    plt.ylabel('Speed Range')
    plt.title('Speed Range Over Time')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()      
