import random
import matplotlib.pyplot as plt

from Configuration import config
from Vehicle import State, Vehicle
from simulation import AdaptiveSeek
import numpy as np
from Methods import *

def add_cars():
    n_cars = config['num_vehicles']
    new_car_list = []
    init_x = 0.01
    init_acc = 0.0
    dist_per_car = config['circumference']/n_cars + random.uniform(-0.5, 0.5)
    for car_id in range(n_cars): # Loop numbers for each car
        init_v = config['init_v_mu'] + random.uniform(-2.5, 2.5)
        state = State(init_x, init_v)
        car = Vehicle(car_id, state, init_acc, config['speed_lmt'], init_acc, state)
        new_car_list.append(car)
        init_x = init_x+dist_per_car
    return new_car_list, n_cars

if __name__ == '__main__':
    # Initialize the vehicles
    car_list, num_cars = add_cars()
    positions_history = []
    velocities_history = []

    for simulation_step in range(config['simulation_steps']):
        all_car_actions = []
        for vid in range(num_cars):
            util, opt_acc = AdaptiveSeek(car_list, vid)
            all_car_actions.append(opt_acc)
            
        # update all the vehicle state based on the selected actions
        positions, velocities = update_vehicle_state(car_list, num_cars, all_car_actions)        

        positions_history.append(positions)
        velocities_history.append(velocities)

    # Convert to numpy array for easier plotting
    positions_history = np.array(positions_history)  # shape: (simulation_steps, num_cars)
    simulation_step = np.array(np.arange(0, config['simulation_steps']))
    # Plot positions of all cars over time
    plt.figure(figsize=(10, 6))

    for vid in range(num_cars):
        plt.scatter(simulation_step, positions_history[:, vid], label=f'Car {vid}', s=5)
    plt.xlabel('Simulation Step')
    plt.ylabel('Position (state.x)')
    plt.title('Car Positions Over Time')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()
