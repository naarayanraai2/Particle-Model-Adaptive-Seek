import random
import matplotlib.pyplot as plt

from Configuration import config
from Vehicle import State, Vehicle
import numpy as np
from Methods import *
from dataclasses import dataclass

#TODO: Unglobalize config
#TODO: Make Data and Stats dataclasses so we can easily pass information

#IMPORTANT: positions_history[position, car_id] is the array structure
# velocities: 2d array

@dataclass
class SimulationData:
    '''
    Class for storing data and statistics.
    You can add any additional information to pass to main.
    '''
    name: str
    simulation_step: np.ndarray

    # 2D Arrays for multiple cars
    # each row correspond to one car, each column corresponds to one step in time
    position: np.ndarray
    velocity: np.ndarray

    # 1D Arrays for all cars
    max_speed: np.ndarray
    min_speed: np.ndarray
    average_speed: np.ndarray
    range: np.ndarray

def adaptive_seek(car_list, vehicle_id) -> tuple[float, float]:
    '''
    Modularized Adaptive Seek Algorithm for Simulation
    Input: 
    Output:  
    '''

    #TODO: Modularize, figure out the input / output, have it runnable for 1 time step
    
    # keep this block in or out of adaptive_seek?
    acc_list = np.array(np.arange(config['min_acc'], config['max_acc'], config['acc_grid_size'])) # 41 intervals like in the paper
    dT = np.array(np.arange(0, config['n_lookahead']*config['dt']+0.01, config['dt']))
    acc_matrix = acc_list[:, None] * dT
    #

    car = car_list[vehicle_id]
    velocities = car.state.v + acc_matrix  # Future car velocity estimates
    estimate_state = (((car.state.x + car.state.v * dT + 0.5 * acc_matrix * dT**2) % config['circumference'])+config['circumference'])%config['circumference']  # Wrap position
    front_car = car_list[car.front_uid]
    front_car_future_state = (((front_car.state.x + front_car.state.v * dT) % config['circumference'])+config['circumference'])%config['circumference']  # Wrap position
    distance = (((front_car_future_state - estimate_state) % config['circumference'])+config['circumference'])%config['circumference']  # Correct distance
    moving_forward_reward = cal_moving_forward_reward(velocities)[:, 1]
    moving_backward_penalty = cal_moving_backward_penalty(velocities)[:, 1]    
    collision_penalty = cal_ego_collision_penalty(distance, velocities, front_car.state.v)
    net_reward = moving_forward_reward - moving_backward_penalty - config['col_pen_coeff'] * collision_penalty
    best_action_index = np.argmax(net_reward)
    optimal_acceleration = acc_list[best_action_index] #previously opt_acc
    # print(opt_acc)
    return net_reward[best_action_index], optimal_acceleration

def add_cars():
    '''Initialize vehicles for simulation'''
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

def run_simulation():
    '''Crunch numbers to return position / speed of vehicles over time
    
    simulation_step: array
    positions_history: 2d array
    velocities_history: 2d array
    '''
    # Initialize the vehicles
    car_list, num_cars = add_cars()
    positions_history = []
    velocities_history = []

    for simulation_step in range(config['simulation_steps']):
        all_car_actions = []
        for vid in range(num_cars):
            util, opt_acc = adaptive_seek(car_list, vid)
            all_car_actions.append(opt_acc)
            
        # update all the vehicle state based on the selected actions
        positions, velocities = update_vehicle_state(car_list, num_cars, all_car_actions)        

        positions_history.append(positions)
        velocities_history.append(velocities)

    # Convert to numpy array for easier plotting
    velocities_history = np.array(velocities_history)
    positions_history = np.array(positions_history)  # shape: (simulation_steps, num_cars)
    simulation_step = np.array(np.arange(0, config['simulation_steps']))

    # Calculate statistics
    max_speed_history = np.max(velocities_history, axis=1)
    min_speed_history = np.min(velocities_history, axis=1)
    average_speed_history = np.average(velocities_history, axis=1)
    range_history = max_speed_history - min_speed_history

    simulation_data = SimulationData("Simulation Data", simulation_step, positions_history, velocities_history, max_speed_history, min_speed_history, average_speed_history, range_history)

    return simulation_data