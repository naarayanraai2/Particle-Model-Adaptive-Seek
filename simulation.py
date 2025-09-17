from Methods import *
from Vehicle import Vehicle

acc_list = np.array(np.arange(config['min_acc'], config['max_acc'], config['acc_grid_size'])) # 41 intervals like in the paper
dT = np.array(np.arange(0, config['n_lookahead']*config['dt']+0.01, config['dt']))
acc_matrix = acc_list[:, None] * dT

def AdaptiveSeek(car_list, vehicle_id):
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
    opt_acc = acc_list[best_action_index]
    # print(opt_acc)
    return net_reward[best_action_index], opt_acc