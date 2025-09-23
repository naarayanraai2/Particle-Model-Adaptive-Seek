from Vehicle import Vehicle
from Configuration import config
import random
import numpy as np

def update_vehicle_state(car_list, num_cars, all_car_actions):
    positions = []
    velocities = []
    for vid in range(num_cars):
        opt_acc = all_car_actions[vid]
        acc = config['rho_alpha'] * car_list[vid].acc + (opt_acc-config['rho_alpha']*car_list[vid].opt_acc) + random.gauss(0, 0.273)
        car_list[vid].opt_acc = opt_acc
        car_list[vid].acc = acc
        
        car_list[vid].state.v = car_list[vid].state.v + car_list[vid].acc * config['dt'] + np.random.randn()*config['v_evolution_noise']
        pos_noise = np.random.normal(0, config['state_noise'])
        car_list[vid].state.x = (((car_list[vid].state.x+config['dt']*car_list[vid].state.v+pos_noise)%config['circumference'])+config['circumference'])%config['circumference']

        positions.append(car_list[vid].state.x)
        velocities.append(car_list[vid].state.v)

    return positions, velocities

def cal_moving_forward_reward(V_ego):
    return  np.exp(- ((V_ego - config['speed_lmt'])/(config['moving_forward_ratio'] * config['speed_lmt'])) ** 2)

def cal_moving_backward_penalty(V_ego):
    return np.exp(-config['backward_ratio'] * (V_ego+config['error_offset']))

def  cal_ego_collision_penalty(distance, Vi, Vj):
    rela_v = Vi - Vj

    bumper_bumper_distance = distance-config['car_length']

    risk_premium_front = \
        config['risk_premium_front_kv'] * np.abs(Vi) +\
        config['risk_premium_front_kc'] +\
        config['risk_premium_front_rela_kv'] * np.maximum(rela_v,0)
    col_pen_X = col_risk_longitudinal(bumper_bumper_distance,risk_premium_front)
    col_pen_aggregated = np.max(col_pen_X, axis=1)
    return col_pen_aggregated

def col_risk_longitudinal(dist_vec,rpf):
    col_risk = np.where(dist_vec>0, np.exp(-((dist_vec/ rpf) ** 2 + 2 * (dist_vec/ rpf))), 1)
    return col_risk