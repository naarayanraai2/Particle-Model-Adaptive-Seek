import simulator
from idealpath import IdealPath

def adaptiveSeek_vectorized(ideal_path, config, car_list, ego_car_id, is_softmax=True):
    '''
    @params
        ideal_path: idealpath.IdealPath class
        config: import values from config.json file
        car_list: list of agent.Vehicle classes
        ego_car_id: current car id -> maps to value in config
    '''
    ego_agent = car_list[ego_car_id]

    n = len(car_list)
    nearby_agents = car_list[(ego_car_id + 1) % n]

    X_ego, Y_ego, Theta_ego, V_ego,Theta_current = simulator.cal_ego_lookahead_states(ideal_path, config, ego_agent)

    X_traffic, Y_traffic, Theta_traffic, V_traffic,v_current = simulator.cal_traffic_agent_lookahead_states(
            ideal_path, config, nearby_agents)

    delta_X_ego = simulator.relative_dist_traffic_to_ego(config, X_ego,
            Y_ego, Theta_ego, X_traffic, Y_traffic, Theta_traffic)

    speed_reward = simulator.cal_moving_forward_reward(V_ego[:,0], ego_agent.desired_speed, config.moving_forward_ratio)

    backward_pen = simulator.cal_moving_backward_penalty(V_ego[:,0], config.backward_ratio, config.error_offset)

    col_pen = simulator.cal_ego_collision_penalty(config, delta_X_ego,
            V_ego, v_current,ego_agent)

    utility = (config.speed_reward_coeff*speed_reward
            - config.backward_pen_coeff*backward_pen
            - config.col_pen_coeff * col_pen
            )
    if is_softmax:

        opt_action = simulator.get_softmax_optimal_action(config, utility, beta=config.beta)


    ego_agent.prev_state=ego_agent.state

    return opt_action

if __name__ == "__main__":
        adaptiveSeek_vectorized()

