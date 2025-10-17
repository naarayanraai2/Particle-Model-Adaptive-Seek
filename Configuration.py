config = {
    "risk_premium_front_kc": 1,
    "risk_premium_front_rela_kv": 1,
    "risk_premium_front_kv": 1, # 0.215,

    "backward_ratio": 100,
    "num_vehicles": 20,
    "simulation_steps": 400,
    "n_lookahead": 4,

    #TODO: Define circumference in terms of radius elsewhere in the code, perhaps?
    "radius": 36.6,
    "circumference": 230,
    "speed_lmt": 5,
    "car_length": 4,
    "max_steer": 0.4,

    "dt": 0.3334,
    "max_acc": 6,
    "min_acc": -4,
    "acc_grid_size": 0.03,

    "speed_reward_coeff": 1,
    "backward_pen_coeff": 1,
    "moving_forward_ratio": 0.7,
    "error_offset": 0.025,
    "col_pen_coeff": 10,
    "rho_alpha": 0.7,
    "init_v_mu": 0.75,

    # "acceleration_noise": 0.1,
    # "state_noise": 0.05,
    # "v_evolution_noise": 0.1,


    "acceleration_noise": 0,
    "state_noise": 0,
    "v_evolution_noise": 0,
}