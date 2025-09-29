import json
import os

class Config:
    def __init__(self, file_name):
        """
        Load config from json file
        """
        with open(file_name) as f:
            params = json.load(f)

        # simulation environemnt selfuration
        self.lane_width = params['lane_width'] # unit: meter
        self.spd_lmt = params['spd_lmt'] # original 20 mph -> 8.89 m/s, this is used as ideal speed
        self.dt = params['dt'] # delta time between simulation steps, unit: second
        self.simulation_steps = params['simulation_steps']
        self.density = params['density'] # car density in simulation.
        self.seed = params['seed'] # numpy random seed
        self.parallel_workers = params['parallel_workers']# number of cores for simulation.
        self.radius = params['radius']
        self.shock_enabled = True  # 启用/关闭外部扰动
        self.shock_steps = set(range(30, 50))  # 对应你的 range(30,50)
        self.shock_targets = {0}  # 作用到哪些车（按 uid）
        self.shock_mode = "override_to_stop"  # "override_to_stop" 或 "add_bias"
        self.shock_limit = -1.0  # 覆盖模式下的最小刹车上限（与原逻辑一致）
        self.shock_bias = -0.8  # add_bias 模式的加速度偏置（负值=额外刹车）

        # default car dimensions, unit: meter
        self.car_length = params['car_length']
        self.car_width = params['car_width']
        self.car_wheel_base = params['car_wheel_base']

        # cars' kinematic physical limits
        self.max_acc = params['max_acc'] # unit: m/s^2 original 4
        self.min_acc = params['min_acc'] # unit: m/s^2 original -6
        self.max_steer = params['max_steer'] # unit: radian original 0.4

        # adaptiveSeek selfuration
        self.acc_grid_size = params['acc_grid_size'] # original 0.5
        self.steer_grid_size = params['steer_grid_size'] # original 0.02
        self.n_lookahead = params['n_lookahead']   # original 6
        self.act_persis_steps = params['act_persis_steps'] # action persistency steps original 3
		
        # utility parameters
        self.speed_reward_coeff = params['speed_reward_coeff']
        self.backward_pen_coeff = params['backward_pen_coeff']
        self.roughness_pen_acc_coeff = params['roughness_pen_acc_coeff'] # roughness penalty coefficient for acceleration
        self.lat_acc = params['lat_acc']
        # self.roughness_pen_brak_coeff = 0. # roughness penalty coefficient for braking
        self.roughness_pen_steer_coeff = params['roughness_pen_steer_coeff'] # roughness penalty coefficient for steering
        self.lane_departure_pen_coeff = params['lane_departure_pen_coeff'] # original 1 ideal path departure penalty coefficient
        self.col_pen_coeff = params['col_pen_coeff'] # collision penalty coefficient
		
        # only check collision within the radius
        self.ego_radius = params['ego_radius']

        # risk premium related settings
        self.safe_distance_front_kv = params['safe_distance_front_kv'] # original 0
        self.safe_distance_front_kc = params['safe_distance_front_kc'] # original 0
        self.risk_premium_front_kv = params['risk_premium_front_kv'] # unit: second, original 1.2
        self.risk_premium_front_rela_kv = params['risk_premium_front_rela_kv']
        self.risk_premium_front_kc = params['risk_premium_front_kc'] # unit: meter, original 1
        self.safe_distance_rear_kv = params['safe_distance_rear_kv'] # original 0
        self.safe_distance_rear_kc = params['safe_distance_rear_kc'] # original 0
        self.risk_premium_rear_kv = params['risk_premium_rear_kv'] # unit: second, original 0.02
        self.risk_premium_rear_kc = params['risk_premium_rear_kc'] # unit: meter, original 0.1
        self.safe_distance_side_kv = params['safe_distance_side_kv'] # original 0
        self.safe_distance_side_kc = params['safe_distance_side_kc'] # original 0
        self.risk_premium_side_kv = params['risk_premium_side_kv'] # unit: second, original 0.03
        self.risk_premium_side_kc = params['risk_premium_side_kc'] #unit meter, original 0.5
        self.RF_ratio = params['RF_ratio'] # rear/front collision penalty ratio, original 0.3

        # AR1 coefficient
        self.rho_alpha = params['rho_alpha'] # original 0.7
        self.rho_delta = params['rho_delta'] # original 0.7

        # state evolution noise
        self.acc_noise = params['acc_noise']
        self.steer_noise = params['steer_noise']
        self.x_evolution_noise = params['x_evolution_noise']
        self.y_evolution_noise = params['y_evolution_noise']
        self.psi_evolution_noise = params['psi_evolution_noise']
        self.v_evolution_noise = params['v_evolution_noise']

        # observation noise
        self.x_observe_noise = params['x_observe_noise'] # unit meter
        self.y_observe_noise = params['y_observe_noise'] # unit meter
        self.v_observe_noise = params['v_observe_noise'] # used for calculate the optimal control
        self.psi_observe_noise = params['psi_observe_noise'] # unit radian

        # initial condition params
        self.init_v_mu = params['init_v_mu'] # 20 mph -> 8.89 m/s
        self.init_v_sigma = params['init_v_sigma']
        self.destination_dist = params['destination_dist'] # criteria to check if the agent has arrived
        self.init_pos_std = params['init_pos_std'] # init position deviation from ideal path
        self.init_angle_std = params['init_angle_std'] # init angle deviation from ideal path

        # softmax beta
        self.beta = params['beta']
        self.with_softmax = params['with_softmax']
        
        # yield sign
        #self.yield_sign = [(86.8001, -69.63880),  (103.7523, -32.6744), (65.4842, -9.0108), (44.3727, -42.8720)]  #Tom old result south, east, north, west
        self.yield_sign = [(86.3978, -71.682), (106.412, -36.317), (67.653, -10.6389), (44.589, -43.867)] # tom newest result
        #self.yield_sign = [(98.91, 137.835),  (70.47, 134.55), (72.18, 108.135), (104.175, 114.705)]
        #self.yield_sign2 = [(105.84, 140.625), (68.22, 143.865), (64.8, 106.11), (107.01, 107.55)]
        #self.yield_sign2  = [(101.7, 138.6), (69.34, 138.24), (69.88, 108.0), (105.34, 109.98)]
        #self.center_coor = (86, 123) # original
        self.center_coor = (74.67191, -40.620835) # tom
        self.sigma_xy = params['sigma_xy']
        self.sigma_v = params['sigma_v']
        self.yield_pen_coeff = params['yield_pen_coeff']

        self.lat_acc_soft_penalty = params['lat_acc_soft_penalty']
        self.lat_acc_hard_penalty = params['lat_acc_hard_penalty']
        
        self.moving_forward_ratio = params['moving_forward_ratio']

        self.backward_ratio = params['backward_ratio']
        self.error_offset = params['error_offset']
        self.curvature_pen_coeff = params['curvature_pen_coeff']

        self.outer_radius = params['outer_radius']
        self.inner_radius = params['inner_radius']

        self.lane_pen_ratio = params['lane_pen_ratio']
        
        self.lane_orientation_pen_coeff = params['lane_orientation_pen_coeff']
        
        self.yield_sign = {'n1': (65.92, 76.35),  
                           'n2': (61.39, 74.92), 
                           'nr': (50.01, 72.49),
                           's1': (82.04, 20.30),
                           's2': (86.17, 20.92),
                           'w1': (43.73, 45.96),
                           'w2': (45.80, 40.12),
                           'e1': (101.06, 50.96),
                           'e2': (100.80, 56.18)}
        
        self.yield_sign_real_world = {'n1': (42.229604, -83.739107),  
                           'n2': (42.229585, -83.739175), 
                           'nr': (42.229555, -83.739307),
                           's1': (42.229186, -83.738870),
                           's2': (42.229203, -83.738807),
                           'w1': (42.229352, -83.739306),
                           'w2': (42.229293, -83.739275),
                           'e1': (42.229444, -83.738721),
                           'e2': (42.229496, -83.738749)}

        self.circle_map_dir = os.path.join(r'../basemap/ROIs-map-new/circle')
        self.entrance_map_dir = os.path.join(r'../basemap/ROIs-map-new/entrance')
        self.exit_map_dir = os.path.join(r'../basemap/ROIs-map-new/exit')
        self.yielding_area_map_dir = os.path.join(r'../basemap/ROIs-map-new/yielding-area')
        self.at_circle_lane_map_dir = os.path.join(r'../basemap/ROIs-map-new/at-circle-lane')
        self.drivable_map_dir =  r'../basemap/drivablemap/AA_rdbt-drivablemap.jpg'
        self.map_dir=r'../preprocess/AA_rdbt.png'
        self.map_height=698 # The map height and width ratio should be consistent with the basemap and drivable map pictures.
        self.map_width=854
        self.car_dir = r'../preprocess/Veh_images'