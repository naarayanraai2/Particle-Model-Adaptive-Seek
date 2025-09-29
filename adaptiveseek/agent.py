import numpy as np
import json

class dimensions():
    lengths = [[0.3516], [0.4652], [0.3678], [0.4544], [0.3895],
               [0.4922], [0.4868], [0.503], [0.4652], [0.4814],
               [0.4922], [0.3949], [0.4111], [0.3949], [0.449],
               [0.4814], [0.4814], [0.357], [0.4381], [0.4057],
               [0.3083], [0.4706]]

    def __init__(self, uid, w=0.18, wb=0.288):
        if 0 <= uid < len(self.lengths):
            self.l = self.lengths[uid][0]
        else:
            self.l = 0.39
        self.w = w
        self.wb = wb

class State():
    def __init__(self, x, y, theta, v):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v

path = 'C:/RC_Car_Lab/particle_model_adaptive_seek/Robotic-Car-Simulation/Robotic-Car-Simulation/config_files/calibration_with_collision/20241017_tom_config/tom.json'

with open(path, "r", encoding="utf-8") as f:
     _PARAM_DATA = json.load(f)["vehicle_params"]

PARAM_BY_UID = { int(p["uid"]): p for p in _PARAM_DATA }

class Vehicle():
    def __init__(self, uid, dimensions, state, acc, steer,
                 desired_speed, entrance_id, exit_id,
                 prev_acc = None, prev_steer = None, prev_state = None):
        self.uid = uid
        self.dimensions = dimensions
        self.state = state
        self.prev_state = None if prev_state is None else prev_state
        self.acc = acc
        self.steer = steer
        self.prev_acc = 0. if prev_acc is None else prev_acc
        self.prev_steer = 0. if prev_steer is None else prev_steer
        # self.desired_speed = desired_speed
        self.entrance_id = entrance_id
        self.exit_id = exit_id
        self.arrived = False
        self.epsilon_acc = 0.
        self.epsilon_steer = 0.
        self.prev_acc = 0.0
        self.prev_a_bar = 0.0

        p = PARAM_BY_UID.get(uid, None)
        if p is None:
            self.acc_noise = 0.02868
            self.risk_premium_front_kv = 0.022552
            self.risk_premium_front_rela_kv = 0.089545
            self.desired_speed = 1.04
        else:
            self.desired_speed = float(p["spd_lmt"])
            self.acc_noise = float(p["acc_noise"])
            self.risk_premium_front_kv = float(p["risk_premium_front_kv"])
            self.risk_premium_front_rela_kv = float(p["risk_premium_front_rela_kv"])


    def update_state_scalar_act(self, state, ideal_path):
        self.state = state
        self.prev_acc = self.acc
        self.prev_steer = self.steer
        self.acc =  None
        self.steer = None


def add_observation_noise(config, state):
    x_obs = state.x + config.x_observe_noise * np.random.randn()
    y_obs = state.y + config.y_observe_noise * np.random.randn()
    theta_obs = state.theta + config.psi_observe_noise * np.random.randn()
    v_obs = state.v + config.v_observe_noise * np.random.randn()

    return State(x_obs, y_obs, theta_obs, v_obs)

def add_evolution_noise(config, state):
    x_ev = state.x + config.x_evolution_noise * np.random.randn()
    y_ev = state.y + config.y_evolution_noise * np.random.randn()
    theta_ev = state.theta + config.psi_evolution_noise * np.random.randn()
    v_ev = state.v + config.v_evolution_noise * np.random.randn()
    return State(x_ev, y_ev, theta_ev, v_ev)
