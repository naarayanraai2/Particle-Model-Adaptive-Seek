import sys
sys.path.append('../')
import numpy as np
from agent import (State, add_evolution_noise, add_observation_noise,
    Vehicle, dimensions)


def sigmoid(x):
    return 1./(1.+np.exp(-x))

def kinemetics_scalar(x0, y0, v0, theta0, acc, steer, dt, wb):

    v = v0 + acc * dt
    delta = steer
    beta = np.arctan(0.5*np.tan(delta))
    dtheta = v0 * dt / wb * np.cos(beta) * np.tan(delta)
    theta = theta0 + dtheta

    theta = (theta+np.pi)%(np.pi*2)-np.pi
    dx = v0 * dt * np.cos(theta0+beta)
    x = x0 + dx
    dy = v0 * dt * np.sin(theta0+beta)
    y = y0 + dy
    return x, y, v, theta, None, None, None

def col_risk_longitudinal(dist_vec, rpf):

    longit_dist_F_2 = (dist_vec / rpf) ** 2 + 2 * (dist_vec / rpf)
    col_risk = (np.exp(-longit_dist_F_2) * (dist_vec > 0)) + \
               (1. * (dist_vec <= 0))

    return col_risk

def single_agent_state_update(config, agent):
    s = agent.state
    acc = agent.acc
    steer = agent.steer
    x, y, v, theta, _, _, _ = kinemetics_scalar(s.x, s.y,
            s.v, s.theta, acc=acc, steer=steer, dt=config.dt, wb=config.car_wheel_base)
    return State(x, y, theta, v)

def newcar_oldcars_collision(newcar_state, oldcars, path_id, safe_dist=1):
    entrance_id, exit_id = path_id.split('_')[0],path_id.split('_')[1]

    if len(oldcars)<1:
        return False

    new_x = newcar_state.x
    new_y = newcar_state.y

    for agent in oldcars:
        if entrance_id == agent.entrance_id:
            x, y = agent.state.x, agent.state.y
            if (new_x - x)**2 + (new_y - y)**2 < safe_dist**2:
                return True
    return False

def add_new_car_fixed_path_on_circle(ideal_path, config, car_list, uid_max,
                                     entrance_id='1', exit_id='2', point_index=0,
                                     with_noise=False):
    new_car_list = []
    path_id = f"{entrance_id}_{exit_id}"
    xs, ys = ideal_path._path[path_id]
    angles = ideal_path._angle[path_id]
    point_index = point_index % len(xs)
    x = xs[point_index]
    y = ys[point_index]
    angle = angles[point_index]
    if with_noise:
        x += config.init_pos_std * np.random.randn()
        y += config.init_pos_std * np.random.randn()
        angle += config.init_angle_std * np.random.randn()
    init_speeds = [
        0.813, 0.948863, 0.747525, 0.819600, 0.700481, 0.785681, 0.857625,
        0.673969, 0.674438, 0.754219, 0.703669, 0.677269, 0.729700, 0.672337,
        0.582319, 0.610106, 0.594825, 0.602719, 0.780263, 0.645338, 0.740906, 0.855919
    ]
    v_init = init_speeds[uid_max] if uid_max < len(init_speeds) else np.random.uniform(0.6, 1)
    init_state = State(x=x, y=y, v=v_init, theta=angle)

    if not newcar_oldcars_collision(init_state, car_list, path_id):
        new_car = Vehicle(uid=uid_max,
                          dimensions=dimensions(uid_max),
                          state=init_state,
                          acc=0,
                          steer=0,
                          desired_speed=config.spd_lmt,
                          entrance_id=entrance_id,
                          exit_id=exit_id)
        new_car_list.append(new_car)
        uid_max += 1
    print(f"Add vehicle {uid_max} at ({x:.2f}, {y:.2f}) with angle {np.degrees(angle):.2f}°")

    return new_car_list, uid_max

def cal_ego_lookahead_states(ideal_path, config, ego_agent):
    acc_grid, _ = create_possible_actions(config)
    n_actions = len(acc_grid)
    h = config.n_lookahead
    k = config.act_persis_steps

    s = ego_agent.state
    x, y, theta, v = s.x, s.y, s.theta, s.v
    entrance_id, exit_id = ego_agent.entrance_id, ego_agent.exit_id

    X = np.zeros((n_actions, h+1)); X[:, 0] = x
    Y = np.zeros((n_actions, h+1)); Y[:, 0] = y
    Theta = np.zeros((n_actions, h+1)); Theta[:, 0] = theta
    V = np.zeros((n_actions, h+1)); V[:, 0] = v
    for i in range(1, h+1):
        update_state_evolution(ideal_path, config, entrance_id, exit_id,
                               X, Y, Theta, V, i, acc_grid, steer_grid=None)

    return X[:, 1:], Y[:, 1:], Theta[:, 1:], V[:, 1:],Theta[:,0]

def update_state_evolution(ideal_path, config, entrance_id, exit_id,
                           X, Y, Theta, V, step, acc_grid, steer_grid):

    pre_X    = X[:, step-1]
    pre_Y    = Y[:, step-1]
    pre_Theta= Theta[:, step-1]
    pre_V    = V[:, step-1]

    if isinstance(steer_grid, np.ndarray):
        delta = steer_grid
    else:
        delta, diag = ideal_path.cal_stanley_angle(
            entrance_id, exit_id, pre_X, pre_Y, pre_Theta, pre_V, config.max_steer
        )

    cur_X, cur_Y, cur_V, cur_Theta, *_ = kinemetics_scalar(
        pre_X, pre_Y, pre_V, pre_Theta,
        acc=acc_grid, steer=delta, dt=config.dt, wb=config.car_wheel_base
    )

    X[:, step]     = cur_X
    Y[:, step]     = cur_Y
    V[:, step]     = cur_V
    Theta[:, step] = cur_Theta

def create_possible_actions(config):

    acc = np.arange(config.min_acc, config.max_acc+config.acc_grid_size, config.acc_grid_size)
    steer = np.arange(-config.max_steer, config.max_steer+config.steer_grid_size,
            config.steer_grid_size)
    acc_grid0, steer_grid0 = np.meshgrid(acc, steer, sparse=False, indexing='ij')

    acc_grid = acc_grid0.flatten()
    acc_grid = np.append(acc_grid,0)
    steer_grid = steer_grid0.flatten()
    steer_grid = np.append(steer_grid, 0)

    return acc_grid, steer_grid

def get_softmax_optimal_action(config, utility, beta=200):
    acc_grid, steer_grid = create_possible_actions(config)
    opt_acc = softmax(utility, acc_grid, beta)

    return opt_acc

def cal_traffic_agent_lookahead_states(ideal_path, config, traffic_agents ,defin_h = 0, ):

    n_traffic = 1

    h = config.n_lookahead
    if defin_h != 0 :
        h = defin_h
    traffic_X = np.zeros((n_traffic, h))
    traffic_Y = np.zeros((n_traffic, h))
    traffic_Theta = np.zeros((n_traffic, h))
    traffic_V = np.zeros((n_traffic, h))

    x, y, theta, v,v_current = single_traffic_agent_lookahead(ideal_path, config, traffic_agents, h)
    traffic_X[0, :] = x
    traffic_Y[0, :] = y
    traffic_Theta[0, :] = theta
    traffic_V[0, :] = v

    return traffic_X, traffic_Y, traffic_Theta, traffic_V,v_current

def single_traffic_agent_lookahead(ideal_path, config, agent,defin_h = 0):

    state = agent.state
    x0, y0, theta0, v0 = state.x, state.y, state.theta, state.v
    entrance_id = agent.entrance_id
    exit_id = agent.exit_id

    X, Y, Theta, V = [x0], [y0], [theta0], [v0]
    h = config.n_lookahead
    if defin_h != 0 :
        h = defin_h

    for i in range(h):
        acc = 0
        delta, diag = ideal_path.cal_stanley_angle(entrance_id, exit_id, X[-1],
                Y[-1], Theta[-1], V[-1], config.max_steer)
        steer = delta

        x, y, v, theta, *rest = kinemetics_scalar(X[-1], Y[-1],
                V[-1], Theta[-1], acc=acc, steer=steer, dt=config.dt, wb=config.car_wheel_base)
        X.append(x)
        Y.append(y)
        V.append(v)
        Theta.append(theta)

    return np.array(X[1:]), np.array(Y[1:]), np.array(Theta[1:]), np.array(V[1:]),np.array([[V[0]]])

def relative_dist_traffic_to_ego(config, X, Y, Theta,
                                      traffic_X, traffic_Y, traffic_Theta):

    l, w = config.car_length, config.car_width
    A, h = X.shape
    N = traffic_X.shape[0]

    if N == 0:
        return np.zeros((A, 0, h), dtype=np.float32)

    ego_X8, ego_Y8 = create_8_points(X, Y, Theta, l, w)
    trf_X8, trf_Y8 = create_8_points(traffic_X, traffic_Y, traffic_Theta, l, w)

    ex = ego_X8[:,  None, :, :, None]
    ey = ego_Y8[:,  None, :, :, None]
    tx = trf_X8[None, :, :, None, :]
    ty = trf_Y8[None, :, :, None, :]

    d = np.sqrt((ex - tx)**2 + (ey - ty)**2)
    d_min = d.min(axis=(-1, -2))
    d_min = d_min.squeeze(axis=1)

    return d_min

def create_8_points(X, Y, Theta, l, w):

    l_sin = l/2 * np.sin(Theta)
    l_cos = l/2 * np.cos(Theta)
    w_sin = w/2 * np.sin(Theta)
    w_cos = w/2 * np.cos(Theta)

    front_left_X = l_cos - w_sin
    front_left_Y = l_sin + w_cos
    front_mid_X = l_cos
    front_mid_Y = l_sin
    front_right_X = l_cos + w_sin
    front_right_Y = l_sin - w_cos
    mid_left_X = -w_sin
    mid_left_Y = w_cos
    mid_right_X = w_sin
    mid_right_Y = -w_cos
    end_left_X = -l_cos - w_sin
    end_left_Y = -l_sin + w_cos
    end_mid_X = -l_cos
    end_mid_Y = -l_sin
    end_right_X = -l_cos + w_sin
    end_right_Y = -l_sin - w_cos

    X_8 = (np.expand_dims(X, axis=-1) +
            np.stack([front_left_X, front_mid_X, front_right_X, mid_left_X, mid_right_X,
            end_left_X, end_mid_X, end_right_X], axis=-1))
    Y_8 = (np.expand_dims(Y, axis=-1) +
            np.stack([front_left_Y, front_mid_Y, front_right_Y, mid_left_Y, mid_right_Y,
                end_left_Y, end_mid_Y, end_right_Y], axis=-1))

    return X_8, Y_8

def cal_ego_collision_penalty(config, delta_X_ego, v, v_current,ego_agent):

    if delta_X_ego.shape[1] == 0:
        return np.zeros(delta_X_ego.shape[0])
    velo = v
    rela_v = velo - v_current
    risk_premium_front = (ego_agent.risk_premium_front_kv * np.abs(velo) +
                          config.risk_premium_front_kc + ego_agent.risk_premium_front_rela_kv * np.maximum(rela_v, 0))

    col_pen_X = col_risk_longitudinal(delta_X_ego, risk_premium_front)
    col_pen_aggregated = np.max(col_pen_X, axis=1)
    return col_pen_aggregated

def cal_moving_forward_reward(V_ego, v_ideal, ratio):
    return np.exp(- ((V_ego - v_ideal) / (ratio* v_ideal)) ** 2)

def cal_moving_backward_penalty(V_ego, backward_ratio, error_offset):
    return np.exp(-backward_ratio * (V_ego+error_offset))

def softmax(weights: np.array, actions: np.array, beta=200) -> np.array:
    weights = weights - np.max(weights)
    w_exp = np.exp(beta * weights) / np.sum(np.exp(beta * weights))

    return np.sum(actions * w_exp)

def _wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def _stanley_on_circle(x, y, theta, v, R, L, max_steer, k=2.5, k_soft=0.7, cw=False):
    r = np.hypot(x, y)
    e_ct = r - R
    psi_tan = np.arctan2(y, x) + (-np.pi/2 if cw else np.pi/2)
    head_err = _wrap_pi(psi_tan - theta)
    st_term = np.arctan2(k * e_ct, abs(v) + k_soft)
    delta_ff = np.arctan(L / R) * (-1 if cw else 1)
    delta = head_err + st_term + delta_ff
    return np.clip(delta, -max_steer, max_steer)

def add_noise(config, car_list, opt_rst, ideal_path, step=None):
    import numpy as np
    log_states = []

    def _get_acc(opt_item):
        if isinstance(opt_item, (list, tuple, np.ndarray)):
            return float(opt_item[0])
        return float(opt_item)

    for i in range(len(car_list)):
        car_list[i].epsilon_acc = 0.0
        car_list[i].epsilon_steer = 0.0

        a_bar_t = _get_acc(opt_rst[i])

        if getattr(config, "shock_enabled", False) and (step is not None):
            uid = getattr(car_list[i], "uid", i)
            if (step in getattr(config, "shock_steps", set())) and (uid in getattr(config, "shock_targets", set())):
                v_now = float(car_list[i].state.v)
                a_to_stop = (0.0 - v_now) / config.dt
                a_bar_t = max(
                    np.clip(a_to_stop, config.min_acc, config.max_acc),
                    getattr(config, "shock_limit", config.min_acc)
                )

        a_prev      = float(getattr(car_list[i], 'prev_acc', 0.0))
        a_bar_prev  = float(getattr(car_list[i], 'prev_a_bar', 0.0))
        rho         = float(getattr(config, 'rho_alpha', 0.0))
        acc_sigma = float(getattr(car_list[i], 'acc_noise', 0.0))*0.4
        noise = np.random.normal(0.0, acc_sigma)

        a_exec = a_bar_t + rho * (a_prev - a_bar_prev) + noise
        a_exec = float(np.clip(a_exec, config.min_acc, config.max_acc))
        car_list[i].acc        = a_exec
        car_list[i].prev_acc   = a_exec
        car_list[i].prev_a_bar = float(a_bar_t)

        steer_cmd = _stanley_on_circle(
            car_list[i].state.x, car_list[i].state.y, car_list[i].state.theta, car_list[i].state.v,
            R=config.radius, L=config.car_wheel_base, max_steer=config.max_steer,
            k=2.5, k_soft=0.7, cw=False
        )
        car_list[i].steer = float(np.clip(steer_cmd + car_list[i].epsilon_steer,
                                          -config.max_steer, config.max_steer))

        gt_state = single_agent_state_update(config, car_list[i])
        ev_state = add_evolution_noise(config, gt_state)
        car_list[i].update_state_scalar_act(state=ev_state, ideal_path=ideal_path)
        obs_state = add_observation_noise(config, ev_state)

        log_states.append([
            car_list[i].uid, car_list[i].entrance_id, car_list[i].exit_id,
            car_list[i].state.x, car_list[i].state.y, car_list[i].state.theta, car_list[i].state.v,
            ev_state.x, ev_state.y, ev_state.theta, ev_state.v,
            obs_state.x, obs_state.y, obs_state.theta, obs_state.v,
            car_list[i].prev_acc, car_list[i].prev_steer,
            car_list[i].epsilon_acc, car_list[i].epsilon_steer,
            car_list[i].dimensions.l, car_list[i].dimensions.w,
        ])


    return log_states