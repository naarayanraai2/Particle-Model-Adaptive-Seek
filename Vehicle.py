import numpy as np
from Configuration import config

class State():
    def __init__(self, x, v):
        self.x = x
        self.v = v

class Vehicle():
    def __init__(self, uid, state, acc, opt_acc, prev_state, prev_opt_acc):
        self.uid = uid
        self.front_uid = (self.uid+1) % config['num_vehicles']
        self.state = state
        self.prev_state = None if prev_state is None else prev_state
        self.acc = acc
        self.opt_acc = opt_acc
        self.prev_opt_acc = prev_opt_acc
