import json
import numpy as np

class IdealPath:

    def __init__(self, file_name, config, sample_interval=1):
        self._init_pos_std = config.init_pos_std
        self._init_angle_std = config.init_angle_std
        self._path = self._read_path(file_name, sample_interval)
        self._angle = self._calculate_angle()

        cnt, sum_val = 0, 0
        for key, val in self._path.items():
            cnt += 1
            sum_val += len(val[0])

        avg_pts = int(sum_val / cnt)
        print('average points in path: ', avg_pts)

    @staticmethod
    def wrap_pi(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    def cal_stanley_angle(self,entrance_id, exit_id,
                X, Y, Theta, V, max_steer, k=2.5, k_soft=0.7, cw=False,L=0.288,
                R=3.66):
        r = np.hypot(X, Y)
        e_ct = r - R
        psi_tan = np.arctan2(Y, X) + (-np.pi / 2 if cw else np.pi / 2)
        head_err = self.wrap_pi(psi_tan - Theta)
        st_term = np.arctan2(k * e_ct, abs(V) + k_soft)
        delta_ff = np.arctan(L / R) * (-1 if cw else 1)
        delta = head_err + st_term + delta_ff
        return np.clip(delta, -max_steer, max_steer), (e_ct, head_err, st_term, delta_ff, psi_tan)

    def _read_path(self, file_name, sample_interval):
        with open(file_name, 'r') as f:
            data = json.load(f)

        res = dict()
        for key, val in data.items():
            xs, ys = zip(*val)
            res[key] = (np.array(xs)[::sample_interval], np.array(ys)[::sample_interval])
        return res

    def _calculate_angle(self):

        angle_dict = dict()

        for path_id, (xs, ys) in self._path.items():
            angles = []
            for x, y in zip(xs, ys):
                theta = np.arctan2(y, x) + np.pi / 2

                theta = (theta + np.pi) % (2 * np.pi) - np.pi

                angles.append(theta)

            angle_dict[path_id] = np.array(angles)

        return angle_dict
    
if __name__ == "__main__":
    path = IdealPath('.\circular_trajectory.json')
