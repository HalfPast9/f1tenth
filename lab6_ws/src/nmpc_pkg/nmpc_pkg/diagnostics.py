import time
import json
import os


class DiagnosticLogger:
    def __init__(self, log_dir='/sim_ws/src/nmpc_pkg/nmpc_pkg',
                 auto_save_interval=30):
        self.log_dir = log_dir
        self.entries = []
        self.t0 = time.time()
        self._save_interval = auto_save_interval
        self._since_save = 0

    def log(self, state, u_cmd, u_prev, ref_points, max_speeds_horizon,
            predicted_traj, info):
        self.entries.append({
            't': time.time() - self.t0,
            'px': float(state[0]),
            'py': float(state[1]),
            'phi': float(state[4]),
            'vel': float(state[3]),
            'yaw_rate': float(state[5]),
            'slip': float(state[6]),
            'speed_cmd': float(u_cmd[0]),
            'steer_cmd': float(u_cmd[1]),
            'speed_prev': float(u_prev[0]),
            'steer_prev': float(u_prev[1]),
            'ref_x': [float(r) for r in ref_points[:, 0]],
            'ref_y': [float(r) for r in ref_points[:, 1]],
            'speed_limit': float(max_speeds_horizon[0]) if len(max_speeds_horizon) > 0 else 0.0,
            'pred_x': [float(p) for p in predicted_traj[:, 0]],
            'pred_y': [float(p) for p in predicted_traj[:, 1]],
            'solve_ms': info['time_ms'],
            'status': info['status'],
            'cost': info['cost'],
        })
        self._since_save += 1
        if self._since_save >= self._save_interval:
            self.save()
            self._since_save = 0

    def save(self):
        path = os.path.join(self.log_dir, 'run_log.json')
        with open(path, 'w') as f:
            json.dump(self.entries, f)
        return path
