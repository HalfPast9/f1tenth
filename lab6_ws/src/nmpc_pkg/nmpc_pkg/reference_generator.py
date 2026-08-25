import csv
import numpy as np


class ReferenceGenerator:
    def __init__(self, waypoints_file, d_s=0.1, P=90):
        self.P = P
        raw_xy, raw_speeds = self._load_csv(waypoints_file)
        self.centerline, self.max_speeds = self._resample(raw_xy, raw_speeds, d_s)
        self.headings = self._compute_headings()
        self.K = len(self.centerline)
        self.last_idx = 0

    def _load_csv(self, filepath):
        pts = []
        speeds = []
        with open(filepath, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                pts.append([float(row['x']), float(row['y'])])
                if 'max_speed' in row:
                    speeds.append(float(row['max_speed']))
        pts = np.array(pts)
        if speeds:
            speeds = np.array(speeds)
        else:
            speeds = np.full(len(pts), 8.0)
        return pts, speeds

    def _resample(self, raw, raw_speeds, d_s):
        diffs = np.diff(raw, axis=0)
        seg_lengths = np.linalg.norm(diffs, axis=1)
        cum_dist = np.concatenate(([0.0], np.cumsum(seg_lengths)))
        total_length = cum_dist[-1]

        num_samples = int(total_length / d_s)
        sample_dists = np.linspace(0, total_length, num_samples, endpoint=False)

        resampled_x = np.interp(sample_dists, cum_dist, raw[:, 0])
        resampled_y = np.interp(sample_dists, cum_dist, raw[:, 1])
        resampled_speeds = np.interp(sample_dists, cum_dist, raw_speeds)
        return np.column_stack([resampled_x, resampled_y]), resampled_speeds

    def _compute_headings(self):
        pts = self.centerline
        dx = np.roll(pts[:, 0], -1) - pts[:, 0]
        dy = np.roll(pts[:, 1], -1) - pts[:, 1]
        return np.arctan2(dy, dx)

    def get_reference(self, px, py):
        pos = np.array([px, py])

        window = 200
        indices = np.arange(self.last_idx - 50, self.last_idx + window) % self.K
        candidates = self.centerline[indices]
        dists = np.linalg.norm(candidates - pos, axis=1)
        best_local = np.argmin(dists)
        self.last_idx = indices[best_local]

        lookahead_indices = (self.last_idx + 1 + np.arange(self.P)) % self.K
        return (self.centerline[lookahead_indices],
                self.max_speeds[lookahead_indices],
                self.headings[lookahead_indices])
