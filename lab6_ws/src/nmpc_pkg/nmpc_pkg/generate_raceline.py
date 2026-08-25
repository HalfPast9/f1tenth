#!/usr/bin/env python3
"""Generate a minimum-curvature raceline from centerline + map.

Iteratively smooths the path while constraining points to stay
within track boundaries (walls minus car collision radius).
"""
import argparse
import csv
import yaml
import numpy as np
from PIL import Image
from scipy import ndimage


def load_waypoints(filepath):
    pts = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append([float(row['x']), float(row['y'])])
    return np.array(pts)


def get_wall_clearance(point, dist_transform, img_height, resolution, origin):
    """Get distance to nearest wall for a world-coordinate point."""
    col = int((point[0] - origin[0]) / resolution)
    row = int(img_height - 1 - (point[1] - origin[1]) / resolution)
    h, w = dist_transform.shape
    row = np.clip(row, 0, h - 1)
    col = np.clip(col, 0, w - 1)
    return dist_transform[row, col] * resolution


def smooth_with_boundaries(points, dist_transform, img_height, resolution,
                           origin, car_radius, margin, iterations, alpha):
    """Iteratively smooth path while respecting track boundaries.

    At each iteration, each point moves toward the average of its neighbors
    (minimizing curvature). Then it's pulled back if too close to a wall.
    """
    path = points.copy()
    n = len(path)

    for it in range(iterations):
        new_path = path.copy()
        for i in range(n):
            prev_idx = (i - 1) % n
            next_idx = (i + 1) % n

            # Move toward neighbor average (smoothing)
            midpoint = (path[prev_idx] + path[next_idx]) / 2.0
            candidate = path[i] + alpha * (midpoint - path[i])

            # Check wall clearance at candidate position
            clearance = get_wall_clearance(
                candidate, dist_transform, img_height, resolution, origin)
            min_clearance = car_radius + margin

            if clearance >= min_clearance:
                new_path[i] = candidate
            else:
                # Try a smaller step
                for shrink in [0.5, 0.25, 0.1]:
                    fallback = path[i] + alpha * shrink * (midpoint - path[i])
                    fb_clearance = get_wall_clearance(
                        fallback, dist_transform, img_height, resolution, origin)
                    if fb_clearance >= min_clearance:
                        new_path[i] = fallback
                        break

        path = new_path

        if (it + 1) % 100 == 0:
            curvatures = compute_curvature(path)
            max_curv = np.max(np.abs(curvatures))
            print(f'  iter {it+1}: max curvature = {max_curv:.3f} 1/m, '
                  f'min turn radius = {1/max(max_curv, 0.01):.2f}m')

    return path


def compute_curvature(points):
    """Compute signed curvature at each point."""
    n = len(points)
    curvatures = np.zeros(n)
    for i in range(n):
        p0 = points[(i - 1) % n]
        p1 = points[i]
        p2 = points[(i + 1) % n]
        d1 = p1 - p0
        d2 = p2 - p1
        ds = np.linalg.norm(d1)
        if ds < 1e-6:
            continue
        dyaw = np.arctan2(d2[1], d2[0]) - np.arctan2(d1[1], d1[0])
        dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))
        curvatures[i] = dyaw / ds
    return curvatures


def compute_max_speed(curvatures, d_s=0.1, mu=1.0489, g=9.81, speed_cap=8.0,
                      a_brake=5.0, a_accel=3.0):
    """Max speed profile with braking and acceleration awareness.

    Pass 1: cornering limit from curvature.
    Pass 2 (backward): v[i]^2 <= v[i+1]^2 + 2*a_brake*d_s (can't brake faster).
    Pass 3 (forward): v[i+1]^2 <= v[i]^2 + 2*a_accel*d_s (can't accelerate faster).
    """
    n = len(curvatures)
    speeds = np.zeros(n)
    for i, k in enumerate(curvatures):
        if abs(k) < 0.01:
            speeds[i] = speed_cap
        else:
            speeds[i] = min(np.sqrt(mu * g / abs(k)), speed_cap)

    # Backward pass (braking into corners) — run twice for wrap-around
    for _ in range(2):
        for i in range(n - 2, -1, -1):
            j = (i + 1) % n
            v_max = np.sqrt(speeds[j]**2 + 2 * a_brake * d_s)
            speeds[i] = min(speeds[i], v_max)
        v_max = np.sqrt(speeds[0]**2 + 2 * a_brake * d_s)
        speeds[-1] = min(speeds[-1], v_max)

    # Forward pass (acceleration out of corners) — run twice for wrap-around
    for _ in range(2):
        for i in range(n - 1):
            j = (i + 1) % n
            v_max = np.sqrt(speeds[i]**2 + 2 * a_accel * d_s)
            speeds[j] = min(speeds[j], v_max)
        v_max = np.sqrt(speeds[-1]**2 + 2 * a_accel * d_s)
        speeds[0] = min(speeds[0], v_max)

    return speeds


def resample(points, d_s):
    diffs = np.diff(points, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cum_dist = np.concatenate(([0.0], np.cumsum(seg_lengths)))
    total_length = cum_dist[-1]
    num_samples = int(total_length / d_s)
    sample_dists = np.linspace(0, total_length, num_samples, endpoint=False)
    rx = np.interp(sample_dists, cum_dist, points[:, 0])
    ry = np.interp(sample_dists, cum_dist, points[:, 1])
    return np.column_stack([rx, ry])


def main():
    parser = argparse.ArgumentParser(description='Generate raceline')
    parser.add_argument('map_yaml', help='Path to map .yaml')
    parser.add_argument('--input', '-i', required=True, help='Centerline CSV')
    parser.add_argument('-o', '--output', default='raceline.csv')
    parser.add_argument('--spacing', type=float, default=0.1)
    parser.add_argument('--car-radius', type=float, default=0.24,
                        help='Car collision radius [m]')
    parser.add_argument('--margin', type=float, default=0.05,
                        help='Extra safety margin from walls [m]')
    parser.add_argument('--iterations', type=int, default=500,
                        help='Smoothing iterations')
    parser.add_argument('--alpha', type=float, default=0.4,
                        help='Smoothing step size (0-1)')
    parser.add_argument('--speed-cap', type=float, default=8.0)
    args = parser.parse_args()

    with open(args.map_yaml) as f:
        meta = yaml.safe_load(f)

    map_dir = args.map_yaml.rsplit('/', 1)[0] if '/' in args.map_yaml else '.'
    image_path = f"{map_dir}/{meta['image']}"
    img = np.array(Image.open(image_path).convert('L'))
    resolution = meta['resolution']
    origin = meta['origin'][:2]

    free = (img > 240).astype(np.uint8)
    dist = ndimage.distance_transform_edt(free)

    centerline = load_waypoints(args.input)
    print(f'Input: {len(centerline)} centerline points')

    print(f'Optimizing raceline ({args.iterations} iterations)...')
    raceline = smooth_with_boundaries(
        centerline, dist, img.shape[0], resolution, origin,
        args.car_radius, args.margin, args.iterations, args.alpha)

    # Close loop and resample
    raceline = np.vstack([raceline, raceline[0]])
    raceline = resample(raceline, args.spacing)

    # Compute curvature and max speeds
    curvatures = compute_curvature(raceline)
    max_speeds = compute_max_speed(curvatures, speed_cap=args.speed_cap)

    # Check wall clearances
    clearances = []
    for p in raceline:
        clearances.append(get_wall_clearance(
            p, dist, img.shape[0], resolution, origin))
    clearances = np.array(clearances)

    total_m = len(raceline) * args.spacing
    print(f'Output: {len(raceline)} points, {total_m:.1f}m')
    print(f'Wall clearance: min={clearances.min():.2f}m, avg={clearances.mean():.2f}m')
    print(f'Max curvature: {np.max(np.abs(curvatures)):.3f} 1/m')
    print(f'Min turn radius: {1/max(np.max(np.abs(curvatures)), 0.01):.2f}m')
    print(f'Speed range: {max_speeds.min():.1f} - {max_speeds.max():.1f} m/s')

    # Compute yaw
    diffs = np.diff(raceline, axis=0)
    yaws = np.arctan2(diffs[:, 1], diffs[:, 0])
    yaws = np.append(yaws, yaws[-1])

    with open(args.output, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'yaw', 'max_speed'])
        for i in range(len(raceline)):
            writer.writerow([raceline[i, 0], raceline[i, 1],
                             yaws[i], max_speeds[i]])

    print(f'Saved to {args.output}')


if __name__ == '__main__':
    main()
