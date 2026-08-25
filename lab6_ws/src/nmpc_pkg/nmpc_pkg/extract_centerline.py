#!/usr/bin/env python3
"""Snap existing waypoints to corridor centers and resample.

Takes a hand-driven waypoints CSV and a ROS map, snaps each waypoint
to the local maximum of the distance transform (corridor center),
then smooths and resamples at uniform spacing.
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


def world_to_pixel(world_pts, img_height, resolution, origin):
    col = ((world_pts[:, 0] - origin[0]) / resolution).astype(int)
    row = (img_height - 1 - (world_pts[:, 1] - origin[1]) / resolution).astype(int)
    return np.column_stack([row, col])


def pixel_to_world(pixel_pts, img_height, resolution, origin):
    x = pixel_pts[:, 1] * resolution + origin[0]
    y = (img_height - 1 - pixel_pts[:, 0]) * resolution + origin[1]
    return np.column_stack([x, y])


def snap_to_center(pixel_pts, dist_transform, search_radius=15):
    """Move each point to the local maximum of the distance transform."""
    h, w = dist_transform.shape
    snapped = []
    for r, c in pixel_pts:
        r, c = int(r), int(c)
        best_r, best_c = r, c
        best_dist = 0
        for dr in range(-search_radius, search_radius + 1):
            for dc in range(-search_radius, search_radius + 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w:
                    if dist_transform[nr, nc] > best_dist:
                        best_dist = dist_transform[nr, nc]
                        best_r, best_c = nr, nc
        snapped.append([best_r, best_c])
    return np.array(snapped)


def smooth_and_resample(points, d_s=0.1, smooth_window=5):
    if smooth_window > 1 and len(points) > smooth_window:
        kernel = np.ones(smooth_window) / smooth_window
        px = np.convolve(points[:, 0], kernel, mode='valid')
        py = np.convolve(points[:, 1], kernel, mode='valid')
        points = np.column_stack([px, py])

    diffs = np.diff(points, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cum_dist = np.concatenate(([0.0], np.cumsum(seg_lengths)))
    total_length = cum_dist[-1]

    num_samples = int(total_length / d_s)
    if num_samples == 0:
        return points
    sample_dists = np.linspace(0, total_length, num_samples, endpoint=False)

    resampled_x = np.interp(sample_dists, cum_dist, points[:, 0])
    resampled_y = np.interp(sample_dists, cum_dist, points[:, 1])
    return np.column_stack([resampled_x, resampled_y])


def main():
    parser = argparse.ArgumentParser(
        description='Snap waypoints to corridor centers')
    parser.add_argument('map_yaml', help='Path to map .yaml file')
    parser.add_argument('--input', '-i', required=True,
                        help='Input waypoints CSV')
    parser.add_argument('-o', '--output', default='waypoints_centered.csv',
                        help='Output CSV path')
    parser.add_argument('--spacing', type=float, default=0.1,
                        help='Output spacing in meters (default 0.1)')
    parser.add_argument('--search-radius', type=int, default=15,
                        help='Search radius for center snapping in pixels')
    args = parser.parse_args()

    with open(args.map_yaml) as f:
        meta = yaml.safe_load(f)

    map_dir = args.map_yaml.rsplit('/', 1)[0] if '/' in args.map_yaml else '.'
    image_path = f"{map_dir}/{meta['image']}"

    img = np.array(Image.open(image_path).convert('L'))
    resolution = meta['resolution']
    origin = meta['origin'][:2]

    print(f'Map: {image_path} ({img.shape[1]}x{img.shape[0]})')
    print(f'Resolution: {resolution} m/px, Origin: {origin}')

    # Distance transform of free space
    free = (img > 240).astype(np.uint8)
    dist = ndimage.distance_transform_edt(free)
    print(f'Max corridor clearance: {dist.max() * resolution:.2f}m')

    # Load and convert waypoints
    raw_world = load_waypoints(args.input)
    print(f'Input: {len(raw_world)} waypoints')

    pixel_pts = world_to_pixel(raw_world, img.shape[0], resolution, origin)

    # Snap to corridor centers
    snapped = snap_to_center(pixel_pts, dist, args.search_radius)
    snapped_world = pixel_to_world(snapped, img.shape[0], resolution, origin)

    # Close the loop
    snapped_world = np.vstack([snapped_world, snapped_world[0]])

    # Smooth and resample
    final = smooth_and_resample(snapped_world, d_s=args.spacing)
    total_m = len(final) * args.spacing
    loop_gap = np.linalg.norm(final[0] - final[-1])
    print(f'Output: {len(final)} waypoints, track length {total_m:.1f}m')
    print(f'Loop gap: {loop_gap:.2f}m')

    # Average distance from walls
    final_pixels = world_to_pixel(final, img.shape[0], resolution, origin)
    wall_dists = []
    for r, c in final_pixels:
        r, c = int(np.clip(r, 0, img.shape[0]-1)), int(np.clip(c, 0, img.shape[1]-1))
        wall_dists.append(dist[r, c] * resolution)
    print(f'Wall clearance: min={min(wall_dists):.2f}m, '
          f'avg={np.mean(wall_dists):.2f}m, max={max(wall_dists):.2f}m')

    # Compute yaw
    diffs = np.diff(final, axis=0)
    yaws = np.arctan2(diffs[:, 1], diffs[:, 0])
    yaws = np.append(yaws, yaws[-1])

    with open(args.output, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'yaw'])
        for i in range(len(final)):
            writer.writerow([final[i, 0], final[i, 1], yaws[i]])

    print(f'Saved to {args.output}')


if __name__ == '__main__':
    main()
