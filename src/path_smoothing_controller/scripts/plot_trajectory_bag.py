#!/usr/bin/env python3
"""Create report plots and cross-track metrics from a recorded custom run."""

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def path_arrays(path_message):
    poses = path_message.poses
    xy = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in poses])
    stamps = np.array([stamp_seconds(pose.header.stamp) for pose in poses])
    return xy, stamps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='rosbag2 directory created by ros2 bag record')
    parser.add_argument('--output', type=Path, default=Path('trajectory_report'))
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='',
                                    output_serialization_format=''),
    )
    type_by_topic = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    needed = {
        '/nav2_planned_path', '/smoothed_path', '/trajectory', '/actual_path'
    }
    optional = {'/user_waypoints'}
    serialized = {topic: [] for topic in needed | optional}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in serialized:
            serialized[topic].append(data)

    missing = {topic for topic in needed if not serialized[topic]}
    if missing:
        raise RuntimeError('Bag is missing required topics: ' + ', '.join(sorted(missing)))

    messages = {
        topic: deserialize_message(max(data_list, key=len), get_message(type_by_topic[topic]))
        for topic, data_list in serialized.items() if data_list
    }
    trajectory = messages['/trajectory']
    planned = messages['/nav2_planned_path']
    smoothed = messages['/smoothed_path']
    actual = messages['/actual_path']
    user_waypoints = None
    if serialized['/user_waypoints']:
        user_waypoints = deserialize_message(
            max(serialized['/user_waypoints'], key=len),
            get_message(type_by_topic['/user_waypoints']))
    planned_xy, _ = path_arrays(planned)
    smoothed_xy, _ = path_arrays(smoothed)
    actual_xy, actual_stamps = path_arrays(actual)
    trajectory_start_time = stamp_seconds(trajectory.header.stamp)
    trajectory_duration = max(
        float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1.0e-9
        for point in trajectory.points
    )
    actual_mask = (
        (actual_stamps >= trajectory_start_time) &
        (actual_stamps <= trajectory_start_time + max(10.0, 3.0 * trajectory_duration))
    )
    actual_xy = actual_xy[actual_mask]
    actual_stamps = actual_stamps[actual_mask]
    trajectory_xy = np.array([[point.x, point.y] for point in trajectory.points])
    trajectory_time = np.array([
        float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1.0e-9
        for point in trajectory.points
    ])
    linear_velocity = np.array([point.linear_velocity for point in trajectory.points])
    angular_velocity = np.array([point.angular_velocity for point in trajectory.points])
    heading = np.array([point.heading for point in trajectory.points])

    time_steps = np.diff(trajectory_time)
    if not np.all(time_steps > 0.0):
        raise RuntimeError('Trajectory timestamps are not strictly increasing')
    linear_acceleration = np.diff(linear_velocity) / time_steps
    max_linear_acceleration = float(np.abs(linear_acceleration).max()) if len(
        linear_acceleration) else 0.0
    if not np.all(np.isfinite(heading)):
        raise RuntimeError('Trajectory contains a non-finite heading')

    if not actual_xy.size or not trajectory_xy.size:
        raise RuntimeError('Actual path or trajectory contains no samples')
    distances = np.linalg.norm(
        actual_xy[:, None, :] - trajectory_xy[None, :, :], axis=2)
    cross_track = distances.min(axis=1)
    final_error = float(np.linalg.norm(actual_xy[-1] - trajectory_xy[-1]))
    rms_error = float(math.sqrt(float(np.mean(cross_track ** 2))))
    max_error = float(cross_track.max())

    actual_time = actual_stamps - actual_stamps[0]
    args.output.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)

    axes[0, 0].plot(planned_xy[:, 0], planned_xy[:, 1], '--', label='Nav2 path')
    axes[0, 0].plot(smoothed_xy[:, 0], smoothed_xy[:, 1], label='Smoothed')
    axes[0, 0].plot(trajectory_xy[:, 0], trajectory_xy[:, 1], ':', label='Trajectory')
    axes[0, 0].plot(actual_xy[:, 0], actual_xy[:, 1], label='Actual')
    if user_waypoints is not None:
        waypoint_xy, _ = path_arrays(user_waypoints)
        axes[0, 0].plot(
            waypoint_xy[:, 0], waypoint_xy[:, 1], 'o-', color='gold',
            markersize=6, label='User waypoints')
        for index, (x_value, y_value) in enumerate(waypoint_xy):
            axes[0, 0].annotate(
                str(index + 1), (x_value, y_value), xytext=(4, 4),
                textcoords='offset points', fontsize=8)
    axes[0, 0].set(title='XY path overlay', xlabel='x [m]', ylabel='y [m]')
    axes[0, 0].axis('equal')
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    axes[0, 1].plot(trajectory_time, linear_velocity)
    axes[0, 1].set(title='Linear velocity profile', xlabel='time [s]', ylabel='v [m/s]')
    axes[0, 1].grid(True)

    axes[1, 0].plot(trajectory_time, angular_velocity)
    axes[1, 0].set(title='Angular velocity profile', xlabel='time [s]', ylabel='ω [rad/s]')
    axes[1, 0].grid(True)

    axes[1, 1].plot(actual_time, cross_track)
    axes[1, 1].set(title='Measured cross-track error', xlabel='time [s]', ylabel='error [m]')
    axes[1, 1].grid(True)

    image_path = args.output / 'trajectory_report.png'
    figure.savefig(image_path, dpi=180)
    metrics_path = args.output / 'metrics.txt'
    metrics_path.write_text(
        f'RMS cross-track error: {rms_error:.4f} m\n'
        f'Max cross-track error: {max_error:.4f} m\n'
        f'Final sampled position error: {final_error:.4f} m\n'
        f'Trajectory duration: {trajectory_time[-1]:.4f} s\n'
        f'Observed track duration: {actual_time[-1]:.4f} s\n'
        f'Max linear speed: {linear_velocity.max():.4f} m/s\n'
        f'Max angular speed: {np.abs(angular_velocity).max():.4f} rad/s\n'
        f'Max sampled linear acceleration: {max_linear_acceleration:.4f} m/s^2\n'
        f'Start/end linear velocity: {linear_velocity[0]:.4f}, {linear_velocity[-1]:.4f} m/s\n'
        f'Start/end angular velocity: {angular_velocity[0]:.4f}, '
        f'{angular_velocity[-1]:.4f} rad/s\n'
        f'Timestamps strictly increasing: yes ({len(trajectory_time)} samples)\n'
        f'Headings finite: yes\n'
        f'User waypoint count: {len(user_waypoints.poses) if user_waypoints is not None else 0}\n',
        encoding='utf-8',
    )
    print(metrics_path.read_text(encoding='utf-8'), end='')
    print(f'Saved plot: {image_path}')


if __name__ == '__main__':
    main()
