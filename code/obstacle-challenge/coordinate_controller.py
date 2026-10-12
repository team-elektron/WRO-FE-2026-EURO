"""Calibrated local-coordinate navigation. See COORDINATE-README.md.

Frame: origin at rear axle centre, x right, y forward, heading clockwise.
Uses zone_controller only for the existing UART, colour and camera settings.
No legacy obstacle controller, turn controller or fixed dropout travel is used.
"""
import argparse
import copy
import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import zone_controller as hardware


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, s], [-s, c]])


def local(point, origin, heading):
    return rotation(heading).T @ (np.asarray(point) - origin)


def positive(value, name):
    if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be a measured positive number')
    return float(value)


DEFAULTS = {
    'schema': 1,
    'image_size': [640, 480],
    'raw_size': [3280, 2464],
    'camera_mount_notes': 'Height about 160 mm; tilt about 12.5-17 degrees downward. Floor grid supplies the mapping.',
    'wheelbase_mm': 160.0,
    'width_mm': 135.0,
    # Conservative bounds from 160 mm wheelbase and <=210 mm overall length.
    # Replace with measured extents to reduce unnecessary rejected paths.
    'front_from_rear_axle_mm': 210.0,
    'rear_overhang_mm': 50.0,
    'left_tof_origin_mm': None,
    'right_tof_origin_mm': None,
    'front_tof_origin_mm': None,
    'rear_tof_origin_mm': None,
    'ticks_per_mm': 2.87,
    'steering_table': [],  # [servo command, measured physical wheel angle: right positive]
    'ground_pixels': [],
    'ground_mm': [],
    'ground_triangles': [],
    'ground_quads': [],
    'validation_pixels': [],
    'validation_mm': [],
    'max_calibration_error_mm': 20.0,
    'pillar_ignore_top_fraction': 0.40,
    # Ignore the robot's own screen/PCB in the bottom centre of the image.
    'bottom_center_ignore_width_fraction': 0.30,
    'bottom_center_ignore_top_fraction': 0.78,
    'unmapped_close_bottom_fraction': 0.72,
    'unmapped_confirm_seconds': 0.35,
    'line_roi_top': 0.75,
    'line_trigger_pct': 0.05,
    'clearance_mm': 35.0,
    'gate_clearance_mm': 15.0,
    # Clearance from a measured ToF wall hit, separate from pillar clearance.
    'wall_clearance_mm': 25.0,
    'odometry_margin_mm': 40.0,
    # A visible pillar is camera-anchored; only add the full odometry allowance
    # after it has remained unseen for this long.
    'track_uncertainty_seconds': 1.0,
    'tof_pillar_reject_mm': 120.0,
    'target_switch_margin_mm': 120.0,
    'pillar_width_mm': 50.0,
    'track_match_mm': 200.0,
    'lookahead_mm': 450.0,
    'horizon_mm': 800.0,
    'path_step_mm': 25.0,
    'wall_distance_mm': 160.0,
    'wall_gain': 0.7,
    'wall_stop_mm': 60.0,
    'wall_detection_inset_mm': 15.0,
    'wall_model_min_samples': 4,
    'wall_model_outlier_mm': 100.0,
    'front_stop_mm': 140.0,
    'rear_stop_mm': 90.0,
    'motor_speed': 100,
    'turn_speed': 90,
    'recovery_reverse_mm': 100.0,
    'recovery_speed': 70,
    'recovery_max_attempts': 2,
    'recovery_reset_forward_mm': 250.0,
    'no_path_confirm_seconds': 0.25,
    'obstacle_start_extra_mm': 300.0,
    'recovery_min_forward_mm': 80.0,
    'recovery_replan_pause_seconds': 0.40,
    'steering_change_penalty': 0.10,
    'servo_rate_per_second': 90.0,
    'max_speed_mm_s': 1200.0,
    'encoder_stall_seconds': 2.0,
    # The RP2350 nominally reports every 100 ms, but observed Pi scheduling can
    # delay a valid parsed sample beyond the previous 350 ms hard limit.
    'telemetry_timeout_seconds': 0.60,
    'max_frame_seconds': 0.25,
    'slow_cycle_count': 5,
    # Must remain below the RP2350 firmware's independent 1.0 s link timeout.
    'command_watchdog_seconds': 0.70,
    'turn_timeout_seconds': 8.0,
    'finish_distance_mm': 200.0,
    'camera_controls': {'AeEnable': True, 'AwbEnable': True, 'AwbMode': 0,
                        'ExposureValue': 0.0, 'Saturation': 1.4, 'Sharpness': 1.5},
}

# Only measured fields belong in a fresh user template. Tuning overrides may be
# added explicitly; DEFAULTS is the single source for untuned values.
CALIBRATION_FIELDS = (
    'wheelbase_mm', 'width_mm', 'front_from_rear_axle_mm', 'rear_overhang_mm',
    'left_tof_origin_mm', 'right_tof_origin_mm', 'front_tof_origin_mm',
    'rear_tof_origin_mm', 'ticks_per_mm', 'steering_table', 'ground_pixels',
    'ground_mm', 'ground_triangles', 'ground_quads', 'validation_pixels',
    'validation_mm',
)


def config_template():
    return {key: copy.deepcopy(DEFAULTS[key]) for key in CALIBRATION_FIELDS}


class GroundMap:
    """Measured local-perspective floor map, no extrapolation or assumed lens FOV.

    Four-corner perspective cells model floor perspective; a dense grid also
    accommodates lens distortion locally. Independent intermediate
    check points are mandatory. This is an estimate on the calibrated floor.
    """
    def __init__(self, cfg):
        self.size = tuple(cfg['image_size'])
        pixels = np.asarray(cfg['ground_pixels'], dtype=float)
        ground = np.asarray(cfg['ground_mm'], dtype=float)
        if pixels.ndim != 2 or pixels.shape[1:] != (2,) or len(pixels) < 9:
            raise ValueError('Calibrate at least a 3x3 measured floor grid first')
        if pixels.shape != ground.shape or not np.isfinite([pixels, ground]).all():
            raise ValueError('Invalid floor calibration coordinates')
        if (pixels < 0).any() or (pixels >= np.asarray(self.size)).any():
            raise ValueError('Calibration pixels outside configured image')
        self.cells = []
        self.triangles = []
        quads = cfg.get('ground_quads', [])
        for indexes in quads:
            if len(indexes) != 4 or any(not isinstance(i, int) or i < 0 or i >= len(pixels) for i in indexes):
                raise ValueError('Invalid calibration grid cell')
            p, g = pixels[indexes].astype(np.float32), ground[indexes].astype(np.float32)
            if not cv2.isContourConvex(p) or not cv2.isContourConvex(g):
                raise ValueError('Folded calibration cell: check click order')
            image_to_ground = cv2.getPerspectiveTransform(p, g)
            if not np.isfinite(image_to_ground).all() or abs(np.linalg.det(image_to_ground)) < 1e-12:
                raise ValueError('Degenerate calibration cell: check clicks')
            self.cells.append((p, g, image_to_ground, np.linalg.inv(image_to_ground)))
        orientation = None
        for indexes in cfg['ground_triangles']:
            if len(indexes) != 3 or any(not isinstance(i, int) or i < 0 or i >= len(pixels) for i in indexes):
                raise ValueError('Invalid calibration triangle')
            p, g = pixels[indexes], ground[indexes]
            matrix = np.vstack((p.T, np.ones(3)))
            dg = np.linalg.det(np.vstack((g.T, np.ones(3))))
            dp = np.linalg.det(matrix)
            if abs(dp) < 1 or abs(dg) < 1:
                raise ValueError('Degenerate floor triangle: check click order')
            sign = np.sign(dp * dg)
            if orientation is not None and sign != orientation:
                raise ValueError('Folded calibration grid: check click order')
            orientation = sign
            self.triangles.append((np.linalg.inv(matrix), g))
        if self.cells and len(self.cells) < 4:
            raise ValueError('Incomplete perspective calibration grid')
        if not self.cells and len(self.triangles) < 8:
            raise ValueError('Incomplete calibration grid')
        checks = cfg['validation_pixels']
        expected = cfg['validation_mm']
        if len(checks) < 4 or len(checks) != len(expected):
            raise ValueError('At least four independent measured check points are required')
        errors = []
        details = []
        for index, (pixel, actual) in enumerate(zip(checks, expected), 1):
            estimate = self.project(pixel)
            if estimate is None:
                raise ValueError('Validation point outside calibrated floor grid')
            error = np.linalg.norm(estimate - np.asarray(actual, dtype=float))
            if not math.isfinite(error):
                raise ValueError('Nonfinite calibration check')
            errors.append(error)
            details.append((error, index, actual, estimate, pixel))
        self.error_mm = max(errors)
        self.validation_details = sorted(details, reverse=True)
        if self.error_mm > positive(cfg['max_calibration_error_mm'], 'max_calibration_error_mm'):
            error, index, actual, estimate, pixel = self.validation_details[0]
            raise ValueError(
                f'Floor check {index} error {error:.1f} mm at measured '
                f'({actual[0]:g}, {actual[1]:g}) mm; map estimated '
                f'({estimate[0]:.1f}, {estimate[1]:.1f}) from clicked pixel '
                f'({pixel[0]:g}, {pixel[1]:g}). Recheck that physical mark and '
                'the surrounding four grid clicks; do not raise the error limit.')

    def project(self, pixel):
        if len(pixel) != 2 or not np.isfinite(pixel).all():
            return None
        if self.cells:
            point = tuple(float(x) for x in pixel)
            for image, _, transform, _ in self.cells:
                if cv2.pointPolygonTest(image, point, False) >= 0:
                    result = cv2.perspectiveTransform(
                        np.asarray([[point]], np.float32), transform)[0, 0]
                    return result.astype(float) if np.isfinite(result).all() else None
            return None
        v = np.array([*pixel, 1.0])
        for inv, ground in self.triangles:
            weights = inv @ v
            if np.min(weights) >= -1e-7 and np.max(weights) <= 1 + 1e-7:
                return weights @ ground
        return None

    def image_point(self, point):
        """Inverse within the measured grid, for associating partial detections."""
        if self.cells:
            point = tuple(float(x) for x in point)
            for _, ground, _, inverse in self.cells:
                if cv2.pointPolygonTest(ground, point, False) >= 0:
                    result = cv2.perspectiveTransform(
                        np.asarray([[point]], np.float32), inverse)[0, 0]
                    return result.astype(float) if np.isfinite(result).all() else None
            return None
        for inv, ground in self.triangles:
            weights = np.linalg.solve(np.vstack((ground.T, np.ones(3))), [*point, 1])
            if weights.min() >= -1e-7 and weights.max() <= 1 + 1e-7:
                return (np.linalg.inv(inv) @ weights)[:2]
        return None


def load_config(path):
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError('Config must be a JSON object')
    # Older calibrated files may contain this retired setting. It has no effect.
    data.pop('recovery_steering_fraction', None)
    unknown = data.keys() - DEFAULTS.keys()
    if unknown:
        raise ValueError('Unknown config keys: ' + ', '.join(sorted(unknown)))
    cfg = {**copy.deepcopy(DEFAULTS), **data}
    if cfg['schema'] != 1:
        raise ValueError('Unsupported config schema; expected 1')
    for key in ('wheelbase_mm', 'width_mm', 'front_from_rear_axle_mm',
                'rear_overhang_mm', 'ticks_per_mm', 'clearance_mm', 'gate_clearance_mm',
                'wall_clearance_mm',
                'odometry_margin_mm', 'pillar_width_mm',
                'track_uncertainty_seconds',
                'tof_pillar_reject_mm', 'target_switch_margin_mm',
                'track_match_mm',
                'lookahead_mm', 'horizon_mm', 'path_step_mm', 'wall_distance_mm',
                'wall_gain', 'wall_stop_mm', 'front_stop_mm', 'rear_stop_mm',
                'wall_detection_inset_mm', 'wall_model_outlier_mm',
                'servo_rate_per_second', 'max_speed_mm_s', 'encoder_stall_seconds',
                'telemetry_timeout_seconds',
                'max_frame_seconds', 'command_watchdog_seconds',
                'turn_timeout_seconds', 'finish_distance_mm',
                'unmapped_confirm_seconds', 'recovery_reverse_mm',
                'recovery_reset_forward_mm', 'no_path_confirm_seconds',
                'obstacle_start_extra_mm', 'recovery_min_forward_mm',
                'recovery_replan_pause_seconds'):
        positive(cfg[key], key)
    if cfg['front_from_rear_axle_mm'] < cfg['wheelbase_mm']:
        raise ValueError('Front footprint cannot be behind the front axle')
    if cfg['path_step_mm'] > 25 or cfg['horizon_mm'] < 2 * cfg['lookahead_mm'] / 3:
        raise ValueError('Use path steps <=25 mm and sufficient horizon')
    if cfg['horizon_mm'] > 1500 or cfg['horizon_mm'] / cfg['path_step_mm'] > 100:
        raise ValueError('Excessive trajectory workload')
    for key in ('motor_speed', 'turn_speed', 'recovery_speed'):
        if not isinstance(cfg[key], int) or not 1 <= cfg[key] <= 255:
            raise ValueError(f'{key} must be 1..255')
    if (not isinstance(cfg['recovery_max_attempts'], int) or
            not 0 <= cfg['recovery_max_attempts'] <= 5):
        raise ValueError('recovery_max_attempts must be an integer from 0 to 5')
    if (not isinstance(cfg['wall_model_min_samples'], int) or
            not 2 <= cfg['wall_model_min_samples'] <= 20):
        raise ValueError('wall_model_min_samples must be an integer from 2 to 20')
    if (not isinstance(cfg['steering_change_penalty'], (float, int)) or
            not 0 <= cfg['steering_change_penalty'] <= 2):
        raise ValueError('steering_change_penalty must be from 0 to 2')
    if cfg['front_stop_mm'] >= 150:
        raise ValueError('front_stop_mm must stay below the firmware 150 mm cap')
    if cfg['rear_stop_mm'] >= 150:
        raise ValueError('rear_stop_mm must stay below the firmware 150 mm cap')
    for key in ('pillar_ignore_top_fraction', 'bottom_center_ignore_top_fraction',
                'unmapped_close_bottom_fraction', 'line_roi_top', 'line_trigger_pct'):
        if not isinstance(cfg[key], (float, int)) or not 0 < cfg[key] < 1:
            raise ValueError(f'{key} must be between 0 and 1')
    width = cfg['bottom_center_ignore_width_fraction']
    if not isinstance(width, (float, int)) or not 0 <= width < 1:
        raise ValueError('bottom_center_ignore_width_fraction must be from 0 up to (but not including) 1')
    if not isinstance(cfg['slow_cycle_count'], int) or not 2 <= cfg['slow_cycle_count'] <= 20:
        raise ValueError('slow_cycle_count must be an integer from 2 to 20')
    if not 0.35 <= cfg['command_watchdog_seconds'] < 0.90:
        raise ValueError('command_watchdog_seconds must be from 0.35 to under 0.90 seconds')
    if not 0.35 <= cfg['telemetry_timeout_seconds'] <= 1.0:
        raise ValueError('telemetry_timeout_seconds must be from 0.35 to 1.0 seconds')
    for key, sign in (('left_tof_origin_mm', -1), ('right_tof_origin_mm', 1)):
        origin = np.asarray(cfg[key], dtype=float)
        if origin.shape != (2,) or not np.isfinite(origin).all() or origin[0] * sign <= 0:
            raise ValueError(f'Measure {key} [x right, y forward] from rear axle')
    if cfg['front_tof_origin_mm'] is None:
        cfg['front_tof_origin_mm'] = [0.0, cfg['front_from_rear_axle_mm']]
    if cfg['rear_tof_origin_mm'] is None:
        cfg['rear_tof_origin_mm'] = [0.0, -cfg['rear_overhang_mm']]
    for key in ('front_tof_origin_mm', 'rear_tof_origin_mm'):
        origin = np.asarray(cfg[key], dtype=float)
        if origin.shape != (2,) or not np.isfinite(origin).all():
            raise ValueError(f'Measure {key} [x right, y forward] from rear axle')
    table = np.asarray(cfg['steering_table'], dtype=float)
    if table.ndim != 2 or table.shape[1:] != (2,) or len(table) < 5 or not np.isfinite(table).all():
        raise ValueError('Measure steering_table: at least five [servo, wheel degrees right-positive] pairs')
    table = table[np.argsort(table[:, 0])]
    if (np.diff(table[:, 0]) <= 0).any() or (table[:, 0] < 0).any() or (table[:, 0] > 180).any():
        raise ValueError('Servo table commands must be distinct and within 0..180')
    differences = np.diff(table[:, 1])
    if not ((differences > 0).all() or (differences < 0).all()):
        raise ValueError('Measured steering angles must be monotonic')
    if table[:, 1].min() >= -5 or table[:, 1].max() <= 5 or np.abs(table[:, 1]).max() > 60:
        raise ValueError('Steering table must cover measured left, straight and right')
    cfg['steering_table'] = table.tolist()
    ground = GroundMap(cfg)
    return cfg, ground


class Pose:
    def __init__(self, cfg):
        self.cfg = cfg
        self.xy = np.zeros(2)
        self.heading = 0.0
        self.encoder = self.yaw = self.stamp = None
        self.travel = 0.0
        self.speed = 0.0

    def update(self, telemetry, now, motion_sign=1):
        stamp = telemetry['_sample_time']
        yaw = float(telemetry['yaw'])
        age = now - stamp
        if not math.isfinite(yaw):
            raise ValueError(f'Telemetry yaw invalid: {yaw!r}')
        if age < 0:
            raise ValueError(f'Telemetry timestamp is {abs(age)*1000:.0f} ms in the future')
        if age > self.cfg['telemetry_timeout_seconds']:
            raise ValueError(
                f'Telemetry stale: age={age*1000:.0f} ms, limit='
                f'{self.cfg["telemetry_timeout_seconds"]*1000:.0f} ms, '
                f'packets={telemetry.get("_packet_count", "?")}, '
                f'crc_errors={telemetry.get("_crc_errors", "?")}, '
                f'raw_bytes={telemetry.get("_raw_bytes", "?")}')
        encoder = int(telemetry['enc']) & 0xffff
        if self.stamp is not None and stamp <= self.stamp:
            return 0.0
        distance = 0.0
        if self.stamp is not None:
            dt = stamp - self.stamp
            measured = ((encoder - self.encoder) & 0xffff) / self.cfg['ticks_per_mm']
            if measured > self.cfg['max_speed_mm_s'] * dt + 10:
                raise ValueError('Encoder reset/jump or implausible speed')
            distance = measured * motion_sign
            self.speed = measured / dt
            change = math.radians(hardware.yaw_error(yaw, self.yaw) / hardware.YAW_SIGN)
            if abs(change) > math.radians(360) * dt + 0.05:
                raise ValueError('IMU heading jump')
            self.xy += rotation(self.heading + change / 2) @ np.array([0.0, distance])
            self.heading = wrap(self.heading + change)
            self.travel += distance
        self.encoder, self.yaw, self.stamp = encoder, yaw, stamp
        return distance


@dataclass
class Track:
    ident: int
    color: str
    point: np.ndarray
    last_seen: float
    # Last observed bottom-centre pixel. This lets a pillar keep its mapped
    # world position when its base briefly leaves the calibrated floor polygon.
    last_pixel: np.ndarray | None = None
    pass_heading: float | None = None
    passed: bool = False
    deferred: bool = False


class World:
    def __init__(self, cfg, mapper):
        self.cfg, self.mapper = cfg, mapper
        self.tracks = []
        self.next_id = 1
        self.wall_points = []
        self.unmapped = 0
        self.unmapped_close = 0
        self.unmapped_detail = None
        self.unknown_close_since = None
        self.unknown_close_signature = None
        self.wall_models = {index: {'left': [], 'right': []} for index in range(4)}
        self.active_wall_heading = None
        self.active_wall_bounds = (None, None)
        self.environment_rejected = 0
        self.pillar_tof_sides = set()

    @staticmethod
    def _segment(heading):
        return int(round(heading / (math.pi / 2))) % 4

    def _bounds_for_heading(self, heading):
        if heading is None:
            return None, None
        model = self.wall_models[self._segment(heading)]
        minimum = self.cfg['wall_model_min_samples']
        def robust(samples):
            if len(samples) < minimum:
                return None
            values = [cross for cross, _ in samples]
            median = float(np.median(values))
            inliers = [(cross, along) for cross, along in samples
                       if abs(cross - median) <= self.cfg['wall_model_outlier_mm']]
            if len(inliers) < minimum or max(y for _, y in inliers) - min(y for _, y in inliers) < 150:
                return None
            return float(np.median([x for x, _ in inliers]))

        left, right = robust(model['left']), robust(model['right'])
        if left is not None and right is not None and left >= right:
            return None, None
        return left, right

    def corridor_bounds(self, route):
        return self._bounds_for_heading(route.heading if route is not None else None)

    def _record_wall(self, side, point, route):
        if route is None or route.turning:
            return
        if abs(wrap(route.heading - self._pose_heading)) > math.radians(12):
            return
        value, along = (rotation(route.heading).T @ point)
        value, along = float(value), float(along)
        samples = self.wall_models[self._segment(route.heading)][side]
        if samples and abs(along - samples[-1][1]) < 30:
            return
        established = self._bounds_for_heading(route.heading)[0 if side == 'left' else 1]
        if (established is not None and
                abs(value - established) > self.cfg['wall_model_outlier_mm']):
            return
        samples.append((value, along))
        del samples[:-50]

    def _add_wall_point(self, point, now):
        if not self.wall_points or all(np.linalg.norm(point - p) > 15
                                       for _, p in self.wall_points[-20:]):
            self.wall_points.append((now, point))

    def _update_walls(self, pose, telemetry, now, route):
        self._pose_heading = pose.heading
        self._pose_xy = pose.xy.copy()
        self.pillar_tof_sides = set()
        for side, sign, index in (('left', -1, 1), ('right', 1, 2)):
            distance = telemetry['tof'][index]
            if 0 < distance < 600:
                origin = np.asarray(self.cfg[f'{side}_tof_origin_mm'])
                hit = pose.xy + rotation(pose.heading) @ (origin + [sign * distance, 0])
                if any(np.linalg.norm(hit - track.point) <= self.cfg['tof_pillar_reject_mm']
                       for track in self.tracks):
                    self.pillar_tof_sides.add(side)
                    continue
                self._add_wall_point(hit, now)
                self._record_wall(side, hit, route)
        front = telemetry['tof'][0]
        if 0 < front < 150:
            origin = np.asarray(self.cfg.get('front_tof_origin_mm') or
                                [0.0, self.cfg['front_from_rear_axle_mm']])
            hit = pose.xy + rotation(pose.heading) @ (origin + [0, front])
            self._add_wall_point(hit, now)
        rear = telemetry.get('rear_mm', 9999)
        # This firmware reports 150 mm when the rear sensor sees nothing in its
        # usable range. Equality is a capped/far value, not a wall hit.
        if 0 < rear < 150:
            origin = np.asarray(self.cfg.get('rear_tof_origin_mm') or
                                [0.0, -self.cfg['rear_overhang_mm']])
            hit = pose.xy + rotation(pose.heading) @ (origin + [0, -rear])
            self._add_wall_point(hit, now)
        self.wall_points = [(t, p) for t, p in self.wall_points if now - t < 1.5][-100:]
        self.active_wall_heading = route.heading if route is not None else None
        self.active_wall_bounds = self.corridor_bounds(route)

    def _inside_corridor(self, point, route):
        left, right = self.corridor_bounds(route)
        if left is None and right is None:
            return True
        cross = float((rotation(route.heading).T @ point)[0])
        inset = self.cfg['wall_detection_inset_mm']
        if not ((left is None or cross >= left + inset) and
                (right is None or cross <= right - inset)):
            return False
        # Once the next straight has been learned (normally on a later lap),
        # one of its side walls is the end wall of this straight. Use its far
        # boundary to reject coloured objects seen over that exterior wall.
        if route.direction in ('blue', 'orange'):
            step = -math.pi / 2 if route.direction == 'blue' else math.pi / 2
            next_heading = wrap(route.heading + step)
            next_left, next_right = self._bounds_for_heading(next_heading)
            if next_left is not None and next_right is not None:
                basis = rotation(next_heading).T
                robot_cross = float((basis @ self._pose_xy)[0])
                point_cross = float((basis @ point)[0])
                if robot_cross < next_left and point_cross > next_right - inset:
                    return False
                if robot_cross > next_right and point_cross < next_left + inset:
                    return False
        return True

    def update(self, detections, pose, telemetry, now, active_target=None, route=None):
        self.unmapped = 0
        self.unmapped_close = 0
        self.environment_rejected = 0
        self._pose_heading = pose.heading
        self._pose_xy = pose.xy.copy()
        self.pillar_tof_sides = set()
        self.active_wall_heading = route.heading if route is not None else None
        self.active_wall_bounds = self.corridor_bounds(route)
        matched = set()
        outside_map = []
        unknown_close = []
        for detection in detections:
            # Bounding box bottom-centre estimates the visible contact point.
            # Clipped bases are not reliable new ground measurements.
            pixel = (detection.center, detection.bottom - 1)
            point = (self.mapper.project(pixel)
                     if detection.bottom < self.cfg['image_size'][1] - 2 else None)
            if point is None:
                outside_map.append((detection, np.asarray(pixel, dtype=float)))
                continue
            if point[1] <= 0:
                self.unmapped += 1
                continue
            world = pose.xy + rotation(pose.heading) @ point
            if route is not None and not self._inside_corridor(world, route):
                self.environment_rejected += 1
                continue
            candidates = [t for t in self.tracks if t.color == detection.color
                          and t.ident not in matched and
                          np.linalg.norm(t.point - world) < self.cfg['track_match_mm']]
            if candidates:
                target = min(candidates, key=lambda t: np.linalg.norm(t.point - world))
                target.point = 0.65 * target.point + 0.35 * world
                target.last_seen = now
                target.last_pixel = np.asarray(pixel, dtype=float)
                if route is not None and not route.turning:
                    target.deferred = False
            else:
                during_turn = route is not None and route.turning
                target = Track(self.next_id, detection.color, world, now,
                               last_pixel=np.asarray(pixel, dtype=float),
                               pass_heading=route.heading if during_turn else None,
                               deferred=during_turn)
                self.next_id += 1
                self.tracks.append(target)
            matched.add(target.ident)

        # Process these after metric detections. During a turn, a continuously
        # visible pillar can cross the calibration boundary. Retain its measured
        # world point when colour, timing and pixel motion show continuity.
        image_width, image_height = self.cfg['image_size']
        for detection, pixel in outside_map:
            self.unmapped += 1
            candidates = [
                t for t in self.tracks
                if t.color == detection.color and t.ident not in matched and
                t.last_pixel is not None and now - t.last_seen <= 1.0 and
                abs(pixel[0] - t.last_pixel[0]) <= image_width * 0.25 and
                abs(pixel[1] - t.last_pixel[1]) <= image_height * 0.35
            ]
            # The active pass may be briefly hidden and reappear after a larger
            # image jump during a turn. It already has a measured world point,
            # so retain that point when the same colour reappears nearby.
            if not candidates and active_target is not None:
                active = next((t for t in self.tracks
                               if t.ident == active_target and
                               t.color == detection.color and
                               t.ident not in matched and
                               t.last_pixel is not None and
                               now - t.last_seen <= 3.0), None)
                if active is not None:
                    relative = local(active.point, pose.xy, pose.heading)
                    same_region = (abs(pixel[0] - active.last_pixel[0]) <= image_width * 0.45 and
                                   abs(pixel[1] - active.last_pixel[1]) <= image_height * 0.65)
                    nearby = np.linalg.norm(relative) <= self.cfg['horizon_mm'] + 700
                    if same_region and nearby:
                        candidates = [active]
            if candidates:
                target = min(candidates, key=lambda t:
                             abs(pixel[0] - t.last_pixel[0]) +
                             0.5 * abs(pixel[1] - t.last_pixel[1]))
                target.last_seen = now
                target.last_pixel = pixel
                matched.add(target.ident)
            elif detection.bottom > image_height * self.cfg['unmapped_close_bottom_fraction']:
                unknown_close.append(detection)

        # A single bad contour must not stop the car. Confirm that a similarly
        # positioned, genuinely low contour persists for several camera cycles.
        if unknown_close:
            detection = max(unknown_close, key=lambda d: d.bottom)
            signature = (detection.color, float(detection.center))
            same = (self.unknown_close_signature is not None and
                    signature[0] == self.unknown_close_signature[0] and
                    abs(signature[1] - self.unknown_close_signature[1]) <= image_width * 0.20)
            if not same:
                self.unknown_close_since = now
            self.unknown_close_signature = signature
            if now - self.unknown_close_since >= self.cfg['unmapped_confirm_seconds']:
                self.unmapped_close = len(unknown_close)
                self.unmapped_detail = (detection.color, int(detection.center), detection.bottom)
        else:
            self.unknown_close_since = None
            self.unknown_close_signature = None
        if route is not None and not route.turning:
            for track in self.tracks:
                track.deferred = False
        # Tracks near the robot are never erased just because they leave view.
        self.tracks = [t for t in self.tracks if now - t.last_seen < 5 or
                       np.linalg.norm(t.point - pose.xy) < 2500]
        # Match camera pillars before learning this frame's side ToF hits. A
        # nearby pillar must not seed a persistent wall model on first sight.
        self._update_walls(pose, telemetry, now, route)


class Route:
    def __init__(self, cfg):
        self.cfg = cfg
        self.heading = 0.0
        self.direction = None
        self.corners = 0
        self.turning = False
        self.started = None
        self.aligned_since = None
        self.armed = True
        self.clear_since = None
        self.lockout = 0.0
        self.finish_start = None
        self.done = False
        self.paused_at = None

    def update(self, line, pose, now, paused=False):
        if self.done:
            return
        if paused:
            if self.paused_at is None:
                self.paused_at = now
            return
        if self.paused_at is not None:
            if self.started is not None:
                self.started += now - self.paused_at
            self.paused_at = None
        if self.turning:
            if now - self.started > self.cfg['turn_timeout_seconds']:
                raise ValueError('Planned turn timed out')
            if abs(wrap(self.heading - pose.heading)) < math.radians(6):
                if self.aligned_since is None:
                    self.aligned_since = now
                if now - self.aligned_since >= 0.15:
                    self.turning = False
                    self.corners += 1
                    self.lockout = now + 0.5
                    self.clear_since = None
                    if self.corners >= 12:
                        self.finish_start = pose.travel
            else:
                self.aligned_since = None
            return
        if self.finish_start is not None:
            self.done = pose.travel - self.finish_start >= self.cfg['finish_distance_mm']
            return
        if line is None:
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since >= 0.2:
                self.armed = True
        else:
            self.clear_since = None
        if line and self.armed and now >= self.lockout and (self.direction is None or line == self.direction):
            self.direction = line
            self.heading = wrap(self.heading + (math.pi / 2 if line == 'orange' else -math.pi / 2))
            self.turning = True
            self.started = now
            self.aligned_since = None
            self.armed = False


@dataclass
class Plan:
    servo: float
    path: np.ndarray
    waypoint: np.ndarray
    wall_shift: float
    reason: str
    target: int | None = None
    side_escape: bool = False


class Planner:
    def __init__(self, cfg, ground_error=0):
        self.cfg = cfg
        table = np.asarray(cfg['steering_table'])
        self.servos, self.angles = table[:, 0], np.radians(table[:, 1])
        order = np.argsort(self.angles)
        self.centre = float(np.interp(0, self.angles[order], self.servos[order]))
        self.last_servo = self.centre
        self.ground_error = ground_error
        self.target = None
        self.front_escape = 0
        self.front_escape_start = None

    def remember_front_escape(self, telemetry, pose, route):
        """Choose once per recovery, then retain the choice during forward travel."""
        if route.turning or self.front_escape:
            return
        left, right = telemetry['tof'][1:]
        lv, rv = 0 < left <= 600, 0 < right <= 600
        if lv and rv:
            self.front_escape = 1 if left < right else -1 if right < left else 0
        elif lv:
            self.front_escape = 1
        elif rv:
            self.front_escape = -1
        self.front_escape_start = None

    def radius(self, track, now):
        # Contact point is a visible face, not necessarily the pillar centre.
        # Half the diagonal bounds the centre of a square pillar from a visible
        # face/corner. The old full diagonal doubled its physical extent.
        # Reserve drift allowance from the start, rather than suddenly inflate
        # the obstacle into the car when its image disappears mid-pass.
        return (self.cfg['pillar_width_mm'] / math.sqrt(2) + self.ground_error +
                self.cfg['odometry_margin_mm'])

    def collision_radius(self, track, now):
        """Camera-anchored collision radius with uncertainty growing after dropout."""
        age = max(0.0, now - track.last_seen)
        drift_fraction = min(1.0, age / self.cfg['track_uncertainty_seconds'])
        return (self.cfg['pillar_width_mm'] / math.sqrt(2) + self.ground_error +
                self.cfg['odometry_margin_mm'] * drift_fraction)

    def gate_pair(self, tracks, current, heading):
        """Return an opposite-colour pillar beside the current one, if any."""
        current_xy = rotation(heading).T @ current.point
        along_limit = (self.cfg['front_from_rear_axle_mm'] +
                       self.cfg['rear_overhang_mm'] + self.cfg['pillar_width_mm'])
        choices = []
        for other in tracks:
            if (other.ident == current.ident or other.color == current.color or
                    other.passed or other.deferred):
                continue
            other_xy = rotation(heading).T @ other.point
            separation = other_xy - current_xy
            correct_order = separation[0] > 0 if current.color == 'red' else separation[0] < 0
            if correct_order and abs(separation[1]) <= along_limit:
                choices.append((abs(separation[1]), abs(separation[0]), other))
        return min(choices, key=lambda entry: entry[:2])[2] if choices else None

    def wall_shift(self, telemetry, pose, route, ignored_sides=()):
        left, right = telemetry['tof'][1:]
        lv = 0 < left < 600 and 'left' not in ignored_sides
        rv = 0 < right < 600 and 'right' not in ignored_sides
        # Side rays do not estimate corridor centre during a 90-degree turn.
        # Their occupied points and close-distance stop still apply throughout.
        if abs(wrap(route.heading - pose.heading)) > math.radians(25):
            return 0.0
        if lv and rv:
            # Sensor origins need not be symmetric about the rear axle.
            lx = self.cfg['left_tof_origin_mm'][0] - left
            rx = self.cfg['right_tof_origin_mm'][0] + right
            error = (lx + rx) / 2
        else:
            error = (max(0, self.cfg['wall_distance_mm'] - left) if lv else 0)
            error -= max(0, self.cfg['wall_distance_mm'] - right) if rv else 0
        return float(np.clip(error * self.cfg['wall_gain'], -120, 120))

    def waypoint(self, world, pose, route, telemetry, now):
        shift = self.wall_shift(telemetry, pose, route, world.pillar_tof_sides)
        goal = pose.xy + rotation(route.heading) @ [shift, self.cfg['lookahead_mm']]
        candidates = []
        for track in world.tracks:
            if (track.deferred and route.turning and
                    local(track.point, pose.xy, pose.heading)[1] > 0 and
                    local(track.point, pose.xy, route.heading)[1] > 0):
                # A mapped pillar ahead on the outgoing straight can guide the
                # corner immediately; retain the route's heading/count state.
                track.deferred = False
            if track.pass_heading is not None:
                rel = local(pose.xy, track.point, track.pass_heading)
                if rel[1] > self.cfg['rear_overhang_mm'] + self.radius(track, now) + self.cfg['clearance_mm']:
                    track.passed = True
            rel = local(track.point, pose.xy, route.heading)
            if (not track.passed and not track.deferred and
                    -100 < rel[1] < self.cfg['horizon_mm'] +
                    self.cfg['front_from_rear_axle_mm'] + self.radius(track, now) +
                    self.cfg['obstacle_start_extra_mm']
                    and abs(rel[0]) < 700):
                candidates.append(track)
        current = next((t for t in candidates if t.ident == self.target), None)
        nearest = min(candidates, key=lambda t: np.linalg.norm(t.point - pose.xy)) if candidates else None
        if (current is None or (nearest is not None and nearest.ident != current.ident and
                np.linalg.norm(nearest.point - pose.xy) + self.cfg['target_switch_margin_mm'] <
                np.linalg.norm(current.point - pose.xy))):
            current = nearest
        if current is None:
            self.target = None
            return goal, shift
        self.target = current.ident
        if current.pass_heading is None:
            current.pass_heading = route.heading

        # Only pillars beside each other form a gate. Distant pillars on the
        # same straight must be passed separately rather than intersecting
        # their lateral constraints into an artificial narrow passage.
        frame = rotation(route.heading).T
        paired = self.gate_pair(candidates, current, route.heading)
        if paired is not None:
            if paired.pass_heading is None:
                paired.pass_heading = route.heading
            red, green = ((current, paired) if current.color == 'red'
                          else (paired, current))
            half = self.cfg['width_mm'] / 2
            lower = (frame @ red.point)[0] + half + self.collision_radius(red, now) + self.cfg['gate_clearance_mm']
            upper = (frame @ green.point)[0] - half - self.collision_radius(green, now) - self.cfg['gate_clearance_mm']
            if lower <= upper:
                pose_route = frame @ pose.xy
                goal_x = float(np.clip(pose_route[0] + shift, lower, upper))
                current_route = frame @ current.point
                relative_y = current_route[1] - pose_route[1]
                along_offset = max(-80, relative_y + self.cfg['lookahead_mm'])
                along_offset = min(along_offset, self.cfg['rear_overhang_mm'] +
                                   self.radius(current, now) + 100)
                return rotation(route.heading) @ [goal_x,
                                                   current_route[1] + along_offset], shift
        side = 1 if current.color == 'red' else -1
        gap = (self.cfg['width_mm'] / 2 + self.collision_radius(current, now) +
               self.cfg['clearance_mm'] + self.cfg['path_step_mm'] / 2)
        relative = local(pose.xy, current.point, current.pass_heading)
        # Keep the waypoint on the required side until the rear has passed.
        along = max(-80, relative[1] + self.cfg['lookahead_mm'])
        along = min(along, self.cfg['rear_overhang_mm'] + self.radius(current, now) + 100)
        return current.point + rotation(current.pass_heading) @ [side * gap, along], shift

    def simulate(self, first, second, speed=200.0, split_fraction=0.5):
        xy = np.zeros(2)
        heading = 0.0
        points = [[0.0, 0.0, 0.0]]
        steps = int(math.ceil(self.cfg['horizon_mm'] / self.cfg['path_step_mm']))
        step = self.cfg['horizon_mm'] / steps
        servo = self.last_servo
        # Encoder speed estimates how far the car travels while steering slews.
        # Startup uses 200 mm/s until movement provides a measurement.
        servo_step = self.cfg['servo_rate_per_second'] * step / max(80, speed)
        for i in range(steps):
            target = first if i < int(steps * split_fraction) else second
            servo += float(np.clip(target - servo, -servo_step, servo_step))
            angle = float(np.interp(servo, self.servos, self.angles))
            change = step * math.tan(angle) / self.cfg['wheelbase_mm']
            xy += rotation(heading + change / 2) @ [0.0, step]
            heading += change
            points.append([*xy, heading])
        return np.array(points)

    def blocked(self, path, world, pose, now):
        positions = pose.xy + path[:, :2] @ rotation(pose.heading).T
        angles = path[:, 2] + pose.heading
        half = self.cfg['width_mm'] / 2
        front, rear = self.cfg['front_from_rear_axle_mm'], self.cfg['rear_overhang_mm']
        pillar_hard_radius = self.cfg['pillar_width_mm'] / math.sqrt(2) + self.ground_error
        gate_ids = {track.ident for track in world.tracks
                    if (not track.passed and not track.deferred and
                        track.pass_heading is not None and
                        self.gate_pair(world.tracks, track, track.pass_heading) is not None)}
        objects = [(t.point, self.collision_radius(t, now), pillar_hard_radius,
                    (self.cfg['gate_clearance_mm'] if t.ident in gate_ids
                     else self.cfg['clearance_mm']) + self.cfg['path_step_mm'] / 2)
                   for t in world.tracks]
        wall_margin = self.cfg['wall_clearance_mm'] + self.cfg['path_step_mm'] / 2
        objects += [(point, 0.0, 0.0, wall_margin) for _, point in world.wall_points]
        for point, radius, hard_radius, object_margin in objects:
            delta = point - positions
            # world to the robot axes at every sample
            xx = np.cos(angles) * delta[:, 0] - np.sin(angles) * delta[:, 1]
            yy = np.sin(angles) * delta[:, 0] + np.cos(angles) * delta[:, 1]
            dx = np.maximum(np.abs(xx) - half, 0)
            dy = np.maximum(np.maximum(yy - front, -rear - yy), 0)
            distances = np.hypot(dx, dy)
            blocked = distances <= radius + object_margin
            if np.any(blocked):
                if not blocked[0]:
                    return True
                # If uncertainty puts the current pose inside only the soft
                # margin, allow a path that moves steadily and clearly outward.
                # The physical-plus-calibration envelope remains a hard block.
                if np.any(distances <= hard_radius + self.cfg['path_step_mm'] / 2):
                    return True
                if len(distances) < 2:
                    return True
                if np.min(distances[1:]) < distances[0] - 5:
                    return True
                if distances[-1] <= distances[0] + 10:
                    return True
        # Persistent ToF wall models form continuous corridor boundaries. This
        # fills the gaps between the recent individual ToF hit points.
        left_wall, right_wall = world.active_wall_bounds
        if world.active_wall_heading is not None and (left_wall is not None or right_wall is not None):
            corridor_x = (positions @ rotation(world.active_wall_heading))[:, 0]
            relative_angle = angles - world.active_wall_heading
            sine = np.sin(relative_angle)
            side_extent = half * np.abs(np.cos(relative_angle))
            left_extent = side_extent + np.maximum(-front * sine, rear * sine)
            right_extent = side_extent + np.maximum(front * sine, -rear * sine)

            def crosses_wall(clearance):
                blocked_samples = clearance <= 10.0
                if not np.any(blocked_samples):
                    return False
                if not blocked_samples[0]:
                    return True
                return (np.min(clearance[1:]) < clearance[0] - 5 or
                        clearance[-1] <= clearance[0] + 10)

            if left_wall is not None and crosses_wall(corridor_x - left_extent - left_wall):
                return True
            if right_wall is not None and crosses_wall(right_wall - corridor_x - right_extent):
                return True
        for track in world.tracks:
            if track.passed or track.deferred or track.pass_heading is None:
                continue
            rel = (positions - track.point) @ rotation(track.pass_heading)
            side = 1 if track.color == 'red' else -1
            # Do not start a pass on the wrong side merely because the horizon
            # ends before crossing the obstacle's plane.
            if (rel[0, 1] < 0 and rel[-1, 1] > rel[0, 1] and
                    rel[0, 0] * side < half + self.radius(track, now) + self.cfg['clearance_mm']
                    and (rel[-1, 0] - rel[0, 0]) * side < -1):
                return True
            # Enforce colour side when the rear axle crosses the pillar plane.
            crossing = np.where((rel[:-1, 1] < 0) & (rel[1:, 1] >= 0))[0]
            clearance = (self.cfg['gate_clearance_mm'] if track.ident in gate_ids
                         else self.cfg['clearance_mm'])
            gap = half + self.collision_radius(track, now) + clearance
            for i in crossing:
                fraction = -rel[i, 1] / (rel[i + 1, 1] - rel[i, 1])
                x = rel[i, 0] + fraction * (rel[i + 1, 0] - rel[i, 0])
                if (x < gap if track.color == 'red' else x > -gap):
                    return True
        return False

    def plan(self, world, pose, route, telemetry, now):
        left, right = telemetry['tof'][1:]
        left_close = (0 < left < self.cfg['wall_stop_mm'] and
                      'left' not in world.pillar_tof_sides)
        right_close = (0 < right < self.cfg['wall_stop_mm'] and
                       'right' not in world.pillar_tof_sides)
        front = telemetry['tof'][0]
        # The team's front range is capped at 150; keep the threshold below the
        # cap so the normal far value cannot trigger recovery.
        if 0 < front < self.cfg['front_stop_mm']:
            return Plan(self.centre, np.empty((0, 3)), pose.xy, 0,
                        f'Front ToF too close: {front} mm')
        goal, shift = self.waypoint(world, pose, route, telemetry, now)
        if route.turning:
            self.front_escape = 0
            self.front_escape_start = None
        elif self.front_escape:
            if self.front_escape_start is None:
                self.front_escape_start = pose.travel
            if pose.travel - self.front_escape_start >= self.cfg['recovery_reset_forward_mm']:
                self.front_escape = 0
                self.front_escape_start = None
        if left_close and right_close:
            # No unambiguous lateral escape. Use the normal checked recovery.
            return Plan(self.centre, np.empty((0, 3)), goal, shift,
                        'No collision-free sampled path', self.target)
        escape = 1 if left_close else -1 if right_close else 0
        # Pillar colour dictates the passing side. Otherwise keep the clearer
        # side selected at the front-wall stop, through the forward retry.
        if not escape and self.target is None:
            escape = self.front_escape
        if escape:
            # Use the robot's current frame even mid-corner; route centering is
            # intentionally suppressed during turns, but a close wall is not.
            shift = escape * self.cfg['wall_distance_mm']
            goal = pose.xy + rotation(pose.heading) @ [shift, self.cfg['lookahead_mm']]
        target_local = local(goal, pose.xy, pose.heading)
        # Nine first-stage choices × three second-stage choices. Only a no-path
        # result expands the search to two alternative switch points.
        sample_servos = sorted(set(np.linspace(self.servos[0], self.servos[-1], 9).tolist() + [self.centre]))
        best = None
        for split_fraction in (0.5, 0.25, 0.75):
            for first in sample_servos:
                if escape and np.interp(first, self.servos, self.angles) * escape <= 0:
                    continue
                for second in (self.servos[0], self.centre, self.servos[-1]):
                    path = self.simulate(first, second,
                                         pose.speed if pose.speed > 20 else 200,
                                         split_fraction)
                    if escape:
                        approach = path[:max(2, int(len(path) * split_fraction) + 1), 0] * escape
                        if approach.min() < -5 or approach[-1] <= 10:
                            continue
                    if self.blocked(path, world, pose, now):
                        continue
                    distances = np.linalg.norm(path[:, :2] - target_local, axis=1)
                    idx = max(1, int(np.argmin(distances)))
                    # Reward movement towards the waypoint and final route alignment.
                    # For an obstacle, reward sustained approach to the passing
                    # waypoint across the whole horizon. The previous 0.30 weight
                    # could prefer a brief late bend and then straighten too early.
                    final_weight = 0.65 if self.target is not None else 0.30
                    score = distances[idx] + final_weight * distances[-1]
                    alignment_weight = 15 if self.target is not None or escape else 70
                    score += alignment_weight * abs(wrap(pose.heading + path[-1, 2] - route.heading))
                    score += self.cfg['steering_change_penalty'] * abs(first - self.last_servo)
                    if best is None or score < best[0]:
                        best = score, first, path
            # The normal 27 paths keep typical cycles fast. Try earlier/later
            # counter-steering only when none of those paths fits a narrow gap.
            if best is not None:
                break
        if best is None:
            return Plan(self.centre, np.empty((0, 3)), goal, shift, 'No collision-free sampled path', self.target)
        # simulate() already models physical servo slew. Gradually limiting the
        # command here as well double-limited steering and caused tiny, hesitant
        # corrections followed by another reverse.
        servo = float(best[1])
        return Plan(servo, best[2], goal, shift, '', self.target, bool(escape))


def draw_map(frame, detections, world, pose, route, plan):
    view = frame.copy()
    for p in detections:
        cv2.rectangle(view, (p.x, p.y), (p.x + p.width, p.bottom), (0, 255, 255), 2)
    panel = np.full((600, 600, 3), 25, np.uint8)
    def pix(point):
        return (int(300 + point[0] * 0.35), int(480 - point[1] * 0.35))
    for distance in range(-400, 1201, 200):
        cv2.line(panel, pix([-800, distance]), pix([800, distance]), (50, 50, 50), 1)
    cv2.arrowedLine(panel, pix([0, 0]), pix([0, 120]), (255, 255, 255), 2)
    for track in world.tracks:
        rel = local(track.point, pose.xy, pose.heading)
        color = (0, 0, 255) if track.color == 'red' else (0, 255, 0)
        cv2.circle(panel, pix(rel), 9, color, 2)
        cv2.putText(panel, f'{track.ident}: {rel[0]:.0f},{rel[1]:.0f} mm', pix(rel + [20, 0]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    for _, point in world.wall_points:
        cv2.circle(panel, pix(local(point, pose.xy, pose.heading)), 2, (160, 160, 160), -1)
    route_position = rotation(route.heading).T @ pose.xy
    for bound in world.active_wall_bounds:
        if bound is not None:
            ends = [rotation(route.heading) @ [bound, route_position[1] - 500],
                    rotation(route.heading) @ [bound, route_position[1] + 1200]]
            cv2.line(panel, pix(local(ends[0], pose.xy, pose.heading)),
                     pix(local(ends[1], pose.xy, pose.heading)), (255, 0, 255), 2)
    if len(plan.path):
        cv2.polylines(panel, [np.array([pix(p[:2]) for p in plan.path], np.int32)], False, (255, 200, 0), 2)
    cv2.circle(panel, pix(local(plan.waypoint, pose.xy, pose.heading)), 6, (0, 255, 255), -1)
    label = (f'corner {route.corners}/12 {"TURN" if route.turning else "STRAIGHT"} '
             f'wall shift {plan.wall_shift:.0f}mm servo {plan.servo:.1f}')
    cv2.putText(panel, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(panel, plan.reason or f'target={plan.target}, unmapped={world.unmapped}', (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 180, 255), 1)
    left_wall, right_wall = world.active_wall_bounds
    tof_pillars = ','.join(sorted(world.pillar_tof_sides)) or 'none'
    cv2.putText(panel, f'walls={left_wall},{right_wall} tof_pillars={tof_pillars} '
                f'env_rejected={world.environment_rejected}',
                (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
    cv2.imshow('Camera detections', view)
    cv2.imshow('Ground coordinates: x right, y forward (mm)', panel)


class CommandLink(hardware.PicoComms):
    """Serialize writes and stop on a stale planning loop, while Python is alive."""
    def __init__(self, *args, centre=hardware.SERVO_CENTER,
                 watchdog_seconds=DEFAULTS['command_watchdog_seconds']):
        self.centre = centre
        self.watchdog_seconds = watchdog_seconds
        self.write_lock = threading.Lock()
        super().__init__(*args)
        self.last_command = time.monotonic()
        self.motion_sign = 0
        self.armed = threading.Event()
        self.expired = threading.Event()
        self.end_watchdog = threading.Event()
        self.watchdog = threading.Thread(target=self._watch, daemon=True)
        self.watchdog.start()

    def send(self, *args, **kwargs):
        with self.write_lock:
            super().send(*args, **kwargs)

    def command(self, servo, speed):
        with self.write_lock:
            if self.expired.is_set():
                raise ValueError('Planning watchdog expired; restart while stationary')
            super().send(hardware.DIR_FWD, speed, servo, hardware.LED_DRIVING)
            self.motion_sign = 1
            self.last_command = time.monotonic()
            self.armed.set()

    def reverse(self, servo, speed):
        with self.write_lock:
            if self.expired.is_set():
                raise ValueError('Planning watchdog expired; restart while stationary')
            super().send(hardware.DIR_BWD, speed, servo, hardware.LED_DRIVING)
            self.motion_sign = -1
            self.last_command = time.monotonic()
            self.armed.set()

    def stop(self, flags=0):
        with self.write_lock:
            super().send(hardware.DIR_STOP, 0, self.centre, hardware.LED_IDLE, flags)
            self.motion_sign = 0
            self.armed.clear()

    def _watch(self):
        while not self.end_watchdog.wait(0.04):
            with self.write_lock:
                command_age = time.monotonic() - self.last_command
                if self.armed.is_set() and command_age > self.watchdog_seconds:
                    self.expired.set()
                    self.motion_sign = 0
                    print(f'[SAFETY] Planning watchdog expired after '
                          f'{command_age*1000:.0f} ms without a drive command; restart required')
                    try:
                        super().send(hardware.DIR_STOP, 0, self.centre, hardware.LED_IDLE)
                    except (OSError, hardware.serial.SerialException):
                        pass

    def close(self):
        self.end_watchdog.set()
        self.watchdog.join(timeout=0.2)
        super().close()


def configure_camera(camera, cfg):
    camera.configure(camera.create_preview_configuration(
        main={'size': tuple(cfg['image_size']), 'format': 'RGB888'},
        raw={'size': tuple(cfg['raw_size'])}, buffer_count=3, queue=False))
    camera.set_controls(cfg['camera_controls'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--drive', action='store_true', help='Enable movement; default is camera/coordinate preview')
    parser.add_argument('--check-config', action='store_true', help='Validate calibration without opening camera or UART')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--port', default=hardware.SERIAL_PORT)
    parser.add_argument('--tof-layout', choices=('team', 'legacy'), default='team')
    args = parser.parse_args()
    cfg, mapper = load_config(args.config)
    planner = Planner(cfg, mapper.error_mm)
    print(f'Independent floor check max error: {mapper.error_mm:.1f} mm')
    if args.check_config:
        print('Geometry, steering table and measured floor map passed software validation.')
        return
    from picamera2 import Picamera2
    camera, link = Picamera2(), None
    started = False
    try:
        configure_camera(camera, cfg)
        camera.start()
        started = True
        print('Warming camera pipeline (motors remain stopped)...')
        for _ in range(4):
            warm = camera.capture_array()
        if warm.shape[1::-1] != tuple(cfg['image_size']):
            raise ValueError('Frame size differs from floor calibration')
        # Open motor link only after camera startup; preview never opens it.
        if args.drive:
            link = CommandLink(args.port, hardware.BAUD_RATE, args.tof_layout,
                               centre=planner.centre,
                               watchdog_seconds=cfg['command_watchdog_seconds'])
            link.stop()
            deadline = time.monotonic() + 0.30
            while not link.telemetry_fresh() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not link.telemetry_fresh():
                raise ValueError('No live telemetry')
        pose, route, world = Pose(cfg), Route(cfg), World(cfg, mapper)
        lines = hardware.LineDetector(
            cfg['line_roi_top'], cfg['line_trigger_pct'],
            cfg['bottom_center_ignore_width_fraction'],
            cfg['bottom_center_ignore_top_fraction'])
        cycle_monitor = CycleMonitor(cfg['max_frame_seconds'], cfg['slow_cycle_count'])
        timing_reports = 0
        motion = EncoderMonitor(cfg['encoder_stall_seconds'])
        recovery = Recovery(cfg)
        no_path_monitor = PersistenceMonitor(cfg['no_path_confirm_seconds'])
        recovery_pause_until = 0.0
        next_encoder_report = 0.0
        next_line_report = 0.0
        while True:
            beginning = time.monotonic()
            frame = camera.capture_array()
            captured = now = time.monotonic()
            if frame.shape[1::-1] != tuple(cfg['image_size']):
                raise ValueError('Frame size differs from floor calibration')
            telemetry = link.get_telemetry() if link else {
                'yaw': 0, 'enc': 0, 'tof': [9999, 9999, 9999],
                'rear_mm': 9999, '_sample_time': now}
            # UART can receive a newer sample between capture and this read.
            # Compare freshness against a time taken after the telemetry snapshot.
            now = time.monotonic()
            distance = pose.update(telemetry, now, link.motion_sign if link else 1)
            motion.update(distance, now)
            recovery.refresh(pose.travel)
            if args.drive and now >= next_encoder_report:
                front, left, right = telemetry['tof']
                print(f'[ENCODER] count={pose.encoder} travel={pose.travel:.1f}mm '
                      f'sample_age={(now-telemetry["_sample_time"])*1000:.0f}ms '
                      f'tof F/L/R/B={front}/{left}/{right}/'
                      f'{telemetry.get("rear_mm", 9999)}mm')
                next_encoder_report = now + 0.5
            detections = hardware.detect_pillars(
                frame, cfg['pillar_ignore_top_fraction'],
                cfg['bottom_center_ignore_width_fraction'],
                cfg['bottom_center_ignore_top_fraction'])
            line, line_pct, line_roi = lines.detect(frame)
            was_turning = route.turning
            route.update(line, pose, now,
                         paused=recovery.active or now < recovery_pause_until)
            # Reversing changes image scale quickly and can make one pillar look
            # like a new metric point before odometry/camera estimates settle.
            # Keep the existing track frozen while still updating all ToF walls.
            mapping_paused = recovery.active or now < recovery_pause_until
            world.update([] if mapping_paused else detections,
                         pose, telemetry, now, planner.target, route)
            if not was_turning and route.turning:
                print(f'[NAV] {line} turn line accepted at {line_pct*100:.1f}%')
            if now >= next_line_report:
                print(f'[LINE] orange={lines.orange_pct*100:.1f}% '
                      f'blue={lines.blue_pct*100:.1f}% '
                      f'need={cfg["line_trigger_pct"]*100:.1f}% '
                      f'state={"TURN" if route.turning else "STRAIGHT"}')
                left_wall, right_wall = world.active_wall_bounds
                print(f'[WALL] left={left_wall} right={right_wall} '
                      f'tof_pillars={sorted(world.pillar_tof_sides)} '
                      f'env_rejected={world.environment_rejected}')
                next_line_report = now + 1.0
            plan = planner.plan(world, pose, route, telemetry, now)
            no_path = plan.reason == 'No collision-free sampled path'
            front_close = plan.reason.startswith('Front ToF too close:')
            checking_forward = not recovery.active and now >= recovery_pause_until
            no_path_confirmed = no_path_monitor.update(no_path and checking_forward, now)
            planned = time.monotonic()
            elapsed = planned - beginning
            if timing_reports < 10:
                print(f'[TIMING] camera={(captured-beginning)*1000:.0f}ms '
                      f'vision+planner={(planned-captured)*1000:.0f}ms '
                      f'total={elapsed*1000:.0f}ms')
                timing_reports += 1
            if args.drive and world.unmapped_close:
                colour, x, bottom = world.unmapped_detail
                plan.reason = (f'Persistent unknown {colour} contour outside floor map: '
                               f'x={x}, bottom={bottom}px')
            if args.drive and cycle_monitor.update(elapsed):
                plan.reason = (f'Camera/planner loop repeatedly slow: '
                               f'camera {(captured-beginning)*1000:.0f}ms, '
                               f'vision+planner {(planned-captured)*1000:.0f}ms, '
                               f'total {elapsed*1000:.0f}ms; limit '
                               f'{cfg["max_frame_seconds"]*1000:.0f}ms')
            recoverable = (front_close or no_path_confirmed or
                           (no_path and not checking_forward))
            if args.drive and motion.stalled(now) and (not plan.reason or recoverable):
                plan.reason = (f'Encoder movement stalled: count={pose.encoder}, '
                               f'travel={pose.travel:.1f}mm; no encoder increase for '
                               f'{now-motion.last_motion:.2f}s while drive commanded. '
                               'If the wheels moved, check encoder feedback and Pico telemetry.')
            if link:
                rear = telemetry.get('rear_mm', 9999)
                rear_blocked = 0 < rear < cfg['rear_stop_mm']
                if recovery.active:
                    # A different fault remains fatal while reversing. The
                    # forward no-path/front-close result is expected here.
                    if rear_blocked:
                        plan.reason = f'Rear ToF too close to reverse: {rear} mm'
                    if rear_blocked or (plan.reason and not recoverable) or route.done:
                        link.stop()
                        print(plan.reason or '12 turns and final distance complete')
                        break
                    reversed_mm = recovery.reversed_mm(pose.travel)
                    remaining = max(0.0, cfg['recovery_reverse_mm'] - reversed_mm)
                    if remaining <= 0:
                        recovery.finish(pose.travel)
                        planner.front_escape_start = None
                        recovery_pause_until = now + cfg['recovery_replan_pause_seconds']
                        no_path_monitor.reset()
                        planner.last_servo = planner.centre
                        plan.reason = ''
                        link.stop()
                        motion.disarm()
                        print(f'[RECOVERY] Reverse complete ({reversed_mm:.0f} mm); replanning')
                    else:
                        plan.reason = ''
                        link.reverse(recovery.servo, cfg['recovery_speed'])
                        planner.last_servo = recovery.servo
                        motion.arm(time.monotonic())
                elif now < recovery_pause_until:
                    no_path_monitor.reset()
                    link.stop()
                    motion.disarm()
                    planner.last_servo = planner.centre
                    if (plan.reason and not recoverable) or route.done:
                        print(plan.reason or '12 turns and final distance complete')
                        break
                    plan.reason = ''
                elif no_path and not no_path_confirmed:
                    # A single marginal planner frame must not spend a recovery
                    # attempt. Hold still briefly and require persistence.
                    link.stop()
                    motion.disarm()
                    planner.last_servo = planner.centre
                elif recoverable:
                    if (no_path and recovery.repeated_without_progress(pose.travel)):
                        link.stop()
                        print('No forward progress after two recovery attempts; '
                              'stop and inspect obstacle positions, walls and calibration')
                        break
                    if recovery.attempts >= cfg['recovery_max_attempts']:
                        link.stop()
                        print(f'{plan.reason} after {recovery.attempts} reverse attempts')
                        break
                    if rear_blocked:
                        link.stop()
                        print(f'{plan.reason}; rear ToF is {rear} mm, cannot reverse')
                        break
                    # Reverse recovery deliberately ignores the camera/world map.
                    # The rear ToF is the sole rear-clearance authority.
                    reverse_servo = planner.centre
                    if front_close:
                        planner.remember_front_escape(telemetry, pose, route)
                    recovery.start(pose.travel, reverse_servo)
                    plan.reason = ''
                    motion.disarm()
                    link.reverse(reverse_servo, cfg['recovery_speed'])
                    planner.last_servo = reverse_servo
                    motion.arm(time.monotonic())
                    cause = 'front ToF' if front_close else 'no forward path'
                    print(f'[RECOVERY] {cause}; reversing '
                          f'{cfg["recovery_reverse_mm"]:.0f} mm '
                          f'with servo {reverse_servo:.0f} '
                          f'(attempt {recovery.attempts}/{cfg["recovery_max_attempts"]})')
                elif plan.reason or route.done:
                    link.stop()
                    print(plan.reason or '12 turns and final distance complete')
                    break
                else:
                    speed = cfg['turn_speed'] if route.turning or planner.target else cfg['motor_speed']
                    if plan.side_escape:
                        speed = min(speed, cfg['recovery_speed'])
                    link.command(plan.servo, speed)
                    planner.last_servo = plan.servo
                    motion.arm(time.monotonic())
            if not args.headless:
                lines.draw_roi(frame, line_roi, line, line_pct)
                draw_map(frame, detections, world, pose, route, plan)
                if cv2.waitKey(1) & 0xff == ord('q'):
                    break
    finally:
        try:
            if link:
                link.close()
        finally:
            if started:
                camera.stop()
            camera.close()
            if not args.headless:
                cv2.destroyAllWindows()


class Recovery:
    def __init__(self, cfg):
        self.cfg = cfg
        self.active = False
        self.attempts = 0
        self.start_travel = 0.0
        self.last_finish_travel = None
        self.servo = None

    def refresh(self, travel):
        if (not self.active and self.last_finish_travel is not None and
                travel - self.last_finish_travel >= self.cfg['recovery_reset_forward_mm']):
            self.attempts = 0
            self.last_finish_travel = None

    def repeated_without_progress(self, travel):
        return (self.attempts >= 2 and self.last_finish_travel is not None and
                travel - self.last_finish_travel < self.cfg['recovery_min_forward_mm'])

    def start(self, travel, servo=None):
        if self.active or self.attempts >= self.cfg['recovery_max_attempts']:
            return False
        self.active = True
        self.attempts += 1
        self.start_travel = travel
        self.servo = servo
        return True

    def reversed_mm(self, travel):
        return max(0.0, self.start_travel - travel)

    def finish(self, travel):
        self.active = False
        self.last_finish_travel = travel
        self.servo = None


class EncoderMonitor:
    """Start the stall timer only after a successful driving command."""
    def __init__(self, timeout=1.0):
        self.timeout = timeout
        self.last_motion = None

    def arm(self, now):
        if self.last_motion is None:
            self.last_motion = now

    def disarm(self):
        self.last_motion = None

    def update(self, distance, now):
        if self.last_motion is not None and abs(distance) > 0:
            self.last_motion = now

    def stalled(self, now):
        return self.last_motion is not None and now - self.last_motion > self.timeout


class PersistenceMonitor:
    """Confirm a continuously present condition without counting loop frames."""
    def __init__(self, seconds):
        self.seconds = positive(seconds, 'no_path_confirm_seconds')
        self.since = None

    def update(self, active, now):
        if not active:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.seconds

    def reset(self):
        self.since = None


class CycleMonitor:
    def __init__(self, limit, count):
        self.limit = positive(limit, 'max_frame_seconds')
        self.allowed = count
        self.slow = 0

    def update(self, elapsed):
        self.slow = self.slow + 1 if elapsed > self.limit else 0
        return self.slow >= self.allowed


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Stopped by user.')
    except (ValueError, RuntimeError) as error:
        print(f'STOPPED: {error}')
        raise SystemExit(2)
