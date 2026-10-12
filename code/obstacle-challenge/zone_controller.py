"""Elektron WRO controller. See README.md for hardware mapping and limitations."""
import cv2
import numpy as np
import serial
import struct
import threading
import time
import math
import argparse
from dataclasses import dataclass
import os
os.environ["QT_LOGGING_RULES"] = "qt.qpa.*=false"

# ─── UART Settings ────────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE   = 115200

# ─── Protocol constants ───────────────────────────────────────────────────────
START_BYTE  = 0xAA
CMD_TYPE    = 0x01
TELEM_TYPE  = 0x02
CMD_FLAG_RESET_YAW     = 0x01
CMD_FLAG_RESET_ENCODER = 0x02
CMD_FLAGS_ALLOWED      = CMD_FLAG_RESET_YAW | CMD_FLAG_RESET_ENCODER

DIR_STOP    = 0
DIR_FWD     = 1
DIR_BWD     = 2

LED_IDLE    = 0
LED_DRIVING = 1

# ─── Drive settings ───────────────────────────────────────────────────────────
SPEED_NORMAL   = 255   # TUNE ME: cruise motor speed on straights
SPEED_TURN     = 230   # TUNE ME: motor speed while rotating through a corner
SERVO_CENTER   = 100    # TUNE ME: servo value that points wheels dead straight
SERVO_LEFT     = 140   # full-lock left (reference / for manual testing)
SERVO_RIGHT    = 30     # full-lock right (reference / for manual testing)

# ─── Adaptive turn settings ───────────────────────────────────────────────────
TURN_SERVO_LEFT      = 150   # TUNE ME: full-lock servo value for a left turn
TURN_SERVO_RIGHT     = 0     # TUNE ME: full-lock servo value for a right turn

TURN_ANGLE_DEG       = 90.0  # WRO Open track corners are ~90°
YAW_SIGN              = -1   # governs BOTH turning and straight-line heading-hold —
                              # see note below. Flip to 1 if turns/heading-hold end up
                              # backwards again after this change.
RIGHT_TURN_YAW_DELTA = TURN_ANGLE_DEG * YAW_SIGN
LEFT_TURN_YAW_DELTA  = -TURN_ANGLE_DEG * YAW_SIGN
# NOTE on YAW_SIGN: this one flag encodes which way your IMU's yaw moves when the
# car physically turns right. It's used in two places that must agree:
#   1. Turn-target math (RIGHT/LEFT_TURN_YAW_DELTA above)
#   2. Straight-line heading-hold correction (SteeringController._servo_from_yaw_error)
# If the car corrects the wrong way on straights (steers further off instead of
# back toward target), or a turn times out instead of converging, flip this single
# constant — you should never need to hand-tune the two locations separately.

YAW_TURN_TOLERANCE_DEG = 5     # TUNE ME: how close to target heading counts as "turn complete"
TURN_TAPER_START_DEG   = 60    # TUNE ME: start easing steering back to center inside this many degrees of error
TURN_MIN_MS             = 100 #100 # minimum time before we even check for turn completion (avoids false-complete at t=0)
TURN_TIMEOUT_MS         = 2300 # Failed convergence latches a stop.
TURN_STRAIGHTEN_MS      = 120  # short straight-servo pulse after rotation completes, before resuming heading hold
TURN_LOCKOUT_MS         = 500 # TUNE ME: ignore new line detections for this long after a turn finishes
PENDING_CORNER_MAX_MM = 450.0  # Upper bound, not a measured wall-clearance distance.
PENDING_CORNER_MAX_SECONDS = 3.0
PENDING_CORNER_SPEED = 120

LINE_ROI_TOP        = 0.75
LINE_TRIGGER_PCT    = 0.15
TOTAL_CORNERS        = 12      # Three laps, four completed turns each.
FINISH_SECONDS      = 0.6     # Tune on track: travel AFTER the final turn straightens.
FINISH_SPEED        = 150
FINISH_TIMEOUT_S    = 5.0     # Distance-mode timeout (encoder stall, etc.).

# ─── Heading-hold (straight-line) correction settings ────────────────────────
YAW_CORRECT_GAIN   = 0.8 #1.6  # TUNE ME: servo units per degree of heading error
YAW_DEAD_ZONE      = 1.5   # TUNE ME: ignore heading error smaller than this (degrees)
YAW_RATE_GAIN = 0.18  # servo units per degree/second
YAW_FILTER_SECONDS = 0.12
STEERING_SLEW_PER_SECOND = 90.0
YAW_MAX_CORRECTION = 30   # TUNE ME: clamp on servo units away from center
TOF_WALL_DANGER    = 150   # TUNE ME: front ToF distance (mm) that triggers an emergency stop

# Valid side ranges: repository firmware caps side sensors at 600 mm.
TOF_CENTER_MIN_VALID_MM = 20
TOF_CENTER_MAX_VALID_MM = 600
TOF_SIDE_CLEARANCE_MM = 200     # Begin moving away from a nearby side wall.
TOF_SIDE_AVOID_GAIN = 0.50      # Heading degrees per mm below clearance.
TOF_SIDE_MAX_HEADING_DEG = 30.0 # Bounded request, still filtered and rate limited.

# ─── LAB ranges — track lines ─────────────────────────────────────────────────
# OpenCV LAB channels are all 0-255:
#   L = lightness (0=black ... 255=white)
#   A = green<->red (128=neutral, <128 green, >128 red/magenta)
#   B = blue<->yellow (128=neutral, <128 blue, >128 yellow)
#
# These are reasonable starting points for a typical orange/blue WRO line
# set. Run the LAB calibration tool first and click on your actual track
# lines to dial these in for your lighting/camera — orange in particular
# varies a lot with exposure.
ORANGE_LOWER = np.array([80,  145, 150], dtype=np.uint8)
ORANGE_UPPER = np.array([100, 165, 215], dtype=np.uint8)
BLUE_LOWER   = np.array([130, 70,   0], dtype=np.uint8)
BLUE_UPPER   = np.array([180, 105, 255], dtype=np.uint8)

# ─── Detection settings ───────────────────────────────────────────────────────
FRAME_W, FRAME_H = 1080, 480
LEFT_ZONE        = 0.25
RIGHT_ZONE       = 0.25


# ─── CRC8 ─────────────────────────────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


# ─── Serial comms ─────────────────────────────────────────────────────────────
class PicoComms:
    def __init__(self, port, baud, tof_layout="team"):
        self.tof_layout = tof_layout
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.telemetry = {"tof": [0, 0, 0], "batt_mv": 0, "yaw": 0.0, "enc": 0}
        self._lock   = threading.Lock()
        self._closing = threading.Event()
        # Diagnostics: lets you tell "receiving nothing" apart from "receiving garbage"
        self.raw_byte_count  = 0
        self.crc_error_count = 0
        self.packet_count    = 0
        self.last_rx_time    = 0.0
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        print("[COMMS] Serial open on", port)

    def telemetry_fresh(self, max_age=0.5):
        """True if we've parsed a valid telemetry packet within the last max_age seconds."""
        return (time.monotonic() - self.last_rx_time) < max_age

    def stats(self):
        with self._lock:
            return {
                "raw_bytes": self.raw_byte_count,
                "packets":   self.packet_count,
                "crc_errs":  self.crc_error_count,
                "age":       time.monotonic() - self.last_rx_time if self.last_rx_time else None,
            }

    def build_cmd(self, direction, speed, servo, led_mode, flags=0):
        """Build the current TYPE=0x01, LEN=5 command packet."""
        payload = bytes([direction, speed, servo, led_mode, flags])
        body    = bytes([CMD_TYPE, len(payload)]) + payload
        return bytes([START_BYTE]) + body + bytes([crc8(body)])

    def send(self, direction, speed, servo=SERVO_CENTER, led_mode=LED_DRIVING, flags=0):
        servo = max(0, min(180, int(round(servo))))
        speed = max(0, min(255, int(round(speed))))
        flags = int(flags)
        if flags < 0 or flags > 0xFF or flags & ~CMD_FLAGS_ALLOWED:
            raise ValueError(f"Unsupported command flags: 0x{flags & 0xFF:02X}")
        self.ser.write(self.build_cmd(direction, speed, servo, led_mode, flags))

    def stop(self, flags=0):
        self.send(DIR_STOP, 0, SERVO_CENTER, LED_IDLE, flags)

    def reset_sensors(self, *, yaw=True, encoder=True):
        """Send one stationary command carrying the requested one-shot reset flags."""
        flags = ((CMD_FLAG_RESET_YAW if yaw else 0) |
                 (CMD_FLAG_RESET_ENCODER if encoder else 0))
        if flags:
            self.stop(flags=flags)

    def get_telemetry(self):
        with self._lock:
            return {**self.telemetry, "tof": list(self.telemetry["tof"]),
                    "_sample_time": self.last_rx_time,
                    "_packet_count": self.packet_count,
                    "_crc_errors": self.crc_error_count,
                    "_raw_bytes": self.raw_byte_count}

    def _recv_loop(self):
        buf = bytearray()
        while not self._closing.is_set():
            try:
                data = self.ser.read(32)
                if not data:
                    time.sleep(0.01)
                    continue
                with self._lock:
                    self.raw_byte_count += len(data)
                buf += data
                while len(buf) >= 3:
                    if buf[0] != START_BYTE:
                        buf.pop(0)
                        continue
                    pkt_type = buf[1]
                    length   = buf[2]
                    if pkt_type != TELEM_TYPE or length != 14:
                        buf.pop(0)
                        continue
                    total    = 1 + 1 + 1 + length + 1
                    if len(buf) < total:
                        break
                    packet = buf[:total]
                    body   = packet[1:-1]
                    if crc8(body) != packet[-1]:
                        with self._lock:
                            self.crc_error_count += 1
                        buf.pop(0)
                        print("[COMMS] CRC mismatch — dropped")
                        continue
                    buf = buf[total:]
                    if pkt_type == TELEM_TYPE and length == 14:
                        self._parse_telemetry(body[2:])
            except serial.SerialException:
                break
            except Exception as e:
                if self._closing.is_set():
                    break
                print("[COMMS] recv error:", e)
                time.sleep(0.01)

    def _parse_telemetry(self, payload):
        if len(payload) < 14:
            print(f"[COMMS] short payload: {len(payload)} bytes")
            return
        t0, t1, t2, t3, batt, yaw_raw, enc_raw = struct.unpack(">HHHHHHH", payload[:14])
        # yaw_raw and enc_raw both arrive as signed int16 on the wire but get
        # unpacked here as unsigned (">H") — round-trip through signed pack/
        # unpack to recover the actual negative values instead of huge wrapped
        # positives (e.g. -40.0deg showing up as ~6513.6 after /10.0).
        yaw_signed = struct.unpack(">h", struct.pack(">H", yaw_raw))[0]
        enc        = struct.unpack(">h", struct.pack(">H", enc_raw))[0]
        with self._lock:
            self.telemetry["tof"]     = [t3, t1, t2] if self.tof_layout == "team" else [t0, t1, t2]
            self.telemetry["rear_mm"] = t0 if self.tof_layout == "team" else t3
            self.telemetry["batt_mv"] = batt
            self.telemetry["yaw"]     = yaw_signed / 10.0
            self.telemetry["enc"]     = enc
            self.packet_count        += 1
            self.last_rx_time         = time.monotonic()

    def close(self):
        try:
            self.stop()
        finally:
            self._closing.set()
            # read() has a 0.1-second timeout. Let it finish before closing
            # the descriptor underneath the receive thread.
            self._thread.join(timeout=0.3)
            self.ser.close()


# ─── Yaw helper ───────────────────────────────────────────────────────────────
def yaw_error(current, target):
    """Signed shortest-path difference (current - target), wrapped to (-180, 180]."""
    err = (current - target) % 360
    if err > 180:
        err -= 360
    return err


# ─── Steering controller (heading hold on straights) ──────────────────────────
class SteeringController:
    """Filtered PD heading hold; side range requests a small heading offset.

    Integral deliberately omitted: trim the servo mechanically first. A yaw-rate
    derivative damps motion without a derivative kick when the target changes.
    """
    def __init__(self, comms):
        self.comms = comms
        self.target_yaw = None
        self.reset_filter()

    def reset_filter(self):
        self._last_time = None
        self._last_yaw = None
        self._last_sample_time = None
        self._rate = 0.0
        self._wall_offset = 0.0
        self._servo = float(SERVO_CENTER)

    def reset_target_yaw(self, yaw):
        self.target_yaw = yaw
        self.reset_filter()

    def _wall_heading_offset(self, left, right):
        # 600 is a clipped measurement, not an exact wall distance. 9999 is
        # the firmware's invalid marker. Neither should disable the OTHER side.
        left_valid = TOF_CENTER_MIN_VALID_MM < left < TOF_CENTER_MAX_VALID_MM
        right_valid = TOF_CENTER_MIN_VALID_MM < right < TOF_CENTER_MAX_VALID_MM
        physical_right = 0.0
        if left_valid and right_valid:
            physical_right = max(-8.0, min(8.0, (right - left) * 0.025))
        left_close = max(0.0, TOF_SIDE_CLEARANCE_MM - left) if left_valid else 0.0
        right_close = max(0.0, TOF_SIDE_CLEARANCE_MM - right) if right_valid else 0.0
        avoidance = max(-TOF_SIDE_MAX_HEADING_DEG, min(TOF_SIDE_MAX_HEADING_DEG,
                         (left_close - right_close) * TOF_SIDE_AVOID_GAIN))
        # Use the stronger request, without adding two controllers together.
        if abs(avoidance) > abs(physical_right):
            physical_right = avoidance
        return physical_right * YAW_SIGN

    def update(self, telemetry, telemetry_fresh, heading_offset=0.0,
               center_walls=True, speed=SPEED_NORMAL, now=None):
        now = time.monotonic() if now is None else now
        yaw = telemetry['yaw']
        front, left, right = telemetry['tof']
        if not telemetry_fresh or not math.isfinite(yaw) or 0 < front < TOF_WALL_DANGER:
            self.comms.stop()
            self.reset_filter()
            return
        if self.target_yaw is None:
            self.reset_target_yaw(yaw)
        elapsed = now - self._last_time if self._last_time is not None else 0.02
        if elapsed <= 0:
            return
        if elapsed > 0.25:
            self.reset_filter()
            elapsed = 0.02
        dt = elapsed
        alpha = dt / (YAW_FILTER_SECONDS + dt)
        sample_time = telemetry.get('_sample_time', now)
        if self._last_sample_time != sample_time:
            sample_dt = sample_time - self._last_sample_time if self._last_sample_time is not None else dt
            rate = yaw_error(yaw, self._last_yaw) / sample_dt if self._last_yaw is not None and sample_dt > 0 else 0.0
            rate_alpha = sample_dt / (YAW_FILTER_SECONDS + sample_dt) if sample_dt > 0 else 1.0
            self._rate += rate_alpha * (rate - self._rate)
            self._last_yaw, self._last_sample_time = yaw, sample_time
        wall = self._wall_heading_offset(left, right) if center_walls else 0.0
        self._wall_offset += alpha * (wall - self._wall_offset)
        if not center_walls:
            self._wall_offset = 0.0
        err = yaw_error(yaw, self.target_yaw + self._wall_offset + heading_offset)
        # Continuous dead band removes the original 13-unit steering step.
        err = math.copysign(max(0.0, abs(err) - YAW_DEAD_ZONE), err)
        correction = YAW_SIGN * (YAW_CORRECT_GAIN * err + YAW_RATE_GAIN * self._rate)
        correction = max(-YAW_MAX_CORRECTION, min(YAW_MAX_CORRECTION, correction))
        desired = SERVO_CENTER + correction
        max_step = STEERING_SLEW_PER_SECOND * dt  # servo units / second, independent of camera FPS
        self._servo += max(-max_step, min(max_step, desired - self._servo))
        self._servo = max(SERVO_RIGHT, min(SERVO_LEFT, self._servo))
        self._last_time = now
        self.comms.send(DIR_FWD, speed, self._servo, LED_DRIVING)


# ─── Line detector ────────────────────────────────────────────────────────────
def bottom_center_bounds(shape, width_fraction, top_fraction):
    """Return the excluded bottom-centre rectangle, or None when disabled."""
    height, width = shape[:2]
    excluded_width = int(round(width * width_fraction))
    if excluded_width <= 0:
        return None
    left = (width - excluded_width) // 2
    return left, int(round(height * top_fraction)), left + excluded_width, height


class LineDetector:
    def __init__(self, roi_top=LINE_ROI_TOP, trigger_pct=LINE_TRIGGER_PCT,
                 bottom_ignore_width=0.0, bottom_ignore_top=1.0):
        self.roi_top = roi_top
        self.trigger_pct = trigger_pct
        self.bottom_ignore_width = bottom_ignore_width
        self.bottom_ignore_top = bottom_ignore_top
        self.orange_pct = 0.0
        self.blue_pct = 0.0

    def detect(self, frame_bgr):
        h          = frame_bgr.shape[0]
        roi_y      = int(h * self.roi_top)
        roi        = frame_bgr[roi_y:, :]
        roi_pixels = roi.shape[0] * roi.shape[1]
        lab        = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        ignored = bottom_center_bounds(
            frame_bgr.shape, self.bottom_ignore_width, self.bottom_ignore_top)
        if ignored is not None:
            x0, y0, x1, y1 = ignored
            local_y0 = max(0, y0 - roi_y)
            local_y1 = max(0, y1 - roi_y)
            if local_y0 < roi.shape[0]:
                # Remove the hidden hardware before blurring so its colour
                # cannot bleed a few pixels outside the ignored rectangle.
                lab[local_y0:local_y1, x0:x1] = (0, 128, 128)
                roi_pixels -= (min(local_y1, roi.shape[0]) - local_y0) * (x1 - x0)
        lab = cv2.GaussianBlur(lab, (7, 7), 0)

        orange_mask = cv2.inRange(lab, ORANGE_LOWER, ORANGE_UPPER)
        blue_mask   = cv2.inRange(lab, BLUE_LOWER,   BLUE_UPPER)
        if ignored is not None and local_y0 < roi.shape[0]:
            orange_mask[local_y0:local_y1, x0:x1] = 0
            blue_mask[local_y0:local_y1, x0:x1] = 0
        self.orange_pct = orange_pct = cv2.countNonZero(orange_mask) / roi_pixels
        self.blue_pct = blue_pct = cv2.countNonZero(blue_mask) / roi_pixels

        if orange_pct > self.trigger_pct and orange_pct > blue_pct:
            return "orange", orange_pct, roi_y
        if blue_pct > self.trigger_pct and blue_pct > orange_pct:
            return "blue", blue_pct, roi_y
        return None, max(orange_pct, blue_pct), roi_y

    def draw_roi(self, frame, roi_y, line, pct):
        h, w   = frame.shape[:2]
        colour = (0, 140, 255) if line == "orange" else (255, 100, 0) if line == "blue" else (180, 180, 180)
        cv2.rectangle(frame, (0, roi_y), (w, h), colour, 2)
        label = (f"LINE: {line.upper()}  {pct*100:.1f}%" if line else
                 f"LINE: none  best {pct*100:.1f}% / need {self.trigger_pct*100:.1f}%")
        cv2.putText(frame, label, (10, roi_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
        ignored = bottom_center_bounds(
            frame.shape, self.bottom_ignore_width, self.bottom_ignore_top)
        if ignored is not None:
            x0, y0, x1, y1 = ignored
            overlay = frame.copy()
            cv2.rectangle(overlay, (x0, y0), (x1, y1), (40, 40, 40), -1)
            cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 255), 2)
            cv2.putText(frame, 'IGNORED', (x0 + 5, min(h - 8, y0 + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        return frame


# ─── Navigator ────────────────────────────────────────────────────────────────
class Navigator:

    DRIVING = "DRIVING"
    TURNING = "TURNING"
    FINISHING = "FINISHING"
    DONE    = "DONE"

    PHASE_ROTATE     = "rotate"
    PHASE_STRAIGHTEN = "straighten"

    def __init__(self, comms: PicoComms, steering: SteeringController,
                 finish_seconds=FINISH_SECONDS, finish_distance_mm=None, ticks_per_mm=None):
        if not math.isfinite(finish_seconds) or finish_seconds < 0:
            raise ValueError('finish_seconds must be finite and nonnegative')
        if finish_distance_mm is not None and (
                not math.isfinite(finish_distance_mm) or finish_distance_mm <= 0 or
                ticks_per_mm is None or not math.isfinite(ticks_per_mm) or ticks_per_mm <= 0):
            raise ValueError('distance finish requires positive distance and ticks_per_mm')
        self.finish_seconds = finish_seconds
        self.finish_distance_mm = finish_distance_mm
        self.ticks_per_mm = ticks_per_mm
        self._finish_start = None
        self._finish_encoder = None
        self._finish_ticks = 0
        self.comms          = comms
        self.steering       = steering
        self.state          = self.DRIVING
        self.direction      = None
        self.corners        = 0
        self._turn_phase    = None
        self._turn_start    = 0.0
        self._target_yaw    = 0.0
        self._turn_initial_err = 0.0
        self._straighten_end = 0.0
        self._lockout_until = 0.0
        self._turn_servo    = SERVO_CENTER
        self._line_clear_since = None
        self._line_armed = True
        self.pending_corner = None
        self._pending_started = None
        self._pending_distance = None
        self.fault = None
        self.last_turn_err  = 0.0   # exposed for the HUD

    @property
    def lap(self):
        return min(self.corners // 4, 2)

    @property
    def corner_in_lap(self):
        return self.corners % 4

    def update(self, line, telemetry, obstacle_active=False, obstacle_distance=None):
        now = time.monotonic()
        yaw = telemetry["yaw"]

        if not telemetry.get('_fresh', False) or not math.isfinite(yaw):
            self.fault = "Telemetry lost; restart while stationary"
        if 0 < telemetry['tof'][0] < TOF_WALL_DANGER:
            self.fault = "Front clearance too small; restart while stationary"
        if self.fault:
            self.state = self.DONE
        if self.state == self.DONE:
            self.comms.stop()
            return

        if self.state == self.TURNING:
            self._update_turn(yaw, now)
            if self.state == self.FINISHING:
                self._finish_start = now
                self._finish_encoder = telemetry.get('enc')
                self._update_finish(telemetry, now)
            return

        if self.state == self.FINISHING:
            self._update_finish(telemetry, now)
            return

        if self.pending_corner is not None:
            travelled = (obstacle_distance - self._pending_distance
                         if obstacle_distance is not None and self._pending_distance is not None
                         else None)
            if (travelled is None or travelled < 0 or travelled > PENDING_CORNER_MAX_MM
                    or now - self._pending_started > PENDING_CORNER_MAX_SECONDS):
                self.fault = 'Pending corner clearance exceeded travel/time limit'
                self.state = self.DONE
                self.comms.stop()
                return
            if not obstacle_active:
                saved_line = self.pending_corner
                self.pending_corner = None
                self._line_armed = False
                self._line_clear_since = None
                self._handle_line(saved_line, yaw, now)
                self._update_turn(yaw, now)
            return

        if line is None:
            if self._line_clear_since is None:
                self._line_clear_since = now
            if now - self._line_clear_since >= 0.2:
                self._line_armed = True
        else:
            self._line_clear_since = None
        expected = 'orange' if self.direction == 'CW' else 'blue'
        if line and self._line_armed and now > self._lockout_until:
            if self.direction is None or line == expected:
                if obstacle_active:
                    self.pending_corner = line
                    self._pending_started = now
                    self._pending_distance = obstacle_distance
                    self._line_armed = False
                    print('[NAV] Corner queued; completing pillar clearance at reduced speed')
                    return
                self._line_armed = False
                self._handle_line(line, yaw, now)
                if self.state == self.TURNING:
                    self._update_turn(yaw, now)

    # ── turning sub-state machine ──────────────────────────────────────────
    def _update_turn(self, yaw, now):
        if self._turn_phase == self.PHASE_ROTATE:
            elapsed_ms = (now - self._turn_start) * 1000.0
            err        = yaw_error(yaw, self._target_yaw)
            abs_err    = abs(err)
            self.last_turn_err = err

            crossed_target = err * self._turn_initial_err <= 0
            rotate_done = (elapsed_ms >= TURN_MIN_MS and (abs_err <= YAW_TURN_TOLERANCE_DEG or crossed_target)) \
                          or elapsed_ms >= TURN_TIMEOUT_MS

            if rotate_done:
                if elapsed_ms >= TURN_TIMEOUT_MS and abs_err > YAW_TURN_TOLERANCE_DEG and not crossed_target:
                    self.fault = "Turn timeout; check IMU sign and turn calibration"
                    self.state = self.DONE
                    self.comms.stop()
                    print(f"[NAV] Turn TIMED OUT — yaw err={err:+.1f}° after {elapsed_ms:.0f}ms "
                          f"(check YAW_SIGN / IMU wiring if this happens often)")
                    return
                else:
                    print(f"[NAV] Rotation complete — yaw err={err:+.1f}°  took {elapsed_ms:.0f}ms")
                self.comms.send(DIR_FWD, SPEED_TURN, SERVO_CENTER, LED_DRIVING)
                self._turn_phase     = self.PHASE_STRAIGHTEN
                self._straighten_end = now + (TURN_STRAIGHTEN_MS / 1000.0)
                return

            # Taper the steering back toward center as we approach the target heading
            if elapsed_ms >= TURN_MIN_MS and abs_err <= TURN_TAPER_START_DEG:
                frac  = max(0.15, abs_err / TURN_TAPER_START_DEG)
                servo = SERVO_CENTER + frac * (self._turn_servo - SERVO_CENTER)
            else:
                servo = self._turn_servo

            self.comms.send(DIR_FWD, SPEED_TURN, servo, LED_DRIVING)
            return

        if self._turn_phase == self.PHASE_STRAIGHTEN:
            if now < self._straighten_end:
                self.comms.send(DIR_FWD, SPEED_TURN, SERVO_CENTER, LED_DRIVING)
            else:
                self.steering.reset_target_yaw(self._target_yaw)
                self.state = self.FINISHING if self.corners >= TOTAL_CORNERS else self.DRIVING
                self._turn_phase = None
                self._lockout_until = now + (TURN_LOCKOUT_MS / 1000.0)
                print(f"[NAV] Turn done — corners={self.corners}  lap={self.lap+1}")

    # ── corner trigger ─────────────────────────────────────────────────────
    def _handle_line(self, line, yaw, now):
        if self.direction is None:
            self.direction = "CW" if line == "orange" else "CCW"
            print(f"[NAV] Direction locked: {self.direction}")

        turning_right     = (line == "orange")
        self._turn_servo  = TURN_SERVO_RIGHT if turning_right else TURN_SERVO_LEFT
        turn_name         = "RIGHT" if turning_right else "LEFT"
        yaw_delta         = RIGHT_TURN_YAW_DELTA if turning_right else LEFT_TURN_YAW_DELTA
        self.corners     += 1

        print(f"[NAV] Corner {self.corners}/{TOTAL_CORNERS} — turning {turn_name}  "
              f"(lap {self.lap+1}, corner {self.corner_in_lap}/4)  "
              f"start_yaw={yaw:.1f}°  target_yaw={(yaw + yaw_delta) % 360:.1f}°")

        self.state        = self.TURNING
        self._turn_phase  = self.PHASE_ROTATE
        self._turn_start  = now
        self._target_yaw  = (self.steering.target_yaw if self.steering.target_yaw is not None else yaw) + yaw_delta
        self._turn_initial_err = yaw_error(yaw, self._target_yaw)
        self.last_turn_err = self._turn_initial_err

    def _update_finish(self, telemetry, now):
        elapsed = now - self._finish_start
        if self.finish_distance_mm is None:
            finished = elapsed >= self.finish_seconds
        else:
            encoder = telemetry.get('enc')
            if encoder is None or self._finish_encoder is None:
                self.fault = 'Missing encoder during distance finish'
            else:
                delta = (encoder - self._finish_encoder) & 0xffff
                self._finish_encoder = encoder
                if delta > 10000:
                    self.fault = 'Encoder reset or implausible jump during finish'
                else:
                    self._finish_ticks += delta
            finished = self._finish_ticks >= self.finish_distance_mm * self.ticks_per_mm
            if elapsed >= FINISH_TIMEOUT_S and not finished:
                self.fault = 'Finish distance timeout'
        if finished or self.fault:
            self.state = self.DONE
            self.comms.stop()


# ─── Draw overlay ─────────────────────────────────────────────────────────────
def draw_overlay(frame, telemetry, nav):
    h, w = frame.shape[:2]
    cv2.line(frame, (int(w * LEFT_ZONE),  0), (int(w * LEFT_ZONE),  h), (200, 200, 200), 1)
    cv2.line(frame, (int(w * (1 - RIGHT_ZONE)), 0), (int(w * (1 - RIGHT_ZONE)), h), (200, 200, 200), 1)

    tof = telemetry["tof"]
    hud = [
        f"State: {nav.state}{'/' + nav._turn_phase if nav._turn_phase else ''}   "
        f"Dir: {nav.direction or '?'}   Lap: {nav.lap+1}/3   "
        f"Corner: {nav.corner_in_lap}/4   Total: {nav.corners}/{TOTAL_CORNERS}",
        f"ToF(mm): F={tof[0]} L={tof[1]} R={tof[2]}",
        f"Yaw: {telemetry['yaw']:.1f}deg   Batt: {telemetry['batt_mv']/1000:.2f}V   Enc: {telemetry['enc']}",
    ]
    if nav.state in (nav.DRIVING, nav.FINISHING):
        hud.append(f"Wall heading: {nav.steering._wall_offset:+.1f}deg   "
                   f"Servo request: {nav.steering._servo:.1f} (center={SERVO_CENTER})")
    if nav.state == Navigator.TURNING and nav._turn_phase == Navigator.PHASE_ROTATE:
        hud.append(f"Turn err remaining: {nav.last_turn_err:+.1f}deg "
                    f"(target {nav._target_yaw % 360:.1f}deg)")

    colours = [(255, 255, 255)] * len(hud)

    if not telemetry.get("_fresh", True):
        hud.append("!! NO TELEMETRY — STOPPED !!")
        colours.append((0, 0, 255))

    for i, (line, colour) in enumerate(zip(hud, colours)):
        cv2.putText(frame, line, (10, 28 + i * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)
    cv2.putText(frame, "WRO FE — filtered steering / obstacle practice", (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    return frame


ROBOT_WIDTH_MM = 135.0
PILLAR_WIDTH_MM = 50.0
CLEARANCE_MM = 75.0
MAX_OFFSET_DEG = 35.0
PILLAR_START_FRACTION = 0.20
ZONE_KP = 45.0  # Heading degrees per normalised image error.
ZONE_KI = 3.0
ZONE_KD = 1.5
ZONE_CLOSE_BOOST = 1.5
ZONE_SPEED = 120
SIDE_STOP_MM = 70.0
SIDE_SLOW_MM = 180.0
SIDE_PUSH_GAIN = 0.20


class ZonePID:
    def __init__(self):
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous = None
        self.last_time = None
        self.derivative = 0.0

    def update(self, error, proximity, now):
        dt = now - self.last_time if self.last_time is not None else 0.02
        if dt <= 0 or dt > 0.25:
            self.reset()
            dt = 0.02
        derivative = (error - self.previous) / dt if self.previous is not None else 0.0
        self.derivative += dt / (0.12 + dt) * (derivative - self.derivative)
        trial = max(-0.5, min(0.5, self.integral + error * dt))
        gain = 1 + ZONE_CLOSE_BOOST * max(0.0, min(1.0, proximity))
        raw = gain * (ZONE_KP * error + ZONE_KI * trial + ZONE_KD * self.derivative)
        if abs(raw) < MAX_OFFSET_DEG or raw * error < 0:
            self.integral = trial
        self.previous, self.last_time = error, now
        if abs(error) < 0.001:
            self.reset()
            return 0.0
        # Derivative braking must not reverse the required passing direction.
        return math.copysign(min(MAX_OFFSET_DEG, max(0.0, raw * math.copysign(1, error))), error)


def protect_sides(offset, telemetry):
    _, left, right = telemetry['tof']
    valid_left = 0 < left < TOF_CENTER_MAX_VALID_MM
    valid_right = 0 < right < TOF_CENTER_MAX_VALID_MM
    if ((valid_left and left < SIDE_STOP_MM) or
            (valid_right and right < SIDE_STOP_MM)):
        return 0.0, 'Side wall too close'
    physical_right = offset / YAW_SIGN
    left_push = max(0.0, SIDE_SLOW_MM - left) if valid_left else 0.0
    right_push = max(0.0, SIDE_SLOW_MM - right) if valid_right else 0.0
    if (physical_right > 0 and right_push > 0) or (physical_right < 0 and left_push > 0):
        return 0.0, 'Pillar passing side blocked by wall'
    physical_right += (left_push - right_push) * SIDE_PUSH_GAIN
    return YAW_SIGN * max(-MAX_OFFSET_DEG, min(MAX_OFFSET_DEG, physical_right)), None
PILLAR_COMMIT_TRAVEL_MM = 120.0  # Additional avoidance travel after visual loss; tune on track.
PILLAR_STALL_SECONDS = 1.0
PILLAR_EDGE_ZONE = 0.08  # Outer 8% of image; side-exit evidence only.
PILLAR_EDGE_MIN_BOTTOM = 0.60  # Image heuristic, must be checked on the car.


@dataclass(frozen=True)
class Pillar:
    color: str
    x: int
    y: int
    width: int
    height: int

    @property
    def bottom(self):
        return self.y + self.height

    @property
    def center(self):
        return self.x + self.width / 2


def detect_pillars(frame, ignore_top_fraction=0.15,
                   bottom_ignore_width=0.0, bottom_ignore_top=1.0):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = {
        'red': cv2.inRange(hsv, (0, 200, 60), (10, 255, 255)) |
               cv2.inRange(hsv, (170, 110, 60), (179, 255, 255)),
        'green': cv2.inRange(hsv, (100, 80, 100), (110, 255, 255)),
    }
    height, width = frame.shape[:2]
    ignored = bottom_center_bounds(
        frame.shape, bottom_ignore_width, bottom_ignore_top)
    result = []
    for color, mask in masks.items():
        mask[:int(height * ignore_top_fraction)] = 0
        if ignored is not None:
            x0, y0, x1, y1 = ignored
            mask[y0:y1, x0:x1] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        if ignored is not None:
            mask[y0:y1, x0:x1] = 0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            # Reject thin floor stripes and speckles; do not use hue alone.
            if (cv2.contourArea(contour) < width * height * 0.0006 or
                    h < height * 0.05 or h / max(w, 1) < 1.15 or
                    cv2.contourArea(contour) / (w * h) < 0.45):
                continue
            result.append(Pillar(color, x, y, w, h))
    return sorted(result, key=lambda p: p.bottom, reverse=True)


class ObstacleController:
    def __init__(self, yaw_sign, ticks_per_mm=None):
        if ticks_per_mm is not None and (not np.isfinite(ticks_per_mm) or ticks_per_mm <= 0):
            raise ValueError("ticks_per_mm must be positive and finite")
        self.ticks_per_mm = ticks_per_mm
        self.previous_encoder = None
        self.distance_mm = 0.0
        self.clear_start_mm = None
        self.clear_started = None
        self.yaw_sign = yaw_sign
        self.target = None
        self.last_seen = None
        self.started = None
        self.offset = 0.0
        self.detections = []
        self.fault = None
        self.candidate = None
        self.candidate_since = None
        self.tracking_paused = False
        self.edge_exit_ready = False
        self.commit_start_mm = None
        self.last_motion_time = None
        self.committed = False
        self.zone_pid = ZonePID()

    def side_exit_ready(self, pillar, width, height):
        margin = pillar.width * (ROBOT_WIDTH_MM / 2 + CLEARANCE_MM) / PILLAR_WIDTH_MM
        close = pillar.bottom >= height * PILLAR_EDGE_MIN_BOTTOM
        if pillar.color == 'green':
            return (close and pillar.x + pillar.width >= width * (1 - PILLAR_EDGE_ZONE)
                    and pillar.x - margin > width / 2)
        return (close and pillar.x <= width * PILLAR_EDGE_ZONE
                and pillar.x + pillar.width + margin < width / 2)

    @staticmethod
    def match_target(target, detections, width):
        matches = [p for p in detections if p.color == target.color and
                   abs(p.center - target.center) < width * 0.22 and
                   0.4 < p.width / target.width < 2.5 and
                   0.4 < p.height / target.height < 2.5 and
                   abs(p.bottom - target.bottom) < max(target.height * 0.6, 20)]
        # Include vertical position and size: horizontal position alone can
        # jump to a second pillar farther down the same side of the track.
        return min(matches, key=lambda p: (
            abs(p.center - target.center) + abs(p.bottom - target.bottom) +
            abs(p.width - target.width)), default=None)

    def update(self, frame, now=None, encoder=None, allow_acquire=True):
        now = time.monotonic() if now is None else now
        if encoder is not None and self.ticks_per_mm is not None:
            if self.previous_encoder is not None:
                # Firmware counts unsigned motion, transmitted as wrapped int16.
                delta = (encoder - self.previous_encoder) & 0xffff
                if delta > 10000:
                    self.fault = 'Encoder reset or implausible jump'
                else:
                    self.distance_mm += delta / self.ticks_per_mm
                    if delta > 0:
                        self.last_motion_time = now
            self.previous_encoder = encoder
        self.detections = detect_pillars(frame)
        self.tracking_paused = False
        h, w = frame.shape[:2]
        if self.fault:
            return 0.0, True, self.fault
        if self.target is not None:
            if now - self.started > 8.0:
                self.fault = 'Committed pillar manoeuvre timeout'
            elif (self.ticks_per_mm is not None and self.last_motion_time is not None
                  and now - self.last_motion_time > PILLAR_STALL_SECONDS):
                self.fault = 'Pillar manoeuvre encoder stalled'
            if self.fault:
                return 0.0, True, self.fault
        if self.clear_start_mm is not None:
            if self.distance_mm - self.clear_start_mm >= 205.0 + CLEARANCE_MM:
                self.target = None
                self.clear_start_mm = None
                self.candidate = None
                self.candidate_since = None
                self.offset = 0.0
                self.edge_exit_ready = False
                self.commit_start_mm = None
                self.committed = False
            elif now - self.clear_started > 3.0:
                self.fault = 'Encoder clearance timeout'
                return 0.0, True, self.fault
            else:
                # Hold track heading while the rear of the chassis passes.
                return 0.0, True, None
        if self.target is None:
            if not allow_acquire:
                self.candidate = None
                self.candidate_since = None
                return 0.0, False, None
            nearest = next((p for p in self.detections if p.bottom > h * PILLAR_START_FRACTION), None)
            confirmed = None
            if self.candidate is not None:
                confirmed = self.match_target(self.candidate, self.detections, w)
                if (confirmed is not None and confirmed.bottom > h * PILLAR_START_FRACTION
                        and nearest is not None and nearest.bottom - confirmed.bottom < h * 0.08):
                    nearest = confirmed
            if nearest is None:
                self.candidate = None
                self.candidate_since = None
                return 0.0, False, None
            if (self.candidate is None or nearest is not confirmed or self.candidate.color != nearest.color or
                    abs(self.candidate.center - nearest.center) > w * 0.15):
                self.candidate = nearest
                self.candidate_since = now
                # Slow-speed main loop continues while confirming the blob.
                return 0.0, False, None
            if now - self.candidate_since < 0.10:
                self.candidate = nearest
                return 0.0, False, None
            self.target = nearest
            self.started = now
            self.zone_pid.reset()
            self.last_seen = now
            self.edge_exit_ready = False
            self.last_motion_time = now
            self.commit_start_mm = None
            self.committed = False
        match = self.match_target(self.target, self.detections, w)
        if match is None:
            if (self.edge_exit_ready and self.last_seen is not None
                    and now - self.last_seen <= 0.25
                    and self.ticks_per_mm is not None and encoder is not None):
                # Only a recent, close target observed outside the vehicle's
                # estimated path may graduate to encoder clearance on loss.
                if any(q.color != self.target.color and q.bottom > h * 0.7
                       for q in self.detections):
                    self.fault = 'Close opposing pillars require a calibrated path planner'
                    return 0.0, True, self.fault
                self.clear_start_mm = self.distance_mm
                self.clear_started = now
                self.edge_exit_ready = False
                self.offset = 0.0
                return 0.0, True, None
            if self.ticks_per_mm is not None and encoder is not None:
                if any(q.color != self.target.color and q.bottom > h * 0.7
                       for q in self.detections):
                    self.fault = 'Close opposing pillars require a calibrated path planner'
                    return 0.0, True, self.fault
                self.committed = True
                if self.commit_start_mm is None:
                    self.commit_start_mm = self.distance_mm
                if self.distance_mm - self.commit_start_mm >= PILLAR_COMMIT_TRAVEL_MM:
                    self.clear_start_mm = self.distance_mm
                    self.clear_started = now
                    self.offset = 0.0
                    return 0.0, True, None
                # Retain the last requested avoidance heading and selected
                # pillar. Never select a distant replacement during dropout.
                return self.offset, True, None
            self.tracking_paused = True
            if self.last_seen is None or now - self.last_seen > 1.0:
                self.fault = (f'Active {self.target.color} pillar lost for 1s '
                              f'(last x={self.target.center / w:.2f}, '
                              f'bottom={self.target.bottom / h:.2f}); passage unconfirmed')
                return 0.0, True, self.fault
            # Main loop stops the motor while waiting for the SAME target.
            return 0.0, True, None
        self.target = match
        self.committed = False
        self.last_seen = now
        p = self.target
        self.edge_exit_ready = self.side_exit_ready(p, w, h)
        # Place the complete vehicle to the required side of the pillar.
        margin = p.width * (ROBOT_WIDTH_MM / 2 + CLEARANCE_MM) / PILLAR_WIDTH_MM
        if p.color == 'red':
            error_px = max(0.0, p.x + p.width - w / 3,
                           p.x + p.width + margin - w / 2)
        else:
            error_px = min(0.0, p.x - w * 2 / 3,
                           p.x - margin - w / 2)
        # Refuse opposing close targets: a single-target heuristic cannot plan a slalom safely.
        if any(q.color != p.color and q.bottom > h * 0.7 for q in self.detections):
            self.fault = 'Close opposing pillars require a calibrated path planner'
        if self.fault:
            return 0.0, True, self.fault
        if (abs(error_px) < 1.0 and p.bottom > h * 0.78 and
                self.ticks_per_mm is not None and encoder is not None):
            self.clear_start_mm = self.distance_mm
            self.clear_started = now
            return 0.0, True, None
        proximity = max(p.bottom / h, min(1.0, p.width / (w * 0.20)))
        self.offset = self.yaw_sign * self.zone_pid.update(error_px / w, proximity, now)
        if now - self.started > 4.0:
            self.fault = 'Pillar manoeuvre timeout'
        return self.offset, True, self.fault

    def draw(self, frame):
        h, w = frame.shape[:2]
        for x in (w // 3, 2 * w // 3):
            cv2.line(frame, (x, 0), (x, h - 1), (255, 255, 255), 2)
        for i, label in enumerate(('LEFT: red target', 'CENTER', 'RIGHT: green target')):
            cv2.putText(frame, label, (i * w // 3 + 10, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        for p in self.detections:
            color = (0, 0, 255) if p.color == 'red' else (0, 255, 0)
            selected = p == self.target
            cv2.rectangle(frame, (p.x, p.y), (p.x+p.width, p.bottom), color, 4 if selected else 1)
            label = p.color + (' TARGET' if selected else ' seen')
            cv2.putText(frame, label, (p.x, p.y-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        if self.tracking_paused:
            cv2.putText(frame, 'STOPPED: reacquiring active pillar', (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        elif self.target is not None:
            phase = ('REAR CLEARANCE' if self.clear_start_mm is not None else
                     'ENCODER PASS' if self.committed else 'VISUAL PASS')
            cv2.putText(frame, f'{self.target.color} {phase}', (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)


def update_obstacle_navigation(nav, obstacles, frame, line, telemetry):
    """Observe during corners; acquire a fresh target only after straightening."""
    offset, active = 0.0, False
    if nav.state == nav.TURNING:
        obstacles.detections = detect_pillars(frame)
        # Never silently abandon a passage that started before a turn.
        if obstacles.target is not None or obstacles.clear_start_mm is not None:
            nav.fault = 'Active pillar passage overlaps a turn; stop for calibration'
        obstacles.candidate = None
        obstacles.candidate_since = None
        obstacles.previous_encoder = None
    elif nav.state in (nav.DRIVING, nav.FINISHING):
        offset, active, fault = obstacles.update(
            frame, encoder=telemetry['enc'], allow_acquire=nav.pending_corner is None)
        if fault:
            nav.fault = fault
    offset, side_fault = protect_sides(offset, telemetry)
    if side_fault:
        nav.fault = side_fault
    nav.update(line, telemetry, obstacle_active=active, obstacle_distance=obstacles.distance_mm)
    return offset


def main():
    parser = argparse.ArgumentParser(description="WRO open / obstacle practice controller")
    parser.add_argument('--challenge', choices=['open', 'obstacle'], default='open')
    parser.add_argument('--tof-layout', choices=['team', 'legacy'], default='team',
                        help='team: wire slots rear/left/right/front; legacy: front/left/right/rear')
    parser.add_argument('--ticks-per-mm', type=float,
                        help='Measured forward encoder counts / travel mm; required for obstacle practice')
    finish = parser.add_mutually_exclusive_group()
    finish.add_argument('--finish-seconds', type=float, default=FINISH_SECONDS,
                        help='Forward travel time after final turn (default: 0.6 seconds)')
    finish.add_argument('--finish-distance-mm', type=float,
                        help='Forward travel after final turn; requires --ticks-per-mm')
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.finish_seconds) or args.finish_seconds < 0:
        parser.error('--finish-seconds must be finite and nonnegative')
    if args.ticks_per_mm is not None and (not math.isfinite(args.ticks_per_mm) or args.ticks_per_mm <= 0):
        parser.error('--ticks-per-mm must be positive and finite')
    if args.finish_distance_mm is not None and (not math.isfinite(args.finish_distance_mm)
            or args.finish_distance_mm <= 0 or args.ticks_per_mm is None):
        parser.error('--finish-distance-mm requires positive distance and --ticks-per-mm')
    if args.challenge == 'obstacle' and (args.ticks_per_mm is None or
            not math.isfinite(args.ticks_per_mm) or args.ticks_per_mm <= 0):
        parser.error('obstacle practice requires a measured --ticks-per-mm value')
    from picamera2 import Picamera2
    comms = PicoComms(SERIAL_PORT, BAUD_RATE, args.tof_layout)
    picam2 = None
    try:
        comms.stop()
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(
            main={"size": (FRAME_W, FRAME_H), "format": "RGB888"},
            raw={"size": (1640, 720)}))
        picam2.set_controls({"AeEnable": True, "AwbEnable": True, "AwbMode": 0,
                             "ExposureValue": 0.0, "Saturation": 1.4, "Sharpness": 1.5})
        picam2.start()
        steering = SteeringController(comms)
        nav = Navigator(comms, steering, args.finish_seconds,
                        args.finish_distance_mm, args.ticks_per_mm)
        line_det = LineDetector()
        obstacles = ObstacleController(YAW_SIGN, args.ticks_per_mm) if args.challenge == 'obstacle' else None
        # Never move until live telemetry has actually arrived.
        deadline = time.monotonic() + 3.0
        while not comms.telemetry_fresh() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not comms.telemetry_fresh():
            raise RuntimeError('No telemetry received; motor remains stopped')
        steering.reset_target_yaw(comms.get_telemetry()['yaw'])
        print(f"Mode: {args.challenge}. Press q or Ctrl-C to stop.")
        if obstacles:
            print('Obstacle PRACTICE mode: passing only; parking is not implemented.')
        while True:
            frame = picam2.capture_array()
            telemetry = comms.get_telemetry()
            telemetry['_fresh'] = (time.monotonic() - telemetry['_sample_time']) < 0.5
            line, line_pct, roi_y = line_det.detect(frame)
            offset = 0.0
            if obstacles:
                offset = update_obstacle_navigation(nav, obstacles, frame, line, telemetry)
            else:
                nav.update(line, telemetry)
            # Dispatch from the NEW state: never overwrite a turn or finish stop.
            if nav.state in (nav.DRIVING, nav.FINISHING):
                if obstacles and obstacles.tracking_paused:
                    comms.stop()
                    steering.reset_filter()
                else:
                    steering.update(telemetry, telemetry['_fresh'], offset,
                                    center_walls=not bool(obstacles),
                                    speed=PENDING_CORNER_SPEED if nav.pending_corner is not None else
                                          FINISH_SPEED if nav.state == nav.FINISHING else
                                          (ZONE_SPEED if obstacles else SPEED_NORMAL))
            if nav.state == nav.DONE:
                print(nav.fault or ('Practice laps finished; parking pending' if obstacles else 'Finished'))
                break
            if not args.headless:
                output = draw_overlay(frame.copy(), telemetry, nav)
                line_det.draw_roi(output, roi_y, line, line_pct)
                if obstacles:
                    obstacles.draw(output)
                cv2.imshow('WRO FE', output)
                if cv2.waitKey(1) & 0xff == ord('q'):
                    break
    finally:
        try:
            comms.close()
        finally:
            if picam2 is not None:
                picam2.stop()
                picam2.close()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
