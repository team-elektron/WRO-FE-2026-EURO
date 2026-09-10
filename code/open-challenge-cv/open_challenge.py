"""
WRO FE Open Challenge - v2
"""

import cv2
import numpy as np
import serial
import struct
import threading
import time
from picamera2 import Picamera2
import os
os.environ["QT_LOGGING_RULES"] = "qt.qpa.*=false"

#  UART Settings 
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE   = 115200

#  Protocol constants 
START_BYTE  = 0xAA
CMD_TYPE    = 0x01
TELEM_TYPE  = 0x02

DIR_STOP    = 0
DIR_FWD     = 1
DIR_BWD     = 2

LED_IDLE    = 0
LED_DRIVING = 1
LED_LOWBATT = 3

#  Drive settings 
SPEED_NORMAL   = 170   # TUNE ME: cruise motor speed on straights
SPEED_TURN     = 140   # TUNE ME: motor speed while rotating through a corner
SERVO_CENTER   = 100    # TUNE ME: servo value that points wheels dead straight
SERVO_LEFT     = 140   # full-lock left (reference / for manual testing)
SERVO_RIGHT    = 30     

#  Adaptive turn settings 
TURN_SERVO_LEFT      = 150   # TUNE ME: full-lock servo value for a left turn
TURN_SERVO_RIGHT     = 0     # TUNE ME: full-lock servo value for a right turn

TURN_ANGLE_DEG       = 90.0  # WRO Open track corners are ~90°
YAW_SIGN              = -1   # governs BOTH turning and straight-line heading-hold -
                              # see note below. Flip to 1 if turns/heading-hold end up
                              # backwards again after this change.
RIGHT_TURN_YAW_DELTA = TURN_ANGLE_DEG * YAW_SIGN
LEFT_TURN_YAW_DELTA  = -TURN_ANGLE_DEG * YAW_SIGN

YAW_TURN_TOLERANCE_DEG = 8     # TUNE ME: how close to target heading counts as "turn complete"
TURN_TAPER_START_DEG   = 35    # TUNE ME: start easing steering back to center inside this many degrees of error
TURN_MIN_MS             = 150  # minimum time before we even check for turn completion (avoids false-complete at t=0)
TURN_TIMEOUT_MS         = 1500 
TURN_STRAIGHTEN_MS      = 120  # short straight-servo pulse after rotation completes, before resuming heading hold
TURN_LOCKOUT_MS         = 1500 

LINE_ROI_TOP        = 0.30
LINE_TRIGGER_PCT    = 0.03
TOTAL_CORNERS        = 12      # 3 laps x 4 corners

#  Heading-hold (straight-line) correction settings 
YAW_CORRECT_GAIN   = 1.6   # TUNE ME: servo units per degree of heading error
YAW_DEAD_ZONE      = 8.0   # TUNE ME: ignore heading error smaller than this (degrees)
YAW_MAX_CORRECTION = 30    # TUNE ME: clamp on servo units away from center
TOF_WALL_DANGER    = 150   # TUNE ME: front ToF distance (mm) that triggers an emergency stop

# varies a lot with exposure.
ORANGE_LOWER = np.array([80,  145, 150], dtype=np.uint8)
ORANGE_UPPER = np.array([100, 165, 215], dtype=np.uint8)
BLUE_LOWER   = np.array([130, 70,   0], dtype=np.uint8)
BLUE_UPPER   = np.array([180, 105, 255], dtype=np.uint8)

#  Detection settings  
FRAME_W, FRAME_H = 640, 480
LEFT_ZONE        = 0.25
RIGHT_ZONE       = 0.25


#  CRC8 
def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


#  Serial comms 
class PicoComms:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.telemetry = {"tof": [0, 0, 0], "batt_mv": 0, "yaw": 0.0, "enc": 0}
        self._lock   = threading.Lock()
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
        return (time.time() - self.last_rx_time) < max_age

    def stats(self):
        with self._lock:
            return {
                "raw_bytes": self.raw_byte_count,
                "packets":   self.packet_count,
                "crc_errs":  self.crc_error_count,
                "age":       time.time() - self.last_rx_time if self.last_rx_time else None,
            }

    def build_cmd(self, direction, speed, servo, led_mode):
        payload = bytes([direction, speed, servo, led_mode])
        body    = bytes([CMD_TYPE, len(payload)]) + payload
        return bytes([START_BYTE]) + body + bytes([crc8(body)])

    def send(self, direction, speed, servo=SERVO_CENTER, led_mode=LED_DRIVING):
        servo = max(0, min(255, int(round(servo))))
        speed = max(0, min(255, int(round(speed))))
        self.ser.write(self.build_cmd(direction, speed, servo, led_mode))

    def stop(self):
        self.send(DIR_STOP, 0, SERVO_CENTER, LED_IDLE)

    def get_telemetry(self):
        with self._lock:
            return dict(self.telemetry)

    def _recv_loop(self):
        buf = bytearray()
        while True:
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
                    total    = 1 + 1 + 1 + length + 1
                    if len(buf) < total:
                        break
                    packet = buf[:total]
                    buf    = buf[total:]
                    body   = packet[1:-1]
                    if crc8(body) != packet[-1]:
                        with self._lock:
                            self.crc_error_count += 1
                        print("[COMMS] CRC mismatch - dropped")
                        continue
                    if pkt_type == TELEM_TYPE and length == 14:
                        self._parse_telemetry(body[2:])
            except serial.SerialException:
                break
            except Exception as e:
                print("[COMMS] recv error:", e)
                time.sleep(0.01)

    def _parse_telemetry(self, payload):
        if len(payload) < 14:
            print(f"[COMMS] short payload: {len(payload)} bytes")
            return
        t0, t1, t2, t3, batt, yaw_raw, enc_raw = struct.unpack(">HHHHHHH", payload[:14])
        enc = struct.unpack(">h", struct.pack(">H", enc_raw))[0]
        with self._lock:
            self.telemetry["tof"]     = [t0, t1, t2]
            self.telemetry["batt_mv"] = batt
            self.telemetry["yaw"]     = yaw_raw / 10.0
            self.telemetry["enc"]     = enc
            self.packet_count        += 1
            self.last_rx_time         = time.time()

    def close(self):
        self.stop()
        time.sleep(0.1)
        self.ser.close()


#  Yaw helper 
def yaw_error(current, target):
    """Signed shortest-path difference (current - target), wrapped to (-180, 180]."""
    err = (current - target) % 360
    if err > 180:
        err -= 360
    return err


#  Steering controller (heading hold on straights) 
class SteeringController:
    def __init__(self, comms: PicoComms):
        self.comms       = comms
        self.target_yaw  = None
        self.current_yaw = 0.0

    def _servo_from_yaw_error(self, err):
        correction = int(round(err * YAW_CORRECT_GAIN * YAW_SIGN))
        correction = max(-YAW_MAX_CORRECTION, min(YAW_MAX_CORRECTION, correction))
        return SERVO_CENTER + correction

    def reset_target_yaw(self, yaw):
        self.target_yaw = yaw
        print(f"[STEER] Target yaw updated to {yaw:.1f}°")

    def update(self, telemetry, telemetry_fresh):
        yaw     = telemetry["yaw"]
        self.current_yaw = yaw
        tof_fwd = telemetry["tof"][0]

        if not telemetry_fresh:
            # No live yaw feedback - don't pretend to correct heading against a
            # frozen reading, that just locks in whatever the car is already
            # doing (including any physical steering bias). Drive straight,
            # uncorrected, and let the HUD warning make the problem visible.
            self.comms.send(DIR_FWD, SPEED_NORMAL, SERVO_CENTER, LED_DRIVING)
            return

        if self.target_yaw is None:
            self.target_yaw = yaw
            return

        # Wall danger
        if 0 < tof_fwd < TOF_WALL_DANGER:
            self.comms.send(DIR_STOP, 0, SERVO_CENTER, LED_IDLE)
            print(f"[STEER] Wall danger! ToF front={tof_fwd}mm")
            return

        # Hold heading with proportional yaw correction
        err   = yaw_error(yaw, self.target_yaw)
        servo = self._servo_from_yaw_error(err) if abs(err) > YAW_DEAD_ZONE else SERVO_CENTER
        self.comms.send(DIR_FWD, SPEED_NORMAL, servo, LED_DRIVING)


#  Line detector 
class LineDetector:
    def detect(self, frame_bgr):
        h          = frame_bgr.shape[0]
        roi_y      = int(h * LINE_ROI_TOP)
        roi        = frame_bgr[roi_y:, :]
        roi_pixels = roi.shape[0] * roi.shape[1]
        lab        = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        lab        = cv2.GaussianBlur(lab, (7, 7), 0)

        orange_mask = cv2.inRange(lab, ORANGE_LOWER, ORANGE_UPPER)
        blue_mask   = cv2.inRange(lab, BLUE_LOWER,   BLUE_UPPER)
        orange_pct  = cv2.countNonZero(orange_mask) / roi_pixels
        blue_pct    = cv2.countNonZero(blue_mask)   / roi_pixels

        if orange_pct > LINE_TRIGGER_PCT and orange_pct > blue_pct:
            return "orange", orange_pct, roi_y
        if blue_pct > LINE_TRIGGER_PCT and blue_pct > orange_pct:
            return "blue", blue_pct, roi_y
        return None, 0.0, roi_y

    def draw_roi(self, frame, roi_y, line, pct):
        h, w   = frame.shape[:2]
        colour = (0, 140, 255) if line == "orange" else (255, 100, 0) if line == "blue" else (180, 180, 180)
        cv2.rectangle(frame, (0, roi_y), (w, h), colour, 2)
        label  = f"LINE: {line.upper()}  {pct*100:.1f}%" if line else "LINE: none"
        cv2.putText(frame, label, (10, roi_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
        return frame


#  Navigator 
class Navigator:

    DRIVING = "DRIVING"
    TURNING = "TURNING"
    DONE    = "DONE"

    PHASE_ROTATE     = "rotate"
    PHASE_STRAIGHTEN = "straighten"

    def __init__(self, comms: PicoComms, steering: SteeringController):
        self.comms          = comms
        self.steering       = steering
        self.state          = self.DRIVING
        self.direction      = None
        self.corners        = 0
        self._turn_phase    = None
        self._turn_start    = 0.0
        self._target_yaw    = 0.0
        self._straighten_end = 0.0
        self._lockout_until = 0.0
        self._turn_servo    = SERVO_CENTER
        self.last_turn_err  = 0.0   # exposed for the HUD

    @property
    def lap(self):
        return self.corners // 4

    @property
    def corner_in_lap(self):
        return self.corners % 4

    def update(self, line, telemetry):
        now = time.time()
        yaw = telemetry["yaw"]

        if self.state == self.DONE:
            self.comms.stop()
            return

        if self.state == self.TURNING:
            self._update_turn(yaw, now)
            return

        if line and now > self._lockout_until:
            self._handle_line(line, yaw, now)

    #  turning sub-state machine 
    def _update_turn(self, yaw, now):
        if self._turn_phase == self.PHASE_ROTATE:
            elapsed_ms = (now - self._turn_start) * 1000.0
            err        = yaw_error(yaw, self._target_yaw)
            abs_err    = abs(err)
            self.last_turn_err = err

            rotate_done = (elapsed_ms >= TURN_MIN_MS and abs_err <= YAW_TURN_TOLERANCE_DEG) \
                          or elapsed_ms >= TURN_TIMEOUT_MS

            if rotate_done:
                if elapsed_ms >= TURN_TIMEOUT_MS and abs_err > YAW_TURN_TOLERANCE_DEG:
                    print(f"[NAV] Turn TIMED OUT - yaw err={err:+.1f}° after {elapsed_ms:.0f}ms "
                          f"(check YAW_SIGN / IMU wiring if this happens often)")
                else:
                    print(f"[NAV] Rotation complete - yaw err={err:+.1f}°  took {elapsed_ms:.0f}ms")
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
                self.steering.reset_target_yaw(yaw)
                self.state       = self.DRIVING
                self._turn_phase = None
                self._lockout_until = now + (TURN_LOCKOUT_MS / 1000.0)
                print(f"[NAV] Turn done - corners={self.corners}  lap={self.lap+1}")

    #  corner trigger 
    def _handle_line(self, line, yaw, now):
        if self.direction is None:
            self.direction = "CW" if line == "orange" else "CCW"
            print(f"[NAV] Direction locked: {self.direction}")

        turning_right     = (line == "orange")
        self._turn_servo  = TURN_SERVO_RIGHT if turning_right else TURN_SERVO_LEFT
        turn_name         = "RIGHT" if turning_right else "LEFT"
        yaw_delta         = RIGHT_TURN_YAW_DELTA if turning_right else LEFT_TURN_YAW_DELTA
        self.corners     += 1

        print(f"[NAV] Corner {self.corners}/{TOTAL_CORNERS} - turning {turn_name}  "
              f"(lap {self.lap+1}, corner {self.corner_in_lap}/4)  "
              f"start_yaw={yaw:.1f}°  target_yaw={(yaw + yaw_delta) % 360:.1f}°")

        if self.corners >= TOTAL_CORNERS:
            print("[NAV] 3 laps complete - stopping!")
            self.state = self.DONE
            self.comms.stop()
            return

        self.state        = self.TURNING
        self._turn_phase  = self.PHASE_ROTATE
        self._turn_start  = now
        self._target_yaw  = yaw + yaw_delta
        self.last_turn_err = yaw_delta


#  Draw overlay 
def draw_overlay(frame, telemetry, nav):
    h, w = frame.shape[:2]
    cv2.line(frame, (int(w * LEFT_ZONE),  0), (int(w * LEFT_ZONE),  h), (200, 200, 200), 1)
    cv2.line(frame, (int(w * RIGHT_ZONE), 0), (int(w * RIGHT_ZONE), h), (200, 200, 200), 1)

    tof = telemetry["tof"]
    hud = [
        f"State: {nav.state}{'/' + nav._turn_phase if nav._turn_phase else ''}   "
        f"Dir: {nav.direction or '?'}   Lap: {nav.lap+1}/3   "
        f"Corner: {nav.corner_in_lap}/4   Total: {nav.corners}/{TOTAL_CORNERS}",
        f"ToF(mm): F={tof[0]} L={tof[1]} R={tof[2]}",
        f"Yaw: {telemetry['yaw']:.1f}deg   Batt: {telemetry['batt_mv']/1000:.2f}V   Enc: {telemetry['enc']}",
    ]
    if nav.state == Navigator.TURNING and nav._turn_phase == Navigator.PHASE_ROTATE:
        hud.append(f"Turn err remaining: {nav.last_turn_err:+.1f}deg "
                    f"(target {nav._target_yaw % 360:.1f}deg)")

    colours = [(255, 255, 255)] * len(hud)

    if not telemetry.get("_fresh", True):
        hud.append("!! NO TELEMETRY FROM PICO - yaw/heading-hold/adaptive-turn disabled !!")
        colours.append((0, 0, 255))

    for i, (line, colour) in enumerate(zip(hud, colours)):
        cv2.putText(frame, line, (10, 28 + i * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)
    cv2.putText(frame, "WRO FE - Open Challenge (adaptive turn v2)", (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    return frame


#  Main 
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
))
picam2.set_controls({
    "AeEnable": True, "AwbEnable": True, "AwbMode": 0,
    "ExposureValue": 0.0, "Saturation": 1.4, "Sharpness": 1.5,
})
picam2.start()

comms    = PicoComms(SERIAL_PORT, BAUD_RATE)
steering = SteeringController(comms)
nav      = Navigator(comms, steering)
line_det = LineDetector()
show_debug = False

print("WRO FE Open Challenge v2 - 'q' quit  'd' debug masks")

_last_diag_print = 0.0
# before the main loop, after comms/steering are constructed:
print("[CAL] Sampling yaw while stationary...")
samples = []
t_end = time.time() + 0.3
while time.time() < t_end:
    t = comms.get_telemetry()
    if comms.telemetry_fresh():
        samples.append(t["yaw"])
    time.sleep(0.02)

if samples:
    steering.reset_target_yaw(sum(samples) / len(samples))
else:
    print("[CAL] WARNING: no fresh telemetry during calibration window")

comms.send(DIR_FWD, SPEED_NORMAL, SERVO_CENTER, LED_DRIVING)
try:
    comms.send(DIR_FWD, SPEED_NORMAL, SERVO_CENTER, LED_DRIVING)

    while True:
        frame     = picam2.capture_array()
        telemetry = comms.get_telemetry()
        fresh     = comms.telemetry_fresh()
        telemetry["_fresh"] = fresh   
        now_diag = time.time()
        if not fresh and (now_diag - _last_diag_print) > 2.0:
            s = comms.stats()
            if s["packets"] == 0 and s["raw_bytes"] == 0:
                print("[DIAG] No bytes received on serial at all - check wiring "
                      "(Pi RX<->Pico TX), SERIAL_PORT, baud rate, and that "
                      "ttyAMA0 isn't claimed by Bluetooth/serial console on the Pi.")
            elif s["packets"] == 0:
                print(f"[DIAG] Receiving bytes ({s['raw_bytes']} so far) but no valid "
                      f"packets parsed yet (crc_errs={s['crc_errs']}) - check baud rate "
                      f"and that the Pico's telemetry frame matches the expected format.")
            else:
                print(f"[DIAG] Telemetry stale for {s['age']:.1f}s "
                      f"(packets so far: {s['packets']}, crc_errs: {s['crc_errs']})")
            _last_diag_print = now_diag

        line, line_pct, roi_y = line_det.detect(frame)

        prev_state = nav.state
        nav.update(line, telemetry)
        if prev_state not in (Navigator.TURNING, Navigator.DONE):
            steering.update(telemetry, fresh)

        output = draw_overlay(frame.copy(), telemetry, nav)
        output = line_det.draw_roi(output, roi_y, line, line_pct)

        if show_debug:
            h_f      = frame.shape[0]
            roi      = cv2.cvtColor(frame[int(h_f * LINE_ROI_TOP):], cv2.COLOR_BGR2LAB)
            orange_m = cv2.inRange(roi, ORANGE_LOWER, ORANGE_UPPER)
            blue_m   = cv2.inRange(roi, BLUE_LOWER,   BLUE_UPPER)
            debug    = np.hstack([
                cv2.cvtColor(orange_m, cv2.COLOR_GRAY2BGR),
                cv2.cvtColor(blue_m,   cv2.COLOR_GRAY2BGR),
            ])
            cv2.imshow("Debug: ORANGE | BLUE", debug)

        if nav.state == Navigator.DONE:
            cv2.imshow("WRO FE Open", output)
            cv2.waitKey(3000)
            break

        cv2.imshow("WRO FE Open", output)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('d'):
            show_debug = not show_debug

finally:
    comms.stop()
    comms.close()
    picam2.stop()
    cv2.destroyAllWindows()
    print("Stopped.")
