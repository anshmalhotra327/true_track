"""
fusion.py — S.A.F.A.R. (Sensor-Aided Fusion for Accurate Routing)
Intelligent Dead Reckoning and Sensor Fusion Engine (SIH 26168).

Key Modules:
1. In-Vehicle Alignment & Calibration Engine:
   - Projects 3D gyroscope and linear acceleration onto Earth-vertical (gravity)
     and vehicle forward/lateral axes.
   - Extracts true vehicle horizontal yaw rate around the gravity axis, independent
     of phone pitch/roll mounting angle.
2. Disturbance / Confidence Model:
   - Identifies STATIONARY (Zero-Velocity Update, ZUPT) to prevent indoor/desk drift.
   - Quarantines HAND_DISTURBANCE (accidental tilts, hand jerks, gestures) where
     erratic 3D rotation rates occur without vehicle kinematics, preventing 40-50 km/h spikes.
   - Confirms VEHICLE_DRIVING when forward dynamics and road texture match driving kinematics.
   - Computes live Confidence Level (0 - 100%).
3. Physics-Constrained AI Speed Estimator:
   - Bounded by road vehicle acceleration limits (|dv/dt| <= 3.0 m/s^2).
   - Seamless hand-off from last confirmed GNSS speed when entering a blackout.
4. Non-Holonomic Constraints (NHC) & Dynamic Heading:
   - Enforces v_lateral = 0, v_vertical = 0.
   - Updates vehicle heading during outages via calibrated turn rate.
5. Naive Dead Reckoning Baseline:
   - Double-integration baseline kept for benchmark comparison.
"""
import time
import math
import numpy as np

from .geo import LocalFrame
from .data_utils import compute_features_at


def _heading_rotation_naive(orient_yaw_deg, orient_pitch_deg, orient_roll_deg):
    """Rough Euler rotation (device frame -> world-ish frame) used ONLY for the
    naive baseline, to demonstrate how quickly uncorrected integration drifts."""
    yaw, pitch, roll = np.radians(orient_yaw_deg), np.radians(orient_pitch_deg), np.radians(orient_roll_deg)
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    return cy, sy, cp, sp, cr, sr


def naive_dead_reckoning(df, start_idx, end_idx, ref_frame: LocalFrame):
    """Double-integrate raw linear acceleration from start_idx to end_idx.
    Returns array of (x, y) positions in local meters, one per sample."""
    lin_x = (df["accel_x"] - df["gravity_x"]).values[start_idx:end_idx]
    lin_y = (df["accel_y"] - df["gravity_y"]).values[start_idx:end_idx]
    yaw = df["orient_yaw"].values[start_idx:end_idx]
    pitch = df["orient_pitch"].values[start_idx:end_idx]
    roll = df["orient_roll"].values[start_idx:end_idx]
    dt = 0.1

    cy, sy, cp, sp, cr, sr = _heading_rotation_naive(yaw, pitch, roll)
    world_ax = cy * lin_x - sy * lin_y
    world_ay = sy * lin_x + cy * lin_y

    lat0, lon0 = df["lat"].values[start_idx], df["lon"].values[start_idx]
    lat1, lon1 = df["lat"].values[start_idx + 5], df["lon"].values[start_idx + 5]
    x0, y0 = ref_frame.to_xy(lat0, lon0)
    x1, y1 = ref_frame.to_xy(lat1, lon1)
    vx, vy = (x1 - x0) / (5 * dt), (y1 - y0) / (5 * dt)

    n = end_idx - start_idx
    xs, ys = np.zeros(n), np.zeros(n)
    x, y = x0, y0
    for i in range(n):
        vx += world_ax[i] * dt
        vy += world_ay[i] * dt
        x += vx * dt
        y += vy * dt
        xs[i], ys[i] = x, y
    return xs, ys


def ai_fused_dead_reckoning(df, start_idx, end_idx, ref_frame: LocalFrame, model_bundle):
    """
    S.A.F.A.R. AI-Fused Dead Reckoning (batch mode for evaluation & replay).
    - Starts with seamless hand-off of entry speed from GNSS.
    - Uses calibrated gravity-projected turn rate to track real heading changes.
    - AI velocity model blends with forward kinematics and NHC constraints.
    """
    model = model_bundle["model"]
    window = model_bundle["window"]
    n = end_idx - start_idx
    dt = 0.1

    # Calibrate initial heading from pre-blackout GPS trajectory (2s baseline)
    calib_n = min(30, start_idx)
    calib_lats = df["lat"].values[start_idx - calib_n:start_idx + 1]
    calib_lons = df["lon"].values[start_idx - calib_n:start_idx + 1]
    cx, cy = zip(*[ref_frame.to_xy(la, lo) for la, lo in zip(calib_lats, calib_lons)])
    cx, cy = np.array(cx), np.array(cy)
    heading0 = math.atan2(cy[-1] - cy[0], cx[-1] - cx[0])

    # Calibrate gyro turn bias over the pre-blackout period
    # Turn rate is projected onto gravity unit vector (Earth vertical down)
    warmup_gyros = np.column_stack([
        df["gyro_pitch"].values[start_idx - calib_n:start_idx],
        df["gyro_roll"].values[start_idx - calib_n:start_idx],
        df["gyro_yaw"].values[start_idx - calib_n:start_idx]
    ])
    warmup_gravs = np.column_stack([
        df["gravity_x"].values[start_idx - calib_n:start_idx],
        df["gravity_y"].values[start_idx - calib_n:start_idx],
        df["gravity_z"].values[start_idx - calib_n:start_idx]
    ])
    warmup_gu = warmup_gravs / np.linalg.norm(warmup_gravs, axis=1, keepdims=True)
    turn_bias = float(np.mean(np.sum(warmup_gyros * warmup_gu, axis=1)))

    # Turn rates during blackout
    blackout_gyros = np.column_stack([
        df["gyro_pitch"].values[start_idx:end_idx],
        df["gyro_roll"].values[start_idx:end_idx],
        df["gyro_yaw"].values[start_idx:end_idx]
    ])
    blackout_gravs = np.column_stack([
        df["gravity_x"].values[start_idx:end_idx],
        df["gravity_y"].values[start_idx:end_idx],
        df["gravity_z"].values[start_idx:end_idx]
    ])
    blackout_gu = blackout_gravs / np.linalg.norm(blackout_gravs, axis=1, keepdims=True)
    turn_rates = np.sum(blackout_gyros * blackout_gu, axis=1) - turn_bias

    # Compute AI speed predictions
    feats = []
    for i in range(start_idx, end_idx):
        feats.append(compute_features_at(df, i, window=window))
    X = np.array(feats)
    raw_ai_speeds = np.maximum(0.0, model.predict(X))

    # Entry speed from GPS
    entry_speed_kmh = float(df["speed_kmh"].values[start_idx])
    v_kmh = entry_speed_kmh
    max_delta_kmh = 3.0 * 3.6 * dt  # max vehicle acceleration ~3 m/s^2

    lat0, lon0 = df["lat"].values[start_idx], df["lon"].values[start_idx]
    x0, y0 = ref_frame.to_xy(lat0, lon0)

    xs, ys = np.zeros(n), np.zeros(n)
    speed_kmh_out = np.zeros(n)
    x, y = x0, y0
    heading = heading0

    for i in range(n):
        # Update heading with deadband
        omega = turn_rates[i]
        if abs(omega) > 0.005:  # ~0.3 deg/s deadband
            heading += omega * dt

        # Smooth kinematic speed blend with AI estimate
        ai_target = raw_ai_speeds[i]
        delta = ai_target - v_kmh
        delta = max(-max_delta_kmh * 2.0, min(max_delta_kmh, delta))
        v_kmh = max(0.0, v_kmh + delta)
        speed_kmh_out[i] = v_kmh

        # Non-Holonomic Constraint (NHC): advance along heading
        speed_ms = v_kmh / 3.6
        x += speed_ms * math.cos(heading) * dt
        y += speed_ms * math.sin(heading) * dt
        xs[i], ys[i] = x, y

    return xs, ys, speed_kmh_out


def ground_truth_positions(df, start_idx, end_idx, ref_frame: LocalFrame):
    lats = df["lat"].values[start_idx:end_idx]
    lons = df["lon"].values[start_idx:end_idx]
    xs, ys = zip(*[ref_frame.to_xy(la, lo) for la, lo in zip(lats, lons)])
    return np.array(xs), np.array(ys)


class OnlineFusionSession:
    """
    S.A.F.A.R. Real-time Incremental Fusion Session (10Hz streaming).
    Processes live phone sensor streams (accel, gravity, gyro, gps) and produces
    drift-resistant position, speed, compass heading, confidence, and motion classification.
    """
    LONG_WINDOW = 30
    SHORT_WINDOW = 10

    def __init__(self, model_bundle):
        self.model = model_bundle["model"]
        self.buf_lin_x, self.buf_lin_y, self.buf_lin_z = [], [], []
        self.buf_gyro_yaw, self.buf_gyro_pitch, self.buf_gyro_roll = [], [], []
        self.buf_grav_x, self.buf_grav_y, self.buf_grav_z = [], [], []

        self.gps_track = []  # list of (x, y) in local meters
        self.ref_frame = None
        self.mode = "GNSS"
        self.motion_state = "STATIONARY"  # STATIONARY, HAND_DISTURBANCE, VEHICLE_DRIVING
        self.confidence = 95

        self.dr_x = self.dr_y = None
        self.dr_heading = 0.0
        self.speed_kmh = 0.0
        self.last_gps_speed_kmh = 0.0
        self.last_gps_raw = None
        self.last_gps_fix_time = 0.0
        self.last_gps_fix_xy = None
        self.turn_bias = 0.0
        self.warmup_turn_rates = []
        self.disturbance_cooldown = 0

    def _push_imu(self, accel, gravity, gyro):
        lx = accel["x"] - gravity["x"]
        ly = accel["y"] - gravity["y"]
        lz = accel["z"] - gravity["z"]

        self.buf_lin_x.append(lx)
        self.buf_lin_y.append(ly)
        self.buf_lin_z.append(lz)
        self.buf_gyro_yaw.append(gyro["yaw"])
        self.buf_gyro_pitch.append(gyro["pitch"])
        self.buf_gyro_roll.append(gyro["roll"])
        self.buf_grav_x.append(gravity["x"])
        self.buf_grav_y.append(gravity["y"])
        self.buf_grav_z.append(gravity["z"])

        maxlen = self.LONG_WINDOW + 10
        for buf in (self.buf_lin_x, self.buf_lin_y, self.buf_lin_z,
                    self.buf_gyro_yaw, self.buf_gyro_pitch, self.buf_gyro_roll,
                    self.buf_grav_x, self.buf_grav_y, self.buf_grav_z):
            if len(buf) > maxlen:
                del buf[0]

    def _analyze_motion(self):
        """
        Disturbance / Confidence Model (Slide 3):
        Decomposes 3D gyro into Earth-vertical turn rate and orthogonal tilt disturbance.
        Detects STATIONARY (ZUPT), HAND_DISTURBANCE, or VEHICLE_DRIVING.
        """
        s = min(len(self.buf_lin_x), self.SHORT_WINDOW)
        if s < 3:
            return "STATIONARY", 90, 0.0, 0.0

        lx = np.array(self.buf_lin_x[-s:])
        ly = np.array(self.buf_lin_y[-s:])
        lz = np.array(self.buf_lin_z[-s:])
        gyaw = np.array(self.buf_gyro_yaw[-s:])
        gpitch = np.array(self.buf_gyro_pitch[-s:])
        groll = np.array(self.buf_gyro_roll[-s:])

        # Current gravity unit vector (Earth vertical down in phone body coordinates)
        gx, gy, gz = self.buf_grav_x[-1], self.buf_grav_y[-1], self.buf_grav_z[-1]
        g_norm = math.sqrt(gx**2 + gy**2 + gz**2)
        if g_norm < 1e-4:
            g_unit = np.array([0.0, 0.0, 1.0])
        else:
            g_unit = np.array([gx, gy, gz]) / g_norm

        # Gyro vector in body frame: [pitch, roll, yaw]
        gyro_vec = np.array([gpitch[-1], groll[-1], gyaw[-1]])

        # Vehicle turn rate around Earth vertical (gravity vector):
        turn_rate = float(np.dot(gyro_vec, g_unit))

        # Tilt / handling disturbance: component orthogonal to Earth vertical
        tilt_rate_vec = gyro_vec - turn_rate * g_unit
        tilt_rate_mag = float(np.linalg.norm(tilt_rate_vec))

        # Linear acceleration energy
        lin_mag = np.sqrt(lx**2 + ly**2 + lz**2)
        lin_std = float(lin_mag.std())
        gyro_mag = float(np.linalg.norm(gyro_vec))

        # Check gravity stability (is phone rotating in hand?)
        if len(self.buf_grav_x) >= 5:
            g_delta = math.sqrt(
                (gx - self.buf_grav_x[-5])**2 +
                (gy - self.buf_grav_y[-5])**2 +
                (gz - self.buf_grav_z[-5])**2
            )
        else:
            g_delta = 0.0

        # Classification logic:
        # 1. ZUPT (Stationary): very low linear variance and low rotation
        if lin_std < 0.25 and lin_mag.mean() < 0.45 and gyro_mag < 0.20:
            if self.speed_kmh < 2.5:
                self.speed_kmh = 0.0
                return "STATIONARY", 98, turn_rate, tilt_rate_mag
            else:
                # Cruising smoothly on a road/highway
                return "VEHICLE_DRIVING", 94, turn_rate, tilt_rate_mag

        # 2. Hand Disturbance / In-Hand Gestures:
        # If vehicle is stopped or moving very slowly (< 3.5 km/h) and user is violently shaking/flipping the phone
        if self.speed_kmh < 3.5:
            if gyro_mag > 1.2 or tilt_rate_mag > 1.0 or g_delta > 2.2:
                conf = max(25, int(90 - tilt_rate_mag * 20 - g_delta * 10))
                self.disturbance_cooldown = 10  # 1.0s cooldown
                return "HAND_DISTURBANCE", conf, turn_rate, tilt_rate_mag
        else:
            # Vehicle is in motion (bike/car): bumps and turns are NORMAL driving kinematics!
            # Only extreme tumble (phone dropped/spun) triggers disturbance
            if gyro_mag > 3.0 or g_delta > 4.5:
                self.disturbance_cooldown = 10
                return "HAND_DISTURBANCE", 40, turn_rate, tilt_rate_mag

        if self.disturbance_cooldown > 0:
            self.disturbance_cooldown -= 1
            if self.speed_kmh < 1.0:
                return "STATIONARY", 80, turn_rate, tilt_rate_mag

        # 3. Vehicle driving
        conf = min(96, max(75, int(96 - lin_std * 3)))
        return "VEHICLE_DRIVING", conf, turn_rate, tilt_rate_mag

    def _current_features(self):
        lin_x = np.array(self.buf_lin_x); lin_y = np.array(self.buf_lin_y); lin_z = np.array(self.buf_lin_z)
        accel_mag = np.sqrt(lin_x ** 2 + lin_y ** 2 + lin_z ** 2)
        gyaw = np.array(self.buf_gyro_yaw); gpitch = np.array(self.buf_gyro_pitch); groll = np.array(self.buf_gyro_roll)
        s, l = self.SHORT_WINDOW, self.LONG_WINDOW
        return np.array([
            lin_x[-s:].std(), lin_y[-s:].std(), lin_z[-s:].std(),
            accel_mag[-s:].mean(), accel_mag[-s:].max(),
            gyaw[-s:].std(), gpitch[-s:].mean(), groll[-s:].mean(),
            lin_z[-l:].std(), accel_mag[-l:].std(),
            np.abs(gyaw[-l:]).mean(), gyaw[-l:].std(),
        ]).reshape(1, -1)

    def update(self, accel, gravity, gyro, gps=None, simulate_outage=False, dt=0.1):
        """
        Processes one sample (10Hz).
        accel, gravity: {"x","y","z"} in m/s^2
        gyro: {"yaw","pitch","roll"} in rad/s
        gps: {"lat", "lon", optional "speed", optional "speed_kmh", optional "heading"} or None
        simulate_outage: bool
        dt: timestep (default 0.1s)
        """
        self._push_imu(accel, gravity, gyro)
        motion_state, conf, turn_rate, tilt_rate = self._analyze_motion()
        self.motion_state = motion_state
        self.confidence = conf

        gps_ok = (gps is not None) and (not simulate_outage)

        # -------------------------------------------------------------------
        # GNSS ACTIVE MODE
        # -------------------------------------------------------------------
        if gps_ok:
            if self.ref_frame is None:
                self.ref_frame = LocalFrame(gps["lat"], gps["lon"])
            x, y = self.ref_frame.to_xy(gps["lat"], gps["lon"])

            # Check if this sample contains a genuinely new GPS fix from hardware
            is_new_gps_fix = False
            if self.last_gps_raw is None:
                is_new_gps_fix = True
            elif (abs(gps["lat"] - self.last_gps_raw["lat"]) > 1e-7 or
                  abs(gps["lon"] - self.last_gps_raw["lon"]) > 1e-7):
                is_new_gps_fix = True

            # Determine GPS speed:
            target_speed_kmh = None

            # Priority 1: speed_kmh provided by frontend (computed from distinct GPS timestamps)
            if gps.get("speed_kmh") is not None and not math.isnan(gps["speed_kmh"]):
                target_speed_kmh = float(gps["speed_kmh"])
            # Priority 2: Browser coords.speed (in m/s)
            elif gps.get("speed") is not None and not math.isnan(gps["speed"]) and float(gps["speed"]) >= 0:
                target_speed_kmh = float(gps["speed"]) * 3.6
            # Priority 3: Derived from successive distinct GPS coordinates
            elif is_new_gps_fix and self.last_gps_fix_xy is not None:
                dx = x - self.last_gps_fix_xy[0]
                dy = y - self.last_gps_fix_xy[1]
                dist = math.hypot(dx, dy)
                now = time.time()
                dt_fix = now - self.last_gps_fix_time if self.last_gps_fix_time > 0 else 1.0
                if 0.4 <= dt_fix <= 10.0:
                    if dist > 1.2:
                        target_speed_kmh = (dist / dt_fix) * 3.6
                    else:
                        target_speed_kmh = 0.0

            if is_new_gps_fix:
                self.last_gps_raw = {"lat": gps["lat"], "lon": gps["lon"]}
                self.last_gps_fix_xy = (x, y)
                self.last_gps_fix_time = time.time()
                self.gps_track.append((x, y))
                if len(self.gps_track) > self.LONG_WINDOW:
                    self.gps_track.pop(0)

            # Update speed:
            if target_speed_kmh is not None:
                target_speed_kmh = min(160.0, max(0.0, target_speed_kmh))
                if target_speed_kmh < 0.8:
                    target_speed_kmh = 0.0
                if self.speed_kmh == 0.0:
                    self.speed_kmh = target_speed_kmh
                else:
                    self.speed_kmh = 0.5 * self.speed_kmh + 0.5 * target_speed_kmh
                self.last_gps_speed_kmh = self.speed_kmh
            else:
                # Same GPS fix held between 100ms sample ticks: MAINTAIN verified speed!
                pass

            if self.speed_kmh >= 3.0:
                self.motion_state = "VEHICLE_DRIVING"
                self.confidence = max(self.confidence, 92)
            elif self.speed_kmh == 0.0 and self.motion_state != "HAND_DISTURBANCE":
                self.motion_state = "STATIONARY"

            # Determine Heading
            if gps.get("heading") is not None and not math.isnan(gps["heading"]):
                self.dr_heading = math.radians(float(gps["heading"]))
            elif len(self.gps_track) >= 2:
                dx = self.gps_track[-1][0] - self.gps_track[0][0]
                dy = self.gps_track[-1][1] - self.gps_track[0][1]
                if math.hypot(dx, dy) > 1.5:  # require meaningful displacement
                    self.dr_heading = math.atan2(dy, dx)

            # Accumulate turn rates during GNSS to calibrate gyro bias
            self.warmup_turn_rates.append(turn_rate)
            if len(self.warmup_turn_rates) > 30:
                self.warmup_turn_rates.pop(0)
            if len(self.warmup_turn_rates) >= 10:
                self.turn_bias = float(np.median(self.warmup_turn_rates))

            self.dr_x, self.dr_y = x, y
            self.mode = "GNSS"

            return {
                "mode": "GNSS",
                "lat": gps["lat"],
                "lon": gps["lon"],
                "speed_kmh": round(self.speed_kmh, 1),
                "heading_deg": round(math.degrees(self.dr_heading) % 360, 1),
                "motion_state": self.motion_state,
                "confidence": self.confidence,
            }

        # -------------------------------------------------------------------
        # DEAD RECKONING MODE (GNSS OUTAGE / SIMULATED OUTAGE)
        # -------------------------------------------------------------------
        if self.mode == "GNSS":
            # Seamless transition from GNSS to Dead Reckoning
            if self.ref_frame is None:
                return {
                    "mode": "NO_FIX", "lat": None, "lon": None, "speed_kmh": 0.0,
                    "heading_deg": 0.0, "motion_state": self.motion_state, "confidence": 0
                }
            if self.dr_x is None:
                self.dr_x = self.gps_track[-1][0] if self.gps_track else 0.0
                self.dr_y = self.gps_track[-1][1] if self.gps_track else 0.0
            # Start DR speed exactly at the last confirmed GPS speed
            self.speed_kmh = self.last_gps_speed_kmh
            self.mode = "DR"

        # Speed Estimation & Kinematic Filtering
        if motion_state == "STATIONARY":
            # Zero-Velocity Update (ZUPT): strictly zero
            self.speed_kmh = 0.0
        elif motion_state == "HAND_DISTURBANCE":
            # Accidental hand movement / phone tilt: quarantine acceleration, decay speed
            decay = math.exp(-dt / 0.8)
            self.speed_kmh *= decay
            if self.speed_kmh < 0.5:
                self.speed_kmh = 0.0
        else:
            # VEHICLE_DRIVING: query the trained model
            if len(self.buf_lin_x) >= self.LONG_WINDOW:
                raw_pred = float(self.model.predict(self._current_features())[0])
                target_speed = max(0.0, raw_pred)
            else:
                target_speed = self.speed_kmh

            # Enforce physical vehicle acceleration limits: max ~3.0 m/s^2 (~1.08 km/h per 100ms)
            max_delta_kmh = 3.0 * 3.6 * dt
            delta = target_speed - self.speed_kmh
            delta = max(-max_delta_kmh * 2.0, min(max_delta_kmh, delta))
            self.speed_kmh = max(0.0, self.speed_kmh + delta)

        # Dynamic Heading Update (Vehicle Turn Rate)
        eff_turn_rate = turn_rate - self.turn_bias
        if motion_state == "VEHICLE_DRIVING" and abs(eff_turn_rate) > 0.015:  # ~0.8 deg/s deadband
            self.dr_heading += eff_turn_rate * dt
            # Keep normalized in [-pi, pi]
            self.dr_heading = (self.dr_heading + math.pi) % (2 * math.pi) - math.pi

        # Non-Holonomic Constraint (NHC): velocity advances strictly along heading
        speed_ms = self.speed_kmh / 3.6
        self.dr_x += speed_ms * math.cos(self.dr_heading) * dt
        self.dr_y += speed_ms * math.sin(self.dr_heading) * dt

        lat, lon = self.ref_frame.to_latlon(self.dr_x, self.dr_y)
        return {
            "mode": "DR",
            "lat": lat,
            "lon": lon,
            "speed_kmh": round(self.speed_kmh, 1),
            "heading_deg": round(math.degrees(self.dr_heading) % 360, 1),
            "motion_state": self.motion_state,
            "confidence": self.confidence,
        }
