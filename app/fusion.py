"""
fusion.py — S.A.F.A.R. Navigation & Sensor Fusion Engine

Integrates Extended Kalman Filtering (EKF), phone-to-vehicle coordinate frame
transformations, continuous state transitions (GNSS_LOCKED -> GNSS_WEAK -> TRANSITION ->
GNSS_DENIED -> RECOVERING -> RELOCKED), gradual error recovery decoupling, and component-wise
confidence scoring.
"""

import math
import numpy as np

from .geo import LocalFrame
from .data_utils import compute_features_at
from .ekf import ExtendedKalmanFilter


def phone_to_vehicle_frame(accel_x, accel_y, accel_z, grav_x, grav_y, grav_z):
    """
    Transforms phone-frame acceleration vector to vehicle-aligned frame using gravity vector.
    Assumes vehicle vertical axis aligns with gravity vector direction, and vehicle longitudinal axis
    is in the horizontal plane.
    """
    g_mag = math.hypot(grav_x, grav_y, grav_z)
    if g_mag < 1e-3:
        return accel_x, accel_y, accel_z

    # Gravity unit vector (Vehicle Z-down / Z-up reference)
    gz_unit = np.array([grav_x / g_mag, grav_y / g_mag, grav_z / g_mag])

    # Linear acceleration with gravity removed
    lin_acc = np.array([accel_x - grav_x, accel_y - grav_y, accel_z - grav_z])

    # Vertical linear acceleration (along gravity vector)
    a_vert_mag = float(np.dot(lin_acc, gz_unit))

    # Horizontal linear acceleration vector (in phone plane orthogonal to gravity)
    a_horiz_vec = lin_acc - a_vert_mag * gz_unit
    a_horiz_mag = float(np.linalg.norm(a_horiz_vec))

    # Forward acceleration component (projected)
    a_forward = a_horiz_mag if (lin_acc[1] * grav_z - lin_acc[2] * grav_y) >= 0 else -a_horiz_mag
    a_lateral = float(lin_acc[0])  # approx lateral

    return a_forward, a_lateral, a_vert_mag


def compute_confidence_score(mode, gps_accuracy=None, speed_kmh=0.0, pos_uncertainty_m=1.0, is_stationary=False):
    """
    Calculates overall Navigation Position Confidence Score (0% to 100%).
    Combines sensor status, GNSS accuracy, state estimation variance, and vehicle motion state.
    """
    if mode == "GNSS":
        if gps_accuracy is not None and gps_accuracy > 0:
            conf = max(40.0, 100.0 - (gps_accuracy * 1.2))
        else:
            conf = 95.0
    elif mode in ("TRANSITION", "DR"):
        base = 88.0
        # Decay confidence based on state uncertainty
        penalty = min(50.0, pos_uncertainty_m * 2.5)
        conf = max(15.0, base - penalty)
    elif mode == "RECOVERING":
        conf = 75.0
    else:
        conf = 20.0

    if is_stationary:
        conf = min(100.0, conf + 5.0)

    return round(float(conf), 1)


def naive_dead_reckoning(df, start_idx, end_idx, ref_frame: LocalFrame):
    """Double-integrate raw linear acceleration from start_idx to end_idx baseline."""
    lin_x = (df["accel_x"] - df["gravity_x"]).values[start_idx:end_idx]
    lin_y = (df["accel_y"] - df["gravity_y"]).values[start_idx:end_idx]
    yaw = df["orient_yaw"].values[start_idx:end_idx]
    dt = 0.1

    cy, sy = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
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
    AI-speed + EKF state estimation dead reckoning for offline trips.
    """
    model = model_bundle["model"]
    window = model_bundle["window"]

    calib_n = 50
    calib_lats = df["lat"].values[max(0, start_idx - calib_n):start_idx + 1]
    calib_lons = df["lon"].values[max(0, start_idx - calib_n):start_idx + 1]
    cx, cy = zip(*[ref_frame.to_xy(la, lo) for la, lo in zip(calib_lats, calib_lons)])
    cx, cy = np.array(cx), np.array(cy)

    dx_win = cx[-1] - cx[0]
    dy_win = cy[-1] - cy[0]
    dist = np.hypot(dx_win, dy_win)
    heading0 = math.atan2(dy_win, dx_win)

    time_span = (len(cx) - 1) * 0.1
    gps_speed_ms = (dist / time_span) if time_span > 0 else 0.0
    gps_speed_kmh = gps_speed_ms * 3.6
    gyro_bias = float(np.median(df["gyro_yaw"].values[max(0, start_idx - calib_n):start_idx]))

    pre_feats = [compute_features_at(df, i, window=window) for i in range(max(0, start_idx - calib_n), start_idx)]
    if len(pre_feats) > 0:
        pre_ai_speeds = model.predict(np.array(pre_feats))
        mean_pre_ai_speed = float(np.mean(pre_ai_speeds))
        if mean_pre_ai_speed > 5.0 and gps_speed_kmh > 5.0:
            speed_scale = max(0.5, min(2.5, gps_speed_kmh / mean_pre_ai_speed))
        else:
            speed_scale = 1.0
    else:
        speed_scale = 1.0

    feats = [compute_features_at(df, i, window=window) for i in range(start_idx, end_idx)]
    raw_speed_kmh = model.predict(np.array(feats))

    lat0, lon0 = df["lat"].values[start_idx], df["lon"].values[start_idx]
    x0, y0 = ref_frame.to_xy(lat0, lon0)

    n = end_idx - start_idx
    xs, ys = np.zeros(n), np.zeros(n)
    speed_out = np.zeros(n)
    dt = 0.1

    init_vx = gps_speed_ms * math.cos(heading0)
    init_vy = gps_speed_ms * math.sin(heading0)

    ekf = ExtendedKalmanFilter(initial_x=x0, initial_y=y0, initial_vx=init_vx, initial_vy=init_vy, initial_heading=heading0)
    ekf.x[5, 0] = gyro_bias

    accel_x = df["accel_x"].values[start_idx:end_idx]
    accel_y = df["accel_y"].values[start_idx:end_idx]
    accel_z = df["accel_z"].values[start_idx:end_idx]
    grav_x = df["gravity_x"].values[start_idx:end_idx]
    grav_y = df["gravity_y"].values[start_idx:end_idx]
    grav_z = df["gravity_z"].values[start_idx:end_idx]
    gyro_yaw = df["gyro_yaw"].values[start_idx:end_idx]

    current_speed_ms = gps_speed_ms

    for i in range(n):
        ax_v, ay_v, _ = phone_to_vehicle_frame(
            accel_x[i], accel_y[i], accel_z[i],
            grav_x[i], grav_y[i], grav_z[i]
        )
        a_forward = float(np.clip(ax_v, -3.0, 3.0))
        if abs(a_forward) < 0.15:
            a_forward = 0.0

        ekf.predict(ax_veh=a_forward, ay_veh=ay_v, gyro_yaw=gyro_yaw[i], dt=dt)

        v_ai_kmh = max(0.0, float(raw_speed_kmh[i]) * speed_scale)
        if v_ai_kmh < 5.0 and gps_speed_kmh > 5.0 and a_forward >= -0.3:
            v_eff_ai = gps_speed_kmh
        else:
            v_eff_ai = v_ai_kmh

        delta_v_phys_kmh = float(np.clip(a_forward, -2.0, 2.0)) * dt * 3.6
        candidate_kmh = current_speed_ms * 3.6 + delta_v_phys_kmh
        
        # Fuse candidate physics speed with AI speed prediction
        target_kmh = 0.85 * v_eff_ai + 0.15 * candidate_kmh
        target_kmh = float(np.clip(target_kmh, max(0.0, v_eff_ai - 15.0), v_eff_ai + 15.0))

        fused_speed_ms = max(0.0, target_kmh / 3.6)
        current_speed_ms = fused_speed_ms
        speed_out[i] = target_kmh

        ekf.update_ai_motion(ai_speed_ms=fused_speed_ms)

        # Straight-road heading stabilization when turn rate is near zero
        gyro_corr = gyro_yaw[i] - gyro_bias
        if abs(gyro_corr) < 0.015:
            h_err = math.atan2(math.sin(heading0 - ekf.x[4, 0]), math.cos(heading0 - ekf.x[4, 0]))
            if abs(h_err) < 0.25: # within ~14 degrees of road heading
                ekf.x[4, 0] += 0.02 * h_err

        st = ekf.get_state()
        xs[i], ys[i] = st["x"], st["y"]

    return xs, ys, speed_out


def ground_truth_positions(df, start_idx, end_idx, ref_frame: LocalFrame):
    lats = df["lat"].values[start_idx:end_idx]
    lons = df["lon"].values[start_idx:end_idx]
    xs, ys = zip(*[ref_frame.to_xy(la, lo) for la, lo in zip(lats, lons)])
    return np.array(xs), np.array(ys)


class OnlineFusionSession:
    """
    Unified incremental Navigation & Sensor Fusion Session using Extended Kalman Filter (EKF).
    """
    LONG_WINDOW = 30
    SHORT_WINDOW = 10

    # --- Stationary / zero-velocity detection thresholds ---
    # Linear (gravity-removed) accel magnitude typically stays under ~0.3-0.4 m/s^2
    # for hand jitter / a phone sitting still, vs clearly higher during any real
    # driving acceleration/braking/turning. Tune these two if the demo phone is
    # unusually shaky or unusually smooth.
    STATIONARY_ACCEL_MEAN_THRESH = 0.35   # m/s^2
    STATIONARY_ACCEL_STD_THRESH = 0.35    # m/s^2
    STATIONARY_GYRO_THRESH = 0.12         # rad/s (~7 deg/s)
    STATIONARY_GPS_SPEED_KMH = 2.0        # only used to corroborate IMU stillness while GNSS is locked

    # GPS fix quality gates
    MAX_TRUSTED_ACCURACY_M = 30.0         # beyond this, treat position/velocity updates as low-confidence
    MIN_FIX_DT_S = 0.25                   # guard against divide-by-tiny-dt on back-to-back fixes
    MAX_FIX_DT_S = 8.0                    # guard against huge dt after a long gap (treat as a fresh start instead)

    def __init__(self, model_bundle):
        self.model = model_bundle["model"]
        self.buf_lin_x, self.buf_lin_y, self.buf_lin_z = [], [], []
        self.buf_gyro_yaw, self.buf_gyro_pitch, self.buf_gyro_roll = [], [], []
        self.gps_track = []
        self.ref_frame = None

        self.mode = "GNSS"
        self.ekf = None
        self.speed_scale = 1.0
        self.last_confirmed_speed_ms = 0.0
        self.last_confirmed_heading = 0.0
        self.last_confirmed_x = 0.0
        self.last_confirmed_y = 0.0
        self.gyro_bias = 0.0
        self.recovery_alpha = 0.15  # Exponential recovery smoothing factor
        self.smoothed_speed_kmh = 0.0

        # Real GPS fix bookkeeping (used instead of assuming every 100ms tick is a new fix)
        self.last_gps_lat = None
        self.last_gps_lon = None
        self.last_gps_fix_time_ms = None

    def _push_imu(self, accel, gravity, gyro):
        self.buf_lin_x.append(accel["x"] - gravity["x"])
        self.buf_lin_y.append(accel["y"] - gravity["y"])
        self.buf_lin_z.append(accel["z"] - gravity["z"])
        self.buf_gyro_yaw.append(gyro["yaw"])
        self.buf_gyro_pitch.append(gyro["pitch"])
        self.buf_gyro_roll.append(gyro["roll"])
        maxlen = self.LONG_WINDOW + 5
        for buf in (self.buf_lin_x, self.buf_lin_y, self.buf_lin_z,
                    self.buf_gyro_yaw, self.buf_gyro_pitch, self.buf_gyro_roll):
            if len(buf) > maxlen:
                del buf[0]

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

    def _is_stationary(self):
        """
        IMU-only zero-velocity check: true when recent linear acceleration and
        gyro rate both look like 'sitting still / being held', not driving.
        Requires no GPS, so this is what actually catches 'holding the phone,
        not driving, GNSS forced off' -- the AI model and pure physics
        integration both drift on pure noise once you're a few seconds into an
        outage, so this acts as a hard floor.
        """
        if len(self.buf_lin_x) < self.SHORT_WINDOW:
            return False
        s = self.SHORT_WINDOW
        lin_x = np.array(self.buf_lin_x[-s:])
        lin_y = np.array(self.buf_lin_y[-s:])
        lin_z = np.array(self.buf_lin_z[-s:])
        gyaw = np.array(self.buf_gyro_yaw[-s:])
        accel_mag = np.sqrt(lin_x ** 2 + lin_y ** 2 + lin_z ** 2)
        accel_ok = (accel_mag.mean() < self.STATIONARY_ACCEL_MEAN_THRESH and
                    accel_mag.std() < self.STATIONARY_ACCEL_STD_THRESH)
        gyro_ok = float(np.abs(gyaw).mean()) < self.STATIONARY_GYRO_THRESH
        return bool(accel_ok and gyro_ok)

    @staticmethod
    def _filter_forward_accel(a_forward, deadzone=0.6, clip=3.0):
        """Shared accel conditioning used in BOTH the GNSS and DR branches.
        Previously only the DR branch clipped/deadzoned this, so hand-shake
        noise fed straight into the EKF predict step whenever GNSS was locked."""
        a_forward = float(np.clip(a_forward, -clip, clip))
        if abs(a_forward) < deadzone:
            a_forward = 0.0
        return a_forward

    def update(self, accel, gravity, gyro, gps=None, simulate_outage=False, dt=0.1):
        self._push_imu(accel, gravity, gyro)
        gps_ok = gps is not None and not simulate_outage

        # Transform IMU readings to vehicle frame
        ax_v, ay_v, az_v = phone_to_vehicle_frame(
            accel["x"], accel["y"], accel["z"],
            gravity["x"], gravity["y"], gravity["z"]
        )

        if gps_ok:
            if self.ref_frame is None:
                self.ref_frame = LocalFrame(gps["lat"], gps["lon"])

            x_gps, y_gps = self.ref_frame.to_xy(gps["lat"], gps["lon"])
            accuracy_m = gps.get("accuracy")
            if accuracy_m is None:
                accuracy_m = 5.0
            native_speed_ms = gps.get("speed")  # browser's own Doppler-derived speed, if it gave one
            fix_time_ms = gps.get("fix_time_ms")

            # Is this a genuinely NEW GPS fix, or the same cached fix being resent on
            # another 100ms IMU tick because the GPS chip itself updates far slower
            # than the IMU loop? Previously every tick was treated as a fresh fix and
            # differenced against the last one using an assumed 0.1s, which turned a
            # couple of meters of ordinary GPS jitter into 10-30 km/h of phantom speed.
            is_new_fix = (
                self.last_gps_lat is None or self.last_gps_lon is None or
                gps["lat"] != self.last_gps_lat or gps["lon"] != self.last_gps_lon or
                fix_time_ms is None or self.last_gps_fix_time_ms is None or
                fix_time_ms != self.last_gps_fix_time_ms
            )

            gnss_vx, gnss_vy, gnss_heading = None, None, None
            trusted_accuracy = accuracy_m <= self.MAX_TRUSTED_ACCURACY_M

            if is_new_fix:
                self.gps_track.append((x_gps, y_gps, fix_time_ms))
                if len(self.gps_track) > self.LONG_WINDOW:
                    self.gps_track.pop(0)

                real_dt = None
                if fix_time_ms is not None and self.last_gps_fix_time_ms is not None:
                    real_dt = (fix_time_ms - self.last_gps_fix_time_ms) / 1000.0

                if native_speed_ms is not None and native_speed_ms >= 0:
                    # Prefer the GPS chip's own Doppler speed: it measures velocity
                    # directly from frequency shift and is far less sensitive to a
                    # few meters of position jitter than differencing two fixes.
                    spd = float(native_speed_ms)
                    if len(self.gps_track) >= 2 and spd > 0.3:
                        (x0, y0, _), (x1, y1, _) = self.gps_track[-2], self.gps_track[-1]
                        gnss_heading = math.atan2(y1 - y0, x1 - x0)
                elif (trusted_accuracy and len(self.gps_track) >= 2 and
                      real_dt is not None and self.MIN_FIX_DT_S <= real_dt <= self.MAX_FIX_DT_S):
                    (x0, y0, _), (x1, y1, _) = self.gps_track[-2], self.gps_track[-1]
                    dx, dy = x1 - x0, y1 - y0
                    disp = math.hypot(dx, dy)
                    # Only trust the displacement as real motion (not GPS noise) once
                    # it clearly exceeds the fix's own accuracy radius.
                    if disp > max(2.0, accuracy_m * 1.5):
                        spd = disp / real_dt
                        gnss_heading = math.atan2(dy, dx)
                    else:
                        spd = 0.0
                else:
                    spd = None

                if spd is not None and spd > 0.3:
                    if gnss_heading is None and len(self.gps_track) >= 2:
                        (x0, y0, _), (x1, y1, _) = self.gps_track[-2], self.gps_track[-1]
                        gnss_heading = math.atan2(y1 - y0, x1 - x0)
                    if gnss_heading is not None:
                        gnss_vx = spd * math.cos(gnss_heading)
                        gnss_vy = spd * math.sin(gnss_heading)
                        self.last_confirmed_speed_ms = spd
                        self.last_confirmed_heading = gnss_heading
                elif spd is not None:
                    self.last_confirmed_speed_ms = 0.0

                self.last_gps_lat, self.last_gps_lon = gps["lat"], gps["lon"]
                if fix_time_ms is not None:
                    self.last_gps_fix_time_ms = fix_time_ms

            self.last_confirmed_x = x_gps
            self.last_confirmed_y = y_gps

            if self.ekf is None:
                init_h = gnss_heading if gnss_heading is not None else 0.0
                init_vx = gnss_vx if gnss_vx is not None else 0.0
                init_vy = gnss_vy if gnss_vy is not None else 0.0
                self.ekf = ExtendedKalmanFilter(
                    initial_x=x_gps, initial_y=y_gps,
                    initial_vx=init_vx, initial_vy=init_vy,
                    initial_heading=init_h
                )

            # Check if recovering from DR outage -> perform smooth gradual alignment
            if self.mode in ("DR", "TRANSITION"):
                self.mode = "RECOVERING"
            elif self.mode == "RECOVERING":
                st = self.ekf.get_state()
                err = math.hypot(st["x"] - x_gps, st["y"] - y_gps)
                if err < 1.5:
                    self.mode = "GNSS"

            # Scale GNSS measurement noise by reported accuracy: a coarse fix should
            # correct the filter gently, not yank it around.
            r_pos = max(0.5, (accuracy_m / 3.0) ** 2)
            R_gnss = np.diag([r_pos, r_pos])

            if self.mode == "RECOVERING":
                st = self.ekf.get_state()
                corr_x = x_gps - st["x"]
                corr_y = y_gps - st["y"]
                self.ekf.update_gnss(st["x"] + self.recovery_alpha * corr_x, st["y"] + self.recovery_alpha * corr_y, gnss_vx=gnss_vx, gnss_vy=gnss_vy, gnss_heading=gnss_heading, R_gnss=R_gnss)
            else:
                self.mode = "GNSS"
                self.ekf.update_gnss(x_gps, y_gps, gnss_vx=gnss_vx, gnss_vy=gnss_vy, gnss_heading=gnss_heading, R_gnss=R_gnss)

            # Predict EKF state -- filter the accel the same way the DR branch does,
            # so phone hand-shake doesn't inject spurious velocity between fixes.
            a_forward = self._filter_forward_accel(ax_v)
            self.ekf.predict(ax_veh=a_forward, ay_veh=ay_v, gyro_yaw=gyro["yaw"], dt=dt)
            st = self.ekf.get_state()

            is_stationary = self._is_stationary()
            speed_kmh = st["speed_kmh"]
            # last_confirmed_speed_ms only gets bumped above ~1 m/s when a GPS fix
            # (native Doppler speed, or a displacement clearly bigger than the fix's
            # own accuracy radius) actually confirmed real motion -- so if it's near
            # zero, GPS agrees nothing is happening and IMU stillness can be trusted
            # to override any jitter that leaked into the EKF's velocity. If GPS
            # HAS recently confirmed real motion (e.g. smooth constant-speed
            # cruising, where accel is legitimately near zero too), don't stomp it.
            if is_stationary and self.last_confirmed_speed_ms < 1.0:
                speed_kmh = 0.0
                self.ekf.x[2, 0] = 0.0
                self.ekf.x[3, 0] = 0.0
                self.last_confirmed_speed_ms = 0.0

            lat, lon = self.ref_frame.to_latlon(st["x"], st["y"])
            conf = compute_confidence_score(self.mode, gps_accuracy=accuracy_m, speed_kmh=speed_kmh, pos_uncertainty_m=st["pos_uncertainty_m"], is_stationary=is_stationary)

            return {
                "mode": self.mode,
                "lat": round(lat, 7),
                "lon": round(lon, 7),
                "speed_kmh": round(speed_kmh, 1),
                "heading_deg": round(math.degrees(st["heading"]) % 360, 1),
                "confidence": conf,
                "pos_uncertainty_m": round(st["pos_uncertainty_m"], 2)
            }

        # --- GNSS Outage / Dead Reckoning (DR) ---
        if self.mode in ("GNSS", "RECOVERING"):
            self.mode = "TRANSITION"
            if len(self.gps_track) >= 5 and self.ref_frame is not None:
                (x0, y0, t0), (x1, y1, t1) = self.gps_track[0], self.gps_track[-1]
                gps_dist = math.hypot(x1 - x0, y1 - y0)
                if t0 is not None and t1 is not None and t1 > t0:
                    gps_dt = (t1 - t0) / 1000.0
                else:
                    # No fix timestamps available (older client) -- fall back to the
                    # last known confirmed speed rather than guessing a timestep.
                    gps_dt = None
                gps_speed_kmh = (gps_dist / gps_dt) * 3.6 if gps_dt and gps_dt > 0 else self.last_confirmed_speed_ms * 3.6

                if len(self.buf_lin_x) >= self.LONG_WINDOW:
                    ai_pred = float(self.model.predict(self._current_features())[0])
                    if ai_pred > 5.0 and gps_speed_kmh > 5.0:
                        self.speed_scale = max(0.5, min(2.5, gps_speed_kmh / ai_pred))
                    else:
                        self.speed_scale = 1.0

            # Re-align EKF velocity and heading to last confirmed GNSS state at transition entry
            if self.ekf is not None:
                init_vx = self.last_confirmed_speed_ms * math.cos(self.last_confirmed_heading)
                init_vy = self.last_confirmed_speed_ms * math.sin(self.last_confirmed_heading)
                self.ekf.x[0, 0] = self.last_confirmed_x
                self.ekf.x[1, 0] = self.last_confirmed_y
                self.ekf.x[2, 0] = init_vx
                self.ekf.x[3, 0] = init_vy
                self.ekf.x[4, 0] = self.last_confirmed_heading

        self.mode = "DR"

        if self.ekf is None:
            if self.ref_frame is None:
                return {"mode": "NO_FIX", "lat": None, "lon": None, "speed_kmh": None, "confidence": 0.0}
            init_vx = self.last_confirmed_speed_ms * math.cos(self.last_confirmed_heading)
            init_vy = self.last_confirmed_speed_ms * math.sin(self.last_confirmed_heading)
            self.ekf = ExtendedKalmanFilter(
                initial_x=self.last_confirmed_x, initial_y=self.last_confirmed_y,
                initial_vx=init_vx, initial_vy=init_vy,
                initial_heading=self.last_confirmed_heading
            )

        # Forward acceleration filtering (shared with the GNSS branch)
        a_forward = self._filter_forward_accel(ax_v)

        # Predict forward with EKF
        self.ekf.predict(ax_veh=a_forward, ay_veh=ay_v, gyro_yaw=gyro["yaw"], dt=dt)

        is_stationary = self._is_stationary()

        if is_stationary:
            # No GPS to lean on during an outage, so IMU stillness is the primary
            # signal here: gyro + linear accel both near zero for a sustained window
            # means the phone is being held / sitting still, not driving. This is
            # exactly the "force GNSS outage while not actually moving" case that
            # was previously showing 40-50 km/h carried over from noisy GNSS-mode
            # speed estimates.
            v_eff_ai = 0.0
            candidate_kmh = 0.0
            target_kmh = 0.0
            self.smoothed_speed_kmh = max(0.0, self.smoothed_speed_kmh * 0.5)
            if self.smoothed_speed_kmh < 0.5:
                self.smoothed_speed_kmh = 0.0
            self.ekf.x[2, 0] *= 0.5
            self.ekf.x[3, 0] *= 0.5
        else:
            # Predict speed via trained AI model & physics fusion
            raw_speed = float(self.model.predict(self._current_features())[0]) if len(self.buf_lin_x) >= self.LONG_WINDOW else 0.0
            v_ai_kmh = max(0.0, raw_speed * self.speed_scale) if len(self.buf_lin_x) >= self.LONG_WINDOW else self.last_confirmed_speed_ms * 3.6

            last_spd_kmh = self.last_confirmed_speed_ms * 3.6
            curr_speed_ms = float(math.hypot(self.ekf.x[2, 0], self.ekf.x[3, 0]))
            curr_speed_kmh = curr_speed_ms * 3.6

            # Anti-flicker logic: if we were mostly stationary, ignore random AI speed spikes from hand motion
            # Make the condition much stricter to prevent 20-30kmph jumps
            if last_spd_kmh < 3.0 and curr_speed_kmh < 10.0 and abs(a_forward) < 2.0:
                v_eff_ai = 0.0
                candidate_kmh = 0.0
            else:
                if v_ai_kmh < 5.0 and last_spd_kmh > 5.0 and a_forward >= -0.3:
                    v_eff_ai = last_spd_kmh
                else:
                    v_eff_ai = v_ai_kmh

                delta_v_phys_kmh = float(np.clip(a_forward, -2.0, 2.0)) * dt * 3.6
                candidate_kmh = curr_speed_kmh + delta_v_phys_kmh

            target_kmh = 0.85 * v_eff_ai + 0.15 * candidate_kmh
            target_kmh = float(np.clip(target_kmh, max(0.0, v_eff_ai - 15.0), v_eff_ai + 15.0))

            # Exponential smoothing to prevent rapid jumps
            self.smoothed_speed_kmh = 0.7 * self.smoothed_speed_kmh + 0.3 * target_kmh

        fused_speed_ms = max(0.0, self.smoothed_speed_kmh / 3.6)
        fused_speed_kmh = fused_speed_ms * 3.6

        # Update EKF with AI predicted speed & NHC constraint
        self.ekf.update_ai_motion(ai_speed_ms=fused_speed_ms)

        # Straight-road heading stabilization when turn rate is near zero
        gyro_corr = gyro["yaw"] - float(self.ekf.x[5, 0])
        if abs(gyro_corr) < 0.015:
            h_err = math.atan2(math.sin(self.last_confirmed_heading - self.ekf.x[4, 0]), math.cos(self.last_confirmed_heading - self.ekf.x[4, 0]))
            if abs(h_err) < 0.25: # within ~14 degrees of initial road course
                self.ekf.x[4, 0] += 0.02 * h_err

        st = self.ekf.get_state()
        lat, lon = self.ref_frame.to_latlon(st["x"], st["y"])
        conf = compute_confidence_score("DR", speed_kmh=fused_speed_kmh, pos_uncertainty_m=st["pos_uncertainty_m"], is_stationary=is_stationary)

        return {
            "mode": "DR",
            "lat": round(lat, 7),
            "lon": round(lon, 7),
            "speed_kmh": round(fused_speed_kmh, 1),
            "heading_deg": round(math.degrees(st["heading"]) % 360, 1),
            "confidence": conf,
            "pos_uncertainty_m": round(st["pos_uncertainty_m"], 2)
        }