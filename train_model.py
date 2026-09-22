"""
train_model.py
Trains the AI velocity model (the "virtual speedometer") on real IO-VNBD trips
with stationary (ZUPT) and hand-disturbance rejection augmentations.

Run: python train_model.py
"""
import sys
import joblib
import numpy as np
from pathlib import Path
from sklearn.ensemble import HistGradientBoostingRegressor
from sklearn.metrics import mean_absolute_error

sys.path.insert(0, str(Path(__file__).parent))
from app.data_utils import load_trip, windowed_features, FEATURE_NAMES

HERE = Path(__file__).parent
SAMPLES = HERE / "training_data"
CLONED = Path(r"c:\ps168\cloned\IO-VNBD\Synchronised V abd S datasets\Uncategorised IOVNB Dataset")

# Prefer the full cloned dataset if present, else fall back to bundled training_data
if (CLONED / "S-Dataset").is_dir():
    print(f"Using full IO-VNBD dataset from {CLONED}")
    s_dir = CLONED / "S-Dataset"
    v_dir = CLONED / "V-Dataset"
    TRAIN_TRIPS = [
        (s_dir / "S-S1.csv", v_dir / "V-S1.csv"),
        (s_dir / "S-M.csv", v_dir / "V-M.csv"),
        (s_dir / "S-Y1.csv", v_dir / "V-Y1.csv"),
        (s_dir / "S-Vfa01.csv", v_dir / "V-Vfa01.csv"),
        (s_dir / "S-Vw1.csv", v_dir / "V-Vw1.csv"),
    ]
    TEST_TRIP = (s_dir / "S-Vta1a.csv", v_dir / "V-Vta1a.csv")
else:
    print("Using bundled training_data trips...")
    TRAIN_TRIPS = [
        (SAMPLES / "S1/S1/S-S1.csv", SAMPLES / "S1/S1/V-S1.csv"),
        (SAMPLES / "M/M (Driver B)/S-M.csv", SAMPLES / "M/M (Driver B)/V-M.csv"),
    ]
    TEST_TRIP = (
        SAMPLES / "Vta/Vta (Driver E)/Vta01a/S-Vta1a.csv",
        SAMPLES / "Vta/Vta (Driver E)/Vta01a/V-Vta1a.csv",
    )

WINDOW = 10  # 1 second @ 10Hz


def build_dataset(trip_paths):
    X_all, y_all = [], []
    for s_path, v_path in trip_paths:
        if not s_path.is_file() or not v_path.is_file():
            print(f"Skipping missing file: {s_path.name}")
            continue
        print(f"Loading {s_path.name}...")
        df = load_trip(str(s_path), str(v_path))
        X, y, _ = windowed_features(df, window=WINDOW)
        X_all.append(X)
        y_all.append(y)
    return np.vstack(X_all), np.concatenate(y_all)


def generate_still_and_disturbance_samples(n_samples: int = 12000):
    """
    Synthesize stationary (desk / traffic light / idling) and hand disturbance
    (wrist jerks / waving / tilts) features, all with ground truth speed = 0.0 km/h.
    This guarantees the regression model learns that high erratic variance without
    vehicle kinematics corresponds to ZERO vehicle speed, rather than jumping to 50 km/h.
    """
    # 1. Stationary / Desk / Rest: near-zero vibration and gyro
    n_half = n_samples // 2
    still_feat = np.abs(np.random.normal(loc=0.03, scale=0.04, size=(n_half, len(FEATURE_NAMES))))
    still_y = np.zeros(n_half)

    # 2. Hand movement / Jerks / Tilts: high erratic variance, rotation rates, speed = 0
    jerk_feat = np.zeros((n_half, len(FEATURE_NAMES)))
    jerk_feat[:, 0] = np.random.uniform(0.8, 5.0, n_half)    # lin_x_std
    jerk_feat[:, 1] = np.random.uniform(0.8, 5.0, n_half)    # lin_y_std
    jerk_feat[:, 2] = np.random.uniform(0.8, 5.0, n_half)    # lin_z_std
    jerk_feat[:, 3] = np.random.uniform(1.5, 8.0, n_half)    # accel_mag_mean
    jerk_feat[:, 4] = np.random.uniform(3.0, 15.0, n_half)   # accel_mag_max
    jerk_feat[:, 5] = np.random.uniform(0.3, 3.0, n_half)    # gyro_yaw_std
    jerk_feat[:, 6] = np.random.uniform(-1.5, 1.5, n_half)   # gyro_pitch_mean
    jerk_feat[:, 7] = np.random.uniform(-1.5, 1.5, n_half)   # gyro_roll_mean
    jerk_feat[:, 8] = np.random.uniform(0.8, 5.0, n_half)    # lin_z_std_3s
    jerk_feat[:, 9] = np.random.uniform(0.8, 5.0, n_half)    # accel_mag_std_3s
    jerk_feat[:, 10] = np.random.uniform(0.3, 3.0, n_half)  # gyro_yaw_absmean_3s
    jerk_feat[:, 11] = np.random.uniform(0.3, 3.0, n_half)  # gyro_yaw_std_3s
    jerk_y = np.zeros(n_half)

    X_dist = np.vstack([still_feat, jerk_feat])
    y_dist = np.concatenate([still_y, jerk_y])
    return X_dist, y_dist


def main():
    print("Loading training trips...")
    X_train, y_train = build_dataset(TRAIN_TRIPS)
    print(f"Train set (driving): {X_train.shape[0]} samples, {X_train.shape[1]} features")

    print("Synthesizing stationary and hand-disturbance rejection samples (speed = 0)...")
    X_dist, y_dist = generate_still_and_disturbance_samples(n_samples=24000)
    X_train_aug = np.vstack([X_train, X_dist])
    y_train_aug = np.concatenate([y_train, y_dist])
    print(f"Total augmented training set: {X_train_aug.shape[0]} samples")

    print(f"Loading held-out test trip ({TEST_TRIP[0].name}, never seen during training)...")
    X_test, y_test = build_dataset([TEST_TRIP])
    print(f"Test set: {X_test.shape[0]} samples")

    print("Fitting HistGradientBoostingRegressor...")
    model = HistGradientBoostingRegressor(
        max_iter=250, max_depth=8, learning_rate=0.08,
        l2_regularization=0.2, random_state=42,
    )
    model.fit(X_train_aug, y_train_aug)

    pred_train = model.predict(X_train)
    pred_test = model.predict(X_test)
    mae_train = mean_absolute_error(y_train, pred_train)
    mae_test = mean_absolute_error(y_test, pred_test)

    print(f"\nTrain MAE: {mae_train:.2f} km/h")
    print(f"Held-out ({TEST_TRIP[0].name}) MAE: {mae_test:.2f} km/h  <-- honest generalization")

    # Sanity check on hand jerk features
    jerk_test = np.array([[2.5, 2.5, 2.5, 4.0, 9.0, 1.2, 0.2, 0.2, 2.0, 2.0, 1.0, 0.8]])
    print(f"Sanity check - Hand jerk prediction: {max(0.0, float(model.predict(jerk_test)[0])):.1f} km/h (Expected: ~0.0 km/h)")

    out_path = HERE / "app" / "trained_model.joblib"
    joblib.dump({"model": model, "window": WINDOW, "feature_names": FEATURE_NAMES}, out_path)
    print(f"\nSaved updated model bundle to {out_path}")


if __name__ == "__main__":
    main()
