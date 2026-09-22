# S.A.F.A.R. — Sensor-Aided Fusion for Accurate Routing
**AI-ML Based Intelligent Dead Reckoning (IDR) System for Seamless Navigation**  
*Smart India Hackathon 2026 | Problem Statement ID: SIH 26168 | Team DietCode*

---

## Overview

S.A.F.A.R. is an edge-deployable software engine and web application that transforms a smartphone into an **Intelligent Dead Reckoning (IDR)** system with GNSS Fusion. When GNSS outages occur (in tunnels, urban canyons, dense forests, or during jamming), S.A.F.A.R. instantly transitions to inertial tracking, maintaining lane-level accuracy without requiring any physical connection to the vehicle OBD-II/CAN bus port.

### Key Capabilities (SIH 26168 Requirements Met)
1. **In-Vehicle Alignment & Calibration Engine**:
   - Projects 3D gyroscope and linear acceleration onto Earth-vertical (gravity) and forward/lateral vehicle axes.
   - Extracts true vehicle horizontal yaw rate around the gravity axis ($\omega_{\text{turn}} = \vec{\omega} \cdot \hat{z}_{\text{down}}$), eliminating phone mounting angle errors.
2. **Disturbance & Confidence Model**:
   - **Zero-Velocity Update (ZUPT)**: Locks speed to `0.0 km/h` and drift to `0.0 m` when stationary (at traffic lights, parked, or resting on a desk).
   - **Hand-Disturbance Rejection**: Detects unconstrained 3D rotation rates ($\|\vec{\omega}_{\text{tilt}}\| > 0.35\text{ rad/s}$) and sudden wrist jerks, preventing artificial 40–50 km/h speed spikes.
   - Computes dynamic **Navigation Confidence (0–100%)**.
3. **Physics-Constrained AI Velocity Estimator**:
   - Bounded by physical vehicle acceleration limits ($|dv/dt| \le 3.0\text{ m/s}^2$).
   - Seamless hand-off from last confirmed GNSS speed when entering a blackout.
4. **Non-Holonomic Constraints (NHC) & Dynamic Heading**:
   - Enforces $v_{\text{lateral}} = 0, v_{\text{vertical}} = 0$, advancing position along vehicle heading.
   - Dynamic heading tracking around turns via calibrated gyro turn rate.
5. **Real-time Navigation HUD**:
   - Displays Speed, Compass Heading, Navigation Confidence, and Motion State in real-time.

---

## Two Operating Modes

- **Live Navigation Mode**: Uses native smartphone IMU (`devicemotion`) and GPS (`geolocation`). When GPS is available, displays live GNSS telemetry; when GPS drops or "Force GNSS Outage" is toggled, switches seamlessly to S.A.F.A.R. Dead Reckoning.
- **Dataset Replay Mode**: Steps through real benchmark trips from the **IO-VNBD dataset** (e.g. `Vta01a` 60s highway blackout) and visualizes Ground Truth vs Naive Double-Integration vs S.A.F.A.R. AI-Fused trajectory.

---

## Deploying to Render / Cloud

The repository is configured as a Render Blueprint:

1. Push this folder to the root of your GitHub repository.
2. In Render: **New → Blueprint**, select your repo (it reads `render.yaml`).
3. Deploy! Health check endpoint is `GET /api/health`.

### Manual Web Service Configuration
- **Runtime**: Python 3
- **Build command**: `pip install --upgrade pip && pip install -r requirements.txt`
- **Start command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT --workers 1`
- **Health check path**: `/api/health`
- **Python version**: `3.12.8`

---

## Running Locally

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
uvicorn app.main:app --reload --port 8000
```
Open `http://localhost:8000/`.

> **Note on Live Mode on mobile**: Mobile browsers only grant IMU sensor and geolocation permissions over **HTTPS** (or on `localhost`). On Render (which provides free HTTPS), live mobile sensors work seamlessly out of the box.

---

## Retraining & Evaluation Scripts

```bash
pip install -r requirements-dev.txt
python train_model.py     # Trains model on diverse IO-VNBD trips + disturbance rejection
python offline_eval.py    # Simulates 60s blackout and generates drift_comparison.png
```

---

## Repository Structure

```
app/
  main.py             FastAPI application routes and telemetry schemas
  fusion.py           S.A.F.A.R. sensor fusion & dead reckoning engine
  data_utils.py       IO-VNBD dataset loading + feature engineering
  geo.py              Local ENU frame conversion
  model.py            Lazy model loader
  replay.py           Replay trip runner
  replay_data/        Bundled test trip CSVs
  trained_model.joblib Trained scikit-learn model bundle
static/
  index.html          S.A.F.A.R. navigation HUD interface
  style.css           Dark telemetry theme styling
  app.js              Client-side sensor acquisition and Leaflet mapping
train_model.py        Multi-trip model training script
offline_eval.py       Benchmark evaluation and drift plotting
requirements.txt      Production dependencies
render.yaml           Render Blueprint configuration
```
