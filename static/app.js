// ============================================================
// S.A.F.A.R. — Sensor-Aided Fusion for Accurate Routing
// Intelligent Dead Reckoning & Navigation Engine (SIH 26168)
// Team DietCode
// ============================================================

// ------------------------------------------------------------
// 1. Map & Custom Marker Initialization
// ------------------------------------------------------------
const DEFAULT_CENTER = [28.6139, 77.2090]; // New Delhi default
const map = L.map('map', {
  zoomControl: false,
  attributionControl: false,
}).setView(DEFAULT_CENTER, 15);

// OpenStreetMap Standard Tiles
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
  attribution: '&copy; OpenStreetMap | S.A.F.A.R. Team DietCode',
}).addTo(map);

L.control.attribution({ position: 'bottomright', prefix: 'S.A.F.A.R.' }).addTo(map);

// Google Maps Style Blue Dot Marker with 110px Flashlight Heading Beam Cone
const userLocationIcon = L.divIcon({
  className: 'gmaps-marker-wrapper',
  html: `
    <div class="gmaps-user-marker">
      <div class="gmaps-heading-cone" id="gmaps-cone">
        <svg viewBox="0 0 110 110" width="110" height="110">
          <defs>
            <radialGradient id="beamGrad" cx="55" cy="55" r="50" fx="55" fy="55" gradientUnits="userSpaceOnUse">
              <stop offset="0%" stop-color="#1a73e8" stop-opacity="0.80"/>
              <stop offset="35%" stop-color="#2979ff" stop-opacity="0.45"/>
              <stop offset="70%" stop-color="#2979ff" stop-opacity="0.18"/>
              <stop offset="100%" stop-color="#2979ff" stop-opacity="0.0"/>
            </radialGradient>
          </defs>
          <!-- 60-degree flashlight beam pointing upward (North / 0 deg) -->
          <path d="M 55 55 L 30 11.7 A 50 50 0 0 1 80 11.7 Z" fill="url(#beamGrad)" />
        </svg>
      </div>
      <div class="gmaps-pulse-ring"></div>
      <div class="gmaps-blue-dot"></div>
    </div>
  `,
  iconSize: [110, 110],
  iconAnchor: [55, 55],
});

// Destination Pin Drop Marker (Google Maps Red Pin with Shadow & Bounce)
const destPinIcon = L.divIcon({
  className: 'gmaps-marker-wrapper',
  html: `
    <div class="gmaps-dest-pin">
      <svg viewBox="0 0 24 24" width="38" height="38">
        <path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5c-1.38 0-2.5-1.12-2.5-2.5s1.12-2.5 2.5-2.5 2.5 1.12 2.5 2.5-1.12 2.5-2.5 2.5z" fill="#ea4335"/>
        <circle cx="12" cy="9" r="2.8" fill="#ffffff"/>
      </svg>
    </div>
  `,
  iconSize: [38, 38],
  iconAnchor: [19, 38],
  popupAnchor: [0, -38],
});

// Initialize User Marker immediately on map
const liveMarker = L.marker(DEFAULT_CENTER, {
  icon: userLocationIcon,
  zIndexOffset: 1000,
}).addTo(map);

// Route Polyline (OSRM Driving Route)
const routeLine = L.polyline([], {
  color: '#1a73e8',
  weight: 6,
  opacity: 0.9,
  lineJoin: 'round',
  lineCap: 'round',
}).addTo(map);

// Breadcrumb Trail for Dead Reckoning Tracking
const liveTrail = L.polyline([], {
  color: '#4285f4',
  weight: 4,
  opacity: 0.5,
  dashArray: '4 6',
}).addTo(map);

let destMarker = null;
let selectedDestination = null;
let currentRouteData = null;
let isNavigating = false;

// ------------------------------------------------------------
// 2. Real-time Compass & Heading Cone Rotation
// ------------------------------------------------------------
let latestCompassHeading = null;
let currentRotationDeg = 0;

function updateHeadingUI(deg) {
  if (deg == null || isNaN(deg)) return;
  currentRotationDeg = Math.round((deg % 360 + 360) % 360);

  // 1. Rotate the Google Maps flashlight beam on the user marker
  const cone = document.getElementById('gmaps-cone');
  if (cone) {
    cone.style.transform = `rotate(${currentRotationDeg}deg)`;
  }

  // 2. Rotate the compass needle icon in the peek bar
  const needle = document.getElementById('hud-compass-needle');
  if (needle) {
    needle.style.transform = `rotate(${currentRotationDeg}deg)`;
  }

  // 3. Update the text in peek bar and drawer
  const headingEl = document.getElementById('hud-heading');
  const drawerHeadingEl = document.getElementById('drawer-heading');
  const drawerCardEl = document.getElementById('drawer-cardinal');

  const { cardinal, text } = degToCompassDetails(currentRotationDeg);
  if (headingEl) headingEl.textContent = text;
  if (drawerHeadingEl) drawerHeadingEl.textContent = `${currentRotationDeg}°`;
  if (drawerCardEl) drawerCardEl.textContent = cardinal;
}

function degToCompassDetails(deg) {
  const points = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'];
  const idx = Math.floor((deg / 22.5) + 0.5) % 16;
  const cardinal = points[idx];
  return { cardinal, text: `${cardinal} (${Math.round(deg)}°)` };
}

// Mobile Orientation Handler (iOS Safari & Android Chrome)
function handleDeviceOrientation(e) {
  let heading = null;

  // iOS Safari: webkitCompassHeading is absolute magnetic North (0 = North, 90 = East)
  if (e.webkitCompassHeading !== undefined && e.webkitCompassHeading !== null) {
    heading = e.webkitCompassHeading;
  }
  // Android Chrome: deviceorientationabsolute or absolute event
  else if (e.alpha !== null && e.alpha !== undefined) {
    heading = (360 - e.alpha) % 360;
  }

  if (heading !== null && !isNaN(heading)) {
    const screenAngle = (window.orientation || (screen.orientation && screen.orientation.angle) || 0);
    heading = (heading + screenAngle + 360) % 360;

    latestCompassHeading = heading;
    updateHeadingUI(heading);
  }
}

function attachOrientationListeners() {
  if ('ondeviceorientationabsolute' in window) {
    window.addEventListener('deviceorientationabsolute', handleDeviceOrientation, true);
  }
  window.addEventListener('deviceorientation', handleDeviceOrientation, true);
}

function checkSensorPermissions() {
  if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
    const modal = document.getElementById('perm-modal');
    if (modal) modal.classList.remove('hidden');

    const btnAllow = document.getElementById('btn-allow-sensors');
    if (btnAllow) {
      btnAllow.onclick = async () => {
        try {
          const res = await DeviceOrientationEvent.requestPermission();
          if (res === 'granted') {
            attachOrientationListeners();
          }
        } catch (err) {
          console.warn('Orientation permission notice:', err);
        }
        if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
          try {
            await DeviceMotionEvent.requestPermission();
          } catch (_) {}
        }
        modal.classList.add('hidden');
      };
    }
  } else {
    attachOrientationListeners();
  }
}

window.addEventListener('touchstart', function onFirstTouch() {
  if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
    DeviceOrientationEvent.requestPermission().then(res => {
      if (res === 'granted') attachOrientationListeners();
    }).catch(() => {});
  }
  window.removeEventListener('touchstart', onFirstTouch);
}, { once: true });

checkSensorPermissions();

// ------------------------------------------------------------
// 3. Motion Sensors (IMU Accel + Gyro) & Live S.A.F.A.R. Engine
// ------------------------------------------------------------
let latestAccel = { x: 0, y: 0, z: 9.81 };
let latestGravityEst = { x: 0, y: 0, z: 9.81 };
let latestGyro = { yaw: 0, pitch: 0, roll: 0 };

function handleDeviceMotion(e) {
  const a = e.accelerationIncludingGravity;
  if (!a || a.x == null) return;

  const lin = e.acceleration;
  let linX = 0, linY = 0, linZ = 0;

  if (lin && lin.x != null) {
    linX = lin.x || 0;
    linY = lin.y || 0;
    linZ = lin.z || 0;
    latestAccel = { x: a.x || 0, y: a.y || 0, z: a.z || 0 };
    latestGravityEst = {
      x: (a.x || 0) - linX,
      y: (a.y || 0) - linY,
      z: (a.z || 0) - linZ,
    };
  } else {
    latestAccel = { x: a.x || 0, y: a.y || 0, z: a.z || 0 };
    const alpha = 0.92;
    latestGravityEst = {
      x: alpha * latestGravityEst.x + (1 - alpha) * (a.x || 0),
      y: alpha * latestGravityEst.y + (1 - alpha) * (a.y || 0),
      z: alpha * latestGravityEst.z + (1 - alpha) * (a.z || 0),
    };
    linX = (a.x || 0) - latestGravityEst.x;
    linY = (a.y || 0) - latestGravityEst.y;
    linZ = (a.z || 0) - latestGravityEst.z;
  }

  if (e.rotationRate) {
    const d2r = Math.PI / 180;
    latestGyro = {
      yaw: (e.rotationRate.alpha || 0) * d2r,
      pitch: (e.rotationRate.beta || 0) * d2r,
      roll: (e.rotationRate.gamma || 0) * d2r,
    };
  }

  // Calculate Motion Acceleration (Linear acceleration magnitude without gravity)
  const linMag = Math.sqrt(linX * linX + linY * linY + linZ * linZ);
  const gravMag = Math.sqrt(latestGravityEst.x ** 2 + latestGravityEst.y ** 2 + latestGravityEst.z ** 2);
  const gyroMag = Math.sqrt(latestGyro.yaw ** 2 + latestGyro.pitch ** 2 + latestGyro.roll ** 2);

  // Sensor noise floor is ~0.15 - 0.22 m/s^2. When idle in hand, cleanly display 0.0 m/s^2.
  const displayMotionAccel = linMag < 0.22 ? 0.0 : linMag;

  const accelEl = document.getElementById('sensor-accel-val');
  const gravEl = document.getElementById('sensor-gravity-val');
  const gyroEl = document.getElementById('sensor-gyro-val');
  const turnRateEl = document.getElementById('drawer-turn-rate');

  if (accelEl) accelEl.textContent = `${displayMotionAccel.toFixed(1)} m/s²`;
  if (gravEl) gravEl.textContent = `${gravMag.toFixed(1)} m/s²`;
  if (gyroEl) gyroEl.textContent = `${gyroMag.toFixed(2)} rad/s`;

  if (turnRateEl) {
    const turnRateDeg = (latestGyro.yaw * 180 / Math.PI);
    turnRateEl.textContent = `${Math.abs(turnRateDeg).toFixed(1)}°/s`;
  }
}
window.addEventListener('devicemotion', handleDeviceMotion, true);

// ------------------------------------------------------------
// 4. Client-Side Geodesic Frame & Autonomous S.A.F.A.R. Engine
// ------------------------------------------------------------
class LocalCartesianFrame {
  constructor(latDeg, lonDeg) {
    this.refLat = latDeg * (Math.PI / 180);
    this.refLon = lonDeg * (Math.PI / 180);
    this.cosRefLat = Math.cos(this.refLat);
    this.earthRadius = 6371000.0;
  }
  toXY(latDeg, lonDeg) {
    const lat = latDeg * (Math.PI / 180);
    const lon = lonDeg * (Math.PI / 180);
    const x = (lon - this.refLon) * this.cosRefLat * this.earthRadius;
    const y = (lat - this.refLat) * this.earthRadius;
    return [x, y];
  }
  toLatLon(x, y) {
    const lat = this.refLat + y / this.earthRadius;
    const lon = this.refLon + x / (this.earthRadius * this.cosRefLat);
    return [lat * (180 / Math.PI), lon * (180 / Math.PI)];
  }
}

// S.A.F.A.R. Client-Side Dead Reckoning State
let localRefFrame = null;
let localDrX = 0;
let localDrY = 0;
let localDrHeadingDeg = 0;
let localSpeedKmh = 0.0;
let lastConfirmedGpsSpeedKmh = 0.0;
let lastGoodGps = null;
let lastHardwareGpsTimestamp = 0;
let wasGpsActive = false;
let currentClientMode = 'GNSS'; // 'GNSS' or 'DR'
let clientMotionState = 'STATIONARY';
let clientConfidence = 95;
let turnBias = 0.0;
let warmupTurnRates = [];
let disturbanceCooldown = 0;

// Rolling buffers for IMU features (last 30 samples = 3.0s @ 10Hz)
const imuBuf = {
  linMag: [],
  linZ: [],
  gyroYaw: [],
  maxLen: 30,
};

let latestGps = null;
let hasGpsFix = false;
let userHasPanned = false;
let lastGeoPoint = null;
let currentFusedSpeedKmh = 0.0;
const MAX_ACCEPTABLE_ACCURACY_M = 100;
const GPS_OUTAGE_THRESHOLD_MS = 2500; // 2.5s without hardware fix = tunnel/outage

function haversineM(lat1, lon1, lat2, lon2) {
  const R = 6371000, toRad = Math.PI / 180;
  const dLat = (lat2 - lat1) * toRad, dLon = (lon2 - lon1) * toRad;
  const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * toRad) * Math.cos(lat2 * toRad) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(a));
}

function onGeoSuccess(pos) {
  const acc = pos.coords.accuracy;
  if (acc != null && acc > MAX_ACCEPTABLE_ACCURACY_M) return;

  const lat = pos.coords.latitude;
  const lon = pos.coords.longitude;
  const rawSpeed = pos.coords.speed;
  const heading = pos.coords.heading;
  const now = pos.timestamp || Date.now();

  lastHardwareGpsTimestamp = Date.now();
  lastGoodGps = { lat, lon, time: lastHardwareGpsTimestamp };

  let fixSpeedKmh = null;
  if (rawSpeed !== null && !isNaN(rawSpeed) && rawSpeed >= 0) {
    fixSpeedKmh = rawSpeed * 3.6;
  } else if (lastGeoPoint) {
    const dtSec = (now - lastGeoPoint.time) / 1000.0;
    if (dtSec >= 0.4 && dtSec <= 8.0) {
      const distM = haversineM(lastGeoPoint.lat, lastGeoPoint.lon, lat, lon);
      const jitterDeadband = Math.max(1.8, Math.min(6.0, (acc || 8) * 0.35));
      if (distM > jitterDeadband) {
        fixSpeedKmh = (distM / dtSec) * 3.6;
      } else {
        fixSpeedKmh = 0.0;
      }
    }
  }

  lastGeoPoint = { lat, lon, time: now };

  if (fixSpeedKmh !== null) {
    fixSpeedKmh = Math.min(160.0, Math.max(0.0, fixSpeedKmh));
    if (fixSpeedKmh < 1.0) fixSpeedKmh = 0.0;
    currentFusedSpeedKmh = currentFusedSpeedKmh === 0.0
      ? fixSpeedKmh
      : (currentFusedSpeedKmh * 0.35 + fixSpeedKmh * 0.65);
  }

  latestGps = {
    lat,
    lon,
    speed: rawSpeed,
    speed_kmh: currentFusedSpeedKmh,
    heading,
    accuracy: acc,
  };

  if (!hasGpsFix) {
    hasGpsFix = true;
    localRefFrame = new LocalCartesianFrame(lat, lon);
    localDrX = 0;
    localDrY = 0;
    map.setView([lat, lon], isNavigating ? 17 : 16, { animate: true });
    liveMarker.setLatLng([lat, lon]);
    liveTrail.setLatLngs([[lat, lon]]);
  }

  if (latestCompassHeading == null && heading != null && !isNaN(heading) && (currentFusedSpeedKmh || 0) > 1.0) {
    updateHeadingUI(heading);
  }
}

function onGeoError(err) {
  console.warn('Geolocation notice:', err.code, err.message);
  if (err.code === 2 || err.code === 3) {
    hasGpsFix = false;
  }
}

if (navigator.geolocation) {
  navigator.geolocation.watchPosition(onGeoSuccess, onGeoError, {
    enableHighAccuracy: true,
    maximumAge: 1000,
    timeout: 10000,
  });
}

map.on('dragstart', () => {
  userHasPanned = true;
  document.getElementById('btn-recenter').classList.remove('active');
});

document.getElementById('btn-recenter').onclick = () => {
  userHasPanned = false;
  document.getElementById('btn-recenter').classList.add('active');
  const target = (currentClientMode === 'GNSS' && latestGps) ? [latestGps.lat, latestGps.lon] : liveMarker.getLatLng();
  map.setView(target, isNavigating ? 17 : 16, { animate: true });
};

// ------------------------------------------------------------
// 5. Autonomous Client-Side S.A.F.A.R. Engine & Outage Watchdog
// ------------------------------------------------------------
let liveSessionId = null;
let isDispatching = false;
let isSimulatedOutage = false;

// Optional background cloud session initialization
async function initLiveBackendSession() {
  if (!navigator.onLine) return;
  try {
    const res = await fetch('/api/live/session/start', {
      method: 'POST',
      signal: AbortSignal.timeout(2000),
    });
    if (res.ok) {
      const data = await res.json();
      liveSessionId = data.session_id;
    }
  } catch (_) {}
}
initLiveBackendSession();

const toggleOutage = document.getElementById('toggle-outage');
if (toggleOutage) {
  toggleOutage.onchange = () => {
    isSimulatedOutage = toggleOutage.checked;
  };
}

function updateModeDisplay(mode) {
  const badge = document.getElementById('mode-badge');
  const text = document.getElementById('mode-text');
  const banner = document.getElementById('dr-banner');

  if (mode === 'DR') {
    badge.className = 'mode-badge mode-dr';
    text.textContent = 'S.A.F.A.R. IDR';
    banner.classList.remove('hidden');
    if (toggleOutage) toggleOutage.checked = isSimulatedOutage;
  } else {
    badge.className = 'mode-badge mode-gnss';
    text.textContent = 'GNSS Active';
    banner.classList.add('hidden');
    if (toggleOutage && !isSimulatedOutage) toggleOutage.checked = false;
  }
}

// ------------------------------------------------------------
// Real-time 10Hz Client-Side S.A.F.A.R. Dead Reckoning Loop
// Runs 100% locally on device CPU with ZERO internet dependency!
// ------------------------------------------------------------
const DT = 0.1; // 100ms timestep

function safeTimeoutSignal(ms) {
  if (typeof AbortSignal !== 'undefined' && typeof AbortSignal.timeout === 'function') {
    return AbortSignal.timeout(ms);
  }
  const controller = new AbortController();
  setTimeout(() => controller.abort(), ms);
  return controller.signal;
}

setInterval(() => {
  // 1. Evaluate GPS Outage Status (Watchdog + Manual Outage Toggle)
  const gpsSilenceMs = Date.now() - lastHardwareGpsTimestamp;
  const isOutage = isSimulatedOutage || !hasGpsFix || (gpsSilenceMs > GPS_OUTAGE_THRESHOLD_MS);
  currentClientMode = isOutage ? 'DR' : 'GNSS';

  // 2. Extract IMU Motion and Energy
  const linX = (latestAccel.x || 0) - (latestGravityEst.x || 0);
  const linY = (latestAccel.y || 0) - (latestGravityEst.y || 0);
  const linZ = (latestAccel.z || 0) - (latestGravityEst.z || 0);
  const linMag = Math.sqrt(linX * linX + linY * linY + linZ * linZ);

  imuBuf.linMag.push(linMag);
  imuBuf.linZ.push(linZ);
  imuBuf.gyroYaw.push(latestGyro.yaw || 0);
  if (imuBuf.linMag.length > imuBuf.maxLen) {
    imuBuf.linMag.shift();
    imuBuf.linZ.shift();
    imuBuf.gyroYaw.shift();
  }

  // Calculate rolling statistics
  const n = imuBuf.linMag.length;
  let linMean = 0;
  for (let i = 0; i < n; i++) linMean += imuBuf.linMag[i];
  linMean = n > 0 ? linMean / n : 0;

  let linVar = 0;
  for (let i = 0; i < n; i++) linVar += (imuBuf.linMag[i] - linMean) ** 2;
  const linStd = n > 1 ? Math.sqrt(linVar / (n - 1)) : 0;

  const gyroMag = Math.sqrt((latestGyro.yaw || 0) ** 2 + (latestGyro.pitch || 0) ** 2 + (latestGyro.roll || 0) ** 2);

  // Decompose Gyro onto Earth-Vertical (Gravity) Vector
  const gNorm = Math.sqrt((latestGravityEst.x || 0) ** 2 + (latestGravityEst.y || 0) ** 2 + (latestGravityEst.z || 0) ** 2) || 9.81;
  const gUnit = [(latestGravityEst.x || 0) / gNorm, (latestGravityEst.y || 0) / gNorm, (latestGravityEst.z || 0) / gNorm];
  const omegaTurn = (latestGyro.pitch || 0) * gUnit[0] + (latestGyro.roll || 0) * gUnit[1] + (latestGyro.yaw || 0) * gUnit[2];
  const tiltRateMag = Math.sqrt(Math.max(0, gyroMag ** 2 - omegaTurn ** 2));

  // 3. Motion Classification (ZUPT / Disturbance / Driving)
  const isImuQuiet = (linStd < 0.22 && linMean < 0.35 && gyroMag < 0.18);

  if (isImuQuiet) {
    if (localSpeedKmh < 1.5) {
      clientMotionState = 'STATIONARY';
      clientConfidence = 98;
    } else {
      clientMotionState = 'COASTING_TO_STOP';
      clientConfidence = 92;
    }
  } else if (localSpeedKmh < 3.5 && (gyroMag > 1.2 || tiltRateMag > 1.0)) {
    clientMotionState = 'HAND_DISTURBANCE';
    clientConfidence = 30;
    disturbanceCooldown = 10;
  } else {
    if (disturbanceCooldown > 0) disturbanceCooldown--;
    clientMotionState = 'VEHICLE_DRIVING';
    clientConfidence = Math.min(96, Math.max(75, Math.round(96 - linStd * 3)));
  }

  // 4. Navigation & Speed Fusion Execution
  if (!isOutage && latestGps) {
    // ---------------- GNSS SATELLITE ACTIVE ----------------
    wasGpsActive = true;
    localSpeedKmh = currentFusedSpeedKmh;
    lastConfirmedGpsSpeedKmh = currentFusedSpeedKmh;

    if (localRefFrame) {
      localRefFrame = new LocalCartesianFrame(latestGps.lat, latestGps.lon);
      localDrX = 0;
      localDrY = 0;
    }

    // Calibrate turn rate bias while driving with GPS
    if (currentFusedSpeedKmh > 5.0 && Math.abs(omegaTurn) < 0.3) {
      warmupTurnRates.push(omegaTurn);
      if (warmupTurnRates.length > 30) warmupTurnRates.shift();
      if (warmupTurnRates.length >= 10) {
        const sorted = [...warmupTurnRates].sort((a, b) => a - b);
        turnBias = sorted[Math.floor(sorted.length / 2)];
      }
    }

    liveMarker.setLatLng([latestGps.lat, latestGps.lon]);
    liveTrail.addLatLng([latestGps.lat, latestGps.lon]);
    if (!userHasPanned) {
      map.panTo([latestGps.lat, latestGps.lon], { animate: true, duration: 0.1 });
    }
  } else {
    // ---------------- S.A.F.A.R. DEAD RECKONING (TUNNEL / NO GPS / NO INTERNET) ----------------
    if (wasGpsActive || !localRefFrame) {
      wasGpsActive = false;
      const refLat = lastGoodGps ? lastGoodGps.lat : liveMarker.getLatLng().lat;
      const refLon = lastGoodGps ? lastGoodGps.lon : liveMarker.getLatLng().lng;
      localRefFrame = new LocalCartesianFrame(refLat, refLon);
      localDrX = 0;
      localDrY = 0;
      localSpeedKmh = Math.max(0.0, lastConfirmedGpsSpeedKmh);
      if (latestCompassHeading != null) {
        localDrHeadingDeg = latestCompassHeading;
      }
    }

    // Kinematic Speed Estimation
    if (clientMotionState === 'STATIONARY') {
      localSpeedKmh = 0.0;
    } else if (isImuQuiet) {
      // Quiet phone = vehicle decelerating / braking to a halt
      const brakeDelta = 3.0 * 3.6 * DT; // ~1.08 km/h per 100ms
      localSpeedKmh = Math.max(0.0, localSpeedKmh - brakeDelta);
      if (localSpeedKmh < 1.0) {
        localSpeedKmh = 0.0;
        clientMotionState = 'STATIONARY';
      }
    } else if (clientMotionState === 'HAND_DISTURBANCE') {
      localSpeedKmh *= Math.exp(-DT / 0.8);
      if (localSpeedKmh < 0.5) localSpeedKmh = 0.0;
    } else {
      // VEHICLE_DRIVING: Model speed using vibration energy & acceleration
      const vibSpeed = 24.0 * Math.sqrt(Math.max(0, linStd - 0.18));
      let targetSpeed = Math.max(localSpeedKmh * 0.994, vibSpeed);
      targetSpeed = Math.min(130.0, Math.max(0.0, targetSpeed));
      const maxDelta = 2.5 * 3.6 * DT; // Physical acceleration bound: 2.5 m/s^2 (~0.9 km/h per 100ms)
      const delta = Math.max(-maxDelta * 1.5, Math.min(maxDelta, targetSpeed - localSpeedKmh));
      localSpeedKmh = Math.max(0.0, localSpeedKmh + delta);
    }

    // Dynamic Heading Update
    if (latestCompassHeading != null) {
      localDrHeadingDeg = latestCompassHeading;
    } else {
      const effTurnRate = (omegaTurn - turnBias) * (180 / Math.PI);
      if (Math.abs(effTurnRate) > 0.8) {
        localDrHeadingDeg = (localDrHeadingDeg + effTurnRate * DT + 360) % 360;
      }
    }
    updateHeadingUI(localDrHeadingDeg);

    // Non-Holonomic Constraint (NHC) Advance in Local Frame
    if (localRefFrame) {
      const speedMs = localSpeedKmh / 3.6;
      const headingRad = (localDrHeadingDeg * Math.PI) / 180;
      localDrX += speedMs * Math.sin(headingRad) * DT; // East (x)
      localDrY += speedMs * Math.cos(headingRad) * DT; // North (y)

      const [newLat, newLon] = localRefFrame.toLatLon(localDrX, localDrY);
      liveMarker.setLatLng([newLat, newLon]);
      liveTrail.addLatLng([newLat, newLon]);

      if (!userHasPanned) {
        map.panTo([newLat, newLon], { animate: true, duration: 0.1 });
      }
    }
  }

  // 5. Update UI Components Instantly (Zero Network Wait)
  const spdText = localSpeedKmh.toFixed(1);
  const speedEl = document.getElementById('hud-speed');
  const drawerSpeedEl = document.getElementById('drawer-speed');
  const confEl = document.getElementById('hud-conf');
  const confBar = document.getElementById('hud-conf-bar');
  const motionEl = document.getElementById('hud-motion');

  if (speedEl) speedEl.textContent = spdText;
  if (drawerSpeedEl) drawerSpeedEl.textContent = spdText;

  if (confEl) confEl.textContent = `${clientConfidence}%`;
  if (confBar) {
    confBar.style.width = `${clientConfidence}%`;
    confBar.style.background = clientConfidence >= 75 ? 'var(--google-green)' : clientConfidence >= 50 ? 'var(--google-amber)' : 'var(--google-red)';
  }

  const motionMap = {
    'STATIONARY': 'Stationary (ZUPT)',
    'COASTING_TO_STOP': 'Braking / Coasting',
    'HAND_DISTURBANCE': 'Disturbance Filtered',
    'VEHICLE_DRIVING': 'Vehicle Kinematics',
  };
  if (motionEl) {
    motionEl.textContent = motionMap[clientMotionState] || clientMotionState;
  }

  updateModeDisplay(currentClientMode);

  // 6. Asynchronous Non-Blocking Cloud Telemetry (Zero network requests during outage / dead zones)
  if (navigator.onLine && !isOutage && liveSessionId && !isDispatching) {
    isDispatching = true;
    const body = {
      accel: latestAccel,
      gravity: latestGravityEst,
      gyro: latestGyro,
      gps: latestGps,
      simulate_outage: false,
      dt: DT,
    };

    fetch(`/api/live/session/${liveSessionId}/sample`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
      signal: safeTimeoutSignal(350),
    }).catch(() => {}).finally(() => {
      isDispatching = false;
    });
  }
}, 100);

// ------------------------------------------------------------
// 6. Interactive Half-Screen Draggable Bottom Sheet
// ------------------------------------------------------------
const bottomSheet = document.getElementById('bottom-sheet');
const dragZone = document.getElementById('sheet-drag-zone');
const peekBar = document.getElementById('sheet-peek-bar');
const expandBtn = document.getElementById('peek-expand-btn');

const PEEK_HEIGHT = 78;
let maxHalfHeight = Math.round(window.innerHeight * 0.5);

window.addEventListener('resize', () => {
  maxHalfHeight = Math.round(window.innerHeight * 0.5);
  if (bottomSheet.classList.contains('expanded')) {
    bottomSheet.style.height = `${maxHalfHeight}px`;
  }
});

let isDragging = false;
let startY = 0;
let startHeight = PEEK_HEIGHT;

function setSheetHeight(height, animated = false) {
  if (animated) {
    bottomSheet.style.transition = 'height 0.28s cubic-bezier(0.2, 0.9, 0.3, 1)';
  } else {
    bottomSheet.style.transition = 'none';
  }
  bottomSheet.style.height = `${height}px`;

  const isExpanded = height > (PEEK_HEIGHT + (maxHalfHeight - PEEK_HEIGHT) * 0.4);
  bottomSheet.classList.toggle('expanded', isExpanded);

  // Adjust map controls position dynamically
  const mapControls = document.querySelector('.map-controls');
  if (mapControls) {
    mapControls.style.bottom = `${height + 14}px`;
  }
}

function expandSheet() {
  setSheetHeight(maxHalfHeight, true);
}

function collapseSheet() {
  setSheetHeight(PEEK_HEIGHT, true);
}

function toggleSheet() {
  if (bottomSheet.classList.contains('expanded')) {
    collapseSheet();
  } else {
    expandSheet();
  }
}

// Touch Drag Listeners
function onDragStart(clientY) {
  isDragging = true;
  startY = clientY;
  startHeight = bottomSheet.offsetHeight;
  bottomSheet.style.transition = 'none';
}

function onDragMove(clientY) {
  if (!isDragging) return;
  const deltaY = startY - clientY; // Dragging up increases height
  const newHeight = Math.max(PEEK_HEIGHT, Math.min(maxHalfHeight, startHeight + deltaY));
  setSheetHeight(newHeight, false);
}

function onDragEnd() {
  if (!isDragging) return;
  isDragging = false;
  const currentH = bottomSheet.offsetHeight;
  const midpoint = PEEK_HEIGHT + (maxHalfHeight - PEEK_HEIGHT) * 0.45;

  if (currentH > midpoint) {
    expandSheet();
  } else {
    collapseSheet();
  }
}

dragZone.addEventListener('touchstart', (e) => {
  onDragStart(e.touches[0].clientY);
}, { passive: true });

window.addEventListener('touchmove', (e) => {
  if (isDragging) {
    onDragMove(e.touches[0].clientY);
  }
}, { passive: true });

window.addEventListener('touchend', () => {
  if (isDragging) onDragEnd();
});

// Mouse Drag Listeners (for Desktop / Testing)
dragZone.addEventListener('mousedown', (e) => {
  onDragStart(e.clientY);
  const onMouseMove = (ev) => onDragMove(ev.clientY);
  const onMouseUp = () => {
    onDragEnd();
    window.removeEventListener('mousemove', onMouseMove);
    window.removeEventListener('mouseup', onMouseUp);
  };
  window.addEventListener('mousemove', onMouseMove);
  window.addEventListener('mouseup', onMouseUp);
});

// Click / Tap Toggle
dragZone.addEventListener('click', toggleSheet);
peekBar.addEventListener('click', (e) => {
  // If not clicking a button inside peek bar
  if (!e.target.closest('#peek-expand-btn')) {
    toggleSheet();
  }
});
expandBtn.addEventListener('click', (e) => {
  e.stopPropagation();
  toggleSheet();
});

// ------------------------------------------------------------
// 7. Destination Search & Route Preview / Start Flow
// ------------------------------------------------------------
const searchInput = document.getElementById('destination-search');
const suggestionsBox = document.getElementById('suggestions-box');
const btnClearSearch = document.getElementById('btn-clear-search');
const routeCard = document.getElementById('route-card');
const btnCancelRoute = document.getElementById('btn-cancel-route');
const btnDropPin = document.getElementById('btn-drop-pin');
const btnStartNav = document.getElementById('btn-start-nav');
const activeTripBar = document.getElementById('active-trip-bar');
const btnStopNav = document.getElementById('btn-stop-nav');

let isPinModeActive = false;
let searchDebounceTimer = null;

btnDropPin.onclick = () => {
  isPinModeActive = !isPinModeActive;
  btnDropPin.classList.toggle('active', isPinModeActive);
  map.getContainer().style.cursor = isPinModeActive ? 'crosshair' : '';
};

// Map Click / Tap: Drop Pin & Preview Route
map.on('click', async (e) => {
  const lat = e.latlng.lat;
  const lon = e.latlng.lng;
  const label = `Pinned (${lat.toFixed(4)}, ${lon.toFixed(4)})`;

  setDestination(lat, lon, label);

  try {
    const revUrl = `https://nominatim.openstreetmap.org/reverse?format=json&lat=${lat}&lon=${lon}`;
    const res = await fetch(revUrl);
    if (res.ok) {
      const data = await res.json();
      if (data && data.display_name) {
        const placeName = data.display_name.split(',')[0].trim();
        document.getElementById('route-dest-name').textContent = placeName;
        searchInput.value = placeName;
      }
    }
  } catch (_) {}
});

async function setDestination(lat, lon, label) {
  selectedDestination = { lat, lon, label };

  if (!destMarker) {
    destMarker = L.marker([lat, lon], { icon: destPinIcon, zIndexOffset: 900 }).addTo(map);
  } else {
    destMarker.setLatLng([lat, lon]).addTo(map);
  }

  searchInput.value = label;
  btnClearSearch.classList.remove('hidden');
  suggestionsBox.classList.add('hidden');

  await calculateRoute();
}

// Compute driving route using OSRM
async function calculateRoute() {
  if (!selectedDestination) return;

  const origin = latestGps ? latestGps : { lat: liveMarker.getLatLng().lat, lon: liveMarker.getLatLng().lng };
  const originLat = origin.lat;
  const originLon = origin.lon;
  const destLat = selectedDestination.lat;
  const destLon = selectedDestination.lon;

  const url = `https://router.project-osrm.org/route/v1/driving/${originLon},${originLat};${destLon},${destLat}?overview=full&geometries=geojson`;

  try {
    const res = await fetch(url);
    if (!res.ok) throw new Error('Route service unavailable');
    const data = await res.json();

    if (!data.routes || !data.routes.length) {
      console.warn('No routes found between points.');
      return;
    }

    const route = data.routes[0];
    currentRouteData = route;
    const coords = route.geometry.coordinates.map(([lon, lat]) => [lat, lon]);
    routeLine.setLatLngs(coords);

    map.fitBounds(routeLine.getBounds(), { padding: [60, 60] });

    const distanceKm = (route.distance / 1000).toFixed(1);
    const durationMin = Math.max(1, Math.round(route.duration / 60));

    document.getElementById('route-dest-name').textContent = selectedDestination.label;
    document.getElementById('route-eta').textContent = `${durationMin} mins`;
    document.getElementById('route-dist').textContent = `${distanceKm} km`;
    routeCard.classList.remove('hidden');
  } catch (err) {
    console.warn('Route calculation notice:', err.message);
  }
}

// "Start Navigation" Button Handler
btnStartNav.onclick = () => {
  if (!selectedDestination || !currentRouteData) return;

  isNavigating = true;
  routeCard.classList.add('hidden');

  // Populate active trip bar inside bottom sheet
  const distanceKm = (currentRouteData.distance / 1000).toFixed(1);
  const durationMin = Math.max(1, Math.round(currentRouteData.duration / 60));
  document.getElementById('trip-dest-name').textContent = selectedDestination.label;
  document.getElementById('trip-meta-info').textContent = `${durationMin} mins • ${distanceKm} km remaining`;
  activeTripBar.classList.remove('hidden');

  // Zoom to navigation follow mode
  userHasPanned = false;
  document.getElementById('btn-recenter').classList.add('active');
  const target = latestGps ? [latestGps.lat, latestGps.lon] : liveMarker.getLatLng();
  map.setView(target, 17, { animate: true });

  // Open the bottom sheet to half screen so user sees all metrics immediately
  expandSheet();
};

// "End Trip" / Cancel Route Handlers
function clearCurrentRoute() {
  isNavigating = false;
  selectedDestination = null;
  currentRouteData = null;

  if (destMarker) destMarker.remove();
  routeLine.setLatLngs([]);
  routeCard.classList.add('hidden');
  activeTripBar.classList.add('hidden');

  searchInput.value = '';
  btnClearSearch.classList.add('hidden');
  suggestionsBox.classList.add('hidden');

  collapseSheet();
}

btnCancelRoute.onclick = clearCurrentRoute;
btnClearSearch.onclick = clearCurrentRoute;
btnStopNav.onclick = clearCurrentRoute;

// Search Autocomplete (Professional SVG Map Pin, 250ms Debounce)
searchInput.addEventListener('input', (e) => {
  const query = e.target.value.trim();
  btnClearSearch.classList.toggle('hidden', query.length === 0);

  if (query.length < 2) {
    suggestionsBox.innerHTML = '';
    suggestionsBox.classList.add('hidden');
    return;
  }

  clearTimeout(searchDebounceTimer);
  searchDebounceTimer = setTimeout(async () => {
    try {
      let url = `https://nominatim.openstreetmap.org/search?format=json&limit=5&q=${encodeURIComponent(query)}`;
      if (latestGps) {
        url += `&viewbox=${latestGps.lon - 0.4},${latestGps.lat + 0.4},${latestGps.lon + 0.4},${latestGps.lat - 0.4}`;
      }

      const res = await fetch(url, { headers: { 'Accept-Language': 'en' } });
      if (!res.ok) return;
      const results = await res.json();

      if (!results || results.length === 0) {
        suggestionsBox.innerHTML = `
          <div class="suggestion-item">
            <div class="sugg-text">
              <span class="sugg-sub">No places found for "${query}"</span>
            </div>
          </div>`;
        suggestionsBox.classList.remove('hidden');
        return;
      }

      suggestionsBox.innerHTML = results.map((item, idx) => {
        const parts = item.display_name.split(',');
        const main = parts[0].trim();
        const sub = parts.slice(1, 4).join(',').trim();
        return `
          <div class="suggestion-item" data-idx="${idx}">
            <div class="sugg-pin-icon">
              <svg viewBox="0 0 24 24" width="16" height="16" fill="#5f6368">
                <path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5a2.5 2.5 0 0 1 0-5 2.5 2.5 0 0 1 0 5z"/>
              </svg>
            </div>
            <div class="sugg-text">
              <span class="sugg-main">${main}</span>
              <span class="sugg-sub">${sub || item.display_name}</span>
            </div>
          </div>
        `;
      }).join('');

      suggestionsBox.classList.remove('hidden');

      suggestionsBox.querySelectorAll('.suggestion-item').forEach(el => {
        el.onclick = () => {
          const idx = parseInt(el.dataset.idx, 10);
          const r = results[idx];
          if (!r) return;
          const mainName = r.display_name.split(',')[0].trim();
          setDestination(parseFloat(r.lat), parseFloat(r.lon), mainName);
        };
      });
    } catch (err) {
      console.warn('Search autocomplete error:', err.message);
    }
  }, 250);
});

searchInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') {
    const firstItem = suggestionsBox.querySelector('.suggestion-item');
    if (firstItem) firstItem.click();
  }
});

// Quick Category Chips (Petrol Pump, Hospital, Station, Metro, Airport)
document.querySelectorAll('.chip-item').forEach(chip => {
  chip.onclick = async () => {
    const category = chip.dataset.q;
    searchInput.value = category;
    btnClearSearch.classList.remove('hidden');

    try {
      const origin = latestGps ? latestGps : { lat: liveMarker.getLatLng().lat, lon: liveMarker.getLatLng().lng };
      const url = `https://nominatim.openstreetmap.org/search?format=json&limit=5&q=${encodeURIComponent(category)}&viewbox=${origin.lon - 0.2},${origin.lat + 0.2},${origin.lon + 0.2},${origin.lat - 0.2}`;

      const res = await fetch(url, { headers: { 'Accept-Language': 'en' } });
      if (!res.ok) return;
      const results = await res.json();

      if (results && results.length > 0) {
        const topResult = results[0];
        const title = topResult.display_name.split(',')[0].trim();
        setDestination(parseFloat(topResult.lat), parseFloat(topResult.lon), `${category}: ${title}`);
      }
    } catch (err) {
      console.warn('Quick chip search error:', err.message);
    }
  };
});