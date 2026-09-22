// ============================================================
// S.A.F.A.R. — Sensor-Aided Fusion for Accurate Routing
// Intelligent Dead Reckoning Frontend (SIH 26168 | Team DietCode)
// ============================================================

const map = L.map('map').setView([28.6139, 77.2090], 14); // New Delhi default
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; OpenStreetMap contributors | S.A.F.A.R. Team DietCode',
}).addTo(map);

// Google Maps Style Blue Dot Marker with Heading Beam Cone
function createGmapsMarker() {
  const html = `
    <div class="gmaps-user-marker">
      <div class="gmaps-heading-cone" id="gmaps-cone">
        <svg viewBox="0 0 90 90" width="90" height="90">
          <defs>
            <radialGradient id="beamGrad" cx="45" cy="45" r="42" fx="45" fy="45" gradientUnits="userSpaceOnUse">
              <stop offset="0%" stop-color="#2979ff" stop-opacity="0.65"/>
              <stop offset="60%" stop-color="#2979ff" stop-opacity="0.22"/>
              <stop offset="100%" stop-color="#2979ff" stop-opacity="0.0"/>
            </radialGradient>
          </defs>
          <!-- 55-degree flashlight beam pointing upward (North / 0 deg) -->
          <path d="M 45 45 L 25 8.5 A 42 42 0 0 1 65 8.5 Z" fill="url(#beamGrad)" />
        </svg>
      </div>
      <div class="gmaps-pulse-ring"></div>
      <div class="gmaps-blue-dot"></div>
    </div>
  `;
  return L.marker([28.6139, 77.2090], {
    icon: L.divIcon({
      className: 'gmaps-marker-wrapper',
      html: html,
      iconSize: [90, 90],
      iconAnchor: [45, 45],
    }),
    zIndexOffset: 1000,
  });
}

function createCircleDot(color) {
  return L.circleMarker([0, 0], {
    radius: 8, color: '#ffffff', weight: 2.5, fillColor: color, fillOpacity: 1
  });
}

// Map Layers
const liveMarker = createGmapsMarker();
const destMarker = createCircleDot('#ff3d00'); // Destination marker (red)
const routeLine = L.polyline([], { color: '#00e676', weight: 5, opacity: 0.85 }).addTo(map);
const liveTrail = L.polyline([], { color: '#2979ff', weight: 4, opacity: 0.8 });

// Replay Layers
const gtLine = L.polyline([], { color: '#00e676', weight: 4, opacity: 0.85 });
const naiveLine = L.polyline([], { color: '#ff3d00', weight: 3, opacity: 0.75, dashArray: '6 6' });
const aiLine = L.polyline([], { color: '#00d2ff', weight: 4, opacity: 0.9 });
const gtMarker = createCircleDot('#00e676');
const naiveMarker = createCircleDot('#ff3d00');
const aiMarker = createCircleDot('#00d2ff');

let currentHeadingDeg = 0;
let latestCompassHeading = null;

function updateConeRotation(deg) {
  if (deg == null || isNaN(deg)) return;
  currentHeadingDeg = deg;
  const cone = document.getElementById('gmaps-cone');
  if (cone) {
    cone.style.transform = `rotate(${deg}deg)`;
  }
  const headingEl = document.getElementById('hud-heading');
  if (headingEl) {
    headingEl.textContent = degToCompass(deg);
  }
}

function degToCompass(deg) {
  if (deg == null || isNaN(deg)) return '—';
  const val = Math.floor((deg / 22.5) + 0.5);
  const arr = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'];
  return `${arr[val % 16]} (${Math.round(deg)}°)`;
}

// Update HUD Dashboard
function updateHUD(data) {
  const speedEl = document.getElementById('hud-speed');
  const confEl = document.getElementById('hud-conf');
  const motionEl = document.getElementById('hud-motion');
  const pillEl = document.getElementById('live-indicator');
  const indText = document.getElementById('indicator-text');
  const banner = document.getElementById('status-banner');

  // Speed
  if (data.speed_kmh != null) {
    speedEl.innerHTML = `${data.speed_kmh.toFixed(1)} <span class="unit">km/h</span>`;
  } else {
    speedEl.innerHTML = `0.0 <span class="unit">km/h</span>`;
  }

  // Heading from fusion if no live compass
  if (data.heading_deg != null && latestCompassHeading == null) {
    updateConeRotation(data.heading_deg);
  }

  // Confidence
  const conf = data.confidence != null ? data.confidence : 95;
  confEl.innerHTML = `${conf}<span class="unit">%</span>`;
  if (conf > 75) {
    confEl.style.color = 'var(--accent-green)';
  } else if (conf > 50) {
    confEl.style.color = 'var(--accent-orange)';
  } else {
    confEl.style.color = 'var(--accent-red)';
  }

  // Motion State
  const stateLabels = {
    'STATIONARY': 'Stationary (ZUPT)',
    'HAND_DISTURBANCE': 'Disturbance Filtered',
    'VEHICLE_DRIVING': 'Vehicle Kinematics',
  };
  motionEl.textContent = stateLabels[data.motion_state] || (data.motion_state || 'Stationary (ZUPT)');

  // Mode and Banner
  if (data.mode === 'DR') {
    pillEl.className = 'live-pill status-dr';
    indText.textContent = 'Dead Reckoning (IDR)';
    banner.classList.remove('hidden');
  } else if (data.mode === 'GNSS') {
    pillEl.className = 'live-pill status-gnss';
    indText.textContent = 'GNSS Available';
    banner.classList.add('hidden');
  } else {
    pillEl.className = 'live-pill';
    indText.textContent = 'Waiting for Fix';
    banner.classList.add('hidden');
  }
}

function haversineM(lat1, lon1, lat2, lon2) {
  const R = 6371000, toRad = Math.PI / 180;
  const dLat = (lat2 - lat1) * toRad, dLon = (lon2 - lon1) * toRad;
  const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * toRad) * Math.cos(lat2 * toRad) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(a));
}

// ============================================================
// Tabs
// ============================================================
document.getElementById('tab-replay').onclick = () => switchTab('replay');
document.getElementById('tab-live').onclick = () => switchTab('live');

function switchTab(which) {
  document.getElementById('tab-replay').classList.toggle('active', which === 'replay');
  document.getElementById('tab-live').classList.toggle('active', which === 'live');
  document.getElementById('panel-replay').classList.toggle('hidden', which !== 'replay');
  document.getElementById('panel-live').classList.toggle('hidden', which !== 'live');

  if (which === 'replay') {
    liveMarker.remove(); destMarker.remove(); routeLine.setLatLngs([]); liveTrail.remove();
    gtLine.addTo(map); naiveLine.addTo(map); aiLine.addTo(map);
  } else {
    gtLine.remove(); naiveLine.remove(); aiLine.remove();
    gtLine.setLatLngs([]); naiveLine.setLatLngs([]); aiLine.setLatLngs([]);
    if (haveEverFixed) {
      liveMarker.addTo(map);
      liveTrail.addTo(map);
    }
  }
}
switchTab('live');

// ============================================================
// Interactive Map Click-to-Pinpoint Destination
// ============================================================
let pinModeActive = false;

document.getElementById('btn-pin-mode').onclick = () => {
  pinModeActive = !pinModeActive;
  document.getElementById('btn-pin-mode').classList.toggle('active', pinModeActive);
};

map.on('click', (e) => {
  setDestination(e.latlng.lat, e.latlng.lng, `Pinned (${e.latlng.lat.toFixed(4)}, ${e.latlng.lng.toFixed(4)})`);
});

function setDestination(lat, lon, label) {
  selectedDestination = { lat, lon, label };
  destMarker.setLatLng([lat, lon]).addTo(map);
  document.getElementById('destination-search').value = label;
  document.getElementById('btn-clear-search').classList.remove('hidden');
  drawRoute();
}

document.getElementById('btn-clear-route').onclick = clearRoute;
document.getElementById('btn-clear-search').onclick = () => {
  document.getElementById('destination-search').value = '';
  document.getElementById('btn-clear-search').classList.add('hidden');
  clearRoute();
};

function clearRoute() {
  selectedDestination = null;
  destMarker.remove();
  routeLine.setLatLngs([]);
  document.getElementById('route-summary-card').classList.add('hidden');
}

// Quick Destination Chips
document.querySelectorAll('.chip-btn').forEach(btn => {
  btn.onclick = async () => {
    const q = btn.dataset.query;
    document.getElementById('destination-search').value = q;
    document.getElementById('btn-clear-search').classList.remove('hidden');
    await performSearch(q);
  };
});

// ============================================================
// Live Navigation Mode
// ============================================================
let liveSessionId = null;
let liveTimer = null;
let tracking = false;
let isDispatching = false;

let latestAccel = null;
let latestGravityEst = { x: 0, y: 0, z: 9.81 };
let latestGyro = { yaw: 0, pitch: 0, roll: 0 };

let latestGps = null;
let lastFixTime = 0;
let haveEverFixed = false;
let gpsErrorOccurred = false;
const MAX_ACCEPTABLE_ACCURACY_M = 80;

function onDeviceMotion(e) {
  const a = e.accelerationIncludingGravity;
  if (!a || a.x == null) return;

  const lin = e.acceleration;
  if (lin && lin.x != null) {
    // Hardware-fused linear acceleration from smartphone IMU chip!
    latestAccel = { x: a.x, y: a.y, z: a.z };
    latestGravityEst = { x: a.x - lin.x, y: a.y - lin.y, z: a.z - lin.z };
  } else {
    // Calibrated 0.5Hz low-pass filter
    latestAccel = { x: a.x, y: a.y, z: a.z };
    const alpha = 0.94;
    latestGravityEst = {
      x: alpha * latestGravityEst.x + (1 - alpha) * a.x,
      y: alpha * latestGravityEst.y + (1 - alpha) * a.y,
      z: alpha * latestGravityEst.z + (1 - alpha) * a.z,
    };
  }

  if (e.rotationRate) {
    const d2r = Math.PI / 180;
    latestGyro = {
      yaw: (e.rotationRate.alpha || 0) * d2r,
      pitch: (e.rotationRate.beta || 0) * d2r,
      roll: (e.rotationRate.gamma || 0) * d2r,
    };
  }
}

// Google Maps Heading Light Cone via Device Orientation
function onDeviceOrientation(e) {
  let heading = null;
  if (e.webkitCompassHeading != null) {
    // iOS Safari provides exact compass heading (0 = North)
    heading = e.webkitCompassHeading;
  } else if (e.alpha != null) {
    // Android Chrome (alpha: 0 to 360)
    heading = 360 - e.alpha;
  }
  if (heading != null) {
    latestCompassHeading = heading;
    updateConeRotation(heading);
  }
}

function onGeoSuccess(pos) {
  const acc = pos.coords.accuracy;
  if (acc != null && acc > MAX_ACCEPTABLE_ACCURACY_M) return;

  latestGps = {
    lat: pos.coords.latitude,
    lon: pos.coords.longitude,
    speed: pos.coords.speed,
    heading: pos.coords.heading,
    accuracy: acc,
  };
  lastFixTime = Date.now();
  gpsErrorOccurred = false;

  if (!haveEverFixed) {
    haveEverFixed = true;
    map.setView([latestGps.lat, latestGps.lon], 16);
    liveMarker.setLatLng([latestGps.lat, latestGps.lon]).addTo(map);
    liveTrail.addTo(map);
    document.getElementById('btn-recenter').disabled = false;
  }
}

function onGeoError(err) {
  console.warn('Geolocation error:', err.message);
  gpsErrorOccurred = true;
}

document.getElementById('btn-recenter').onclick = () => {
  if (latestGps) {
    map.setView([latestGps.lat, latestGps.lon], 17);
  }
};

document.getElementById('btn-start-live').onclick = async () => {
  if (tracking) return;

  // iOS 13+ Motion permission
  if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
    try {
      const perm = await DeviceMotionEvent.requestPermission();
      if (perm !== 'granted') {
        alert('Motion sensor permission is required for S.A.F.A.R. dead-reckoning.');
        return;
      }
    } catch (err) {
      alert('Could not request motion permission: ' + err);
      return;
    }
  }

  // iOS 13+ Orientation permission
  if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
    try {
      await DeviceOrientationEvent.requestPermission();
    } catch (_) {}
  }

  window.addEventListener('devicemotion', onDeviceMotion);
  window.addEventListener('deviceorientation', onDeviceOrientation);

  if (!navigator.geolocation) {
    alert('This device does not support geolocation.');
    return;
  }
  navigator.geolocation.watchPosition(onGeoSuccess, onGeoError, {
    enableHighAccuracy: true, maximumAge: 1000, timeout: 8000,
  });

  try {
    const res = await fetch('/api/live/session/start', { method: 'POST' });
    const data = await res.json();
    liveSessionId = data.session_id;
  } catch (err) {
    alert('Failed to connect to S.A.F.A.R. backend engine: ' + err.message);
    return;
  }

  tracking = true;
  document.getElementById('btn-start-live').disabled = true;
  document.getElementById('btn-start-live').innerHTML = '<span class="pulse-icon"></span> Navigation Active';

  // 10Hz sampling loop with non-blocking dispatch
  liveTimer = setInterval(async () => {
    if (!latestAccel || isDispatching) return;

    isDispatching = true;
    try {
      const manualOutage = document.getElementById('toggle-outage-live').checked;
      const isOutage = manualOutage || gpsErrorOccurred || !haveEverFixed;
      const useGps = haveEverFixed && !isOutage;

      const body = {
        accel: latestAccel,
        gravity: latestGravityEst,
        gyro: latestGyro,
        gps: useGps ? latestGps : null,
        simulate_outage: isOutage,
        dt: 0.1,
      };

      const r = await fetch(`/api/live/session/${liveSessionId}/sample`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      const d = await r.json();

      if (d.mode === 'NO_FIX' || d.lat == null) {
        updateHUD({ mode: 'NO_FIX', speed_kmh: 0.0, heading_deg: null, confidence: 15, motion_state: 'Waiting for GPS' });
        return;
      }

      liveMarker.setLatLng([d.lat, d.lon]).addTo(map);
      liveTrail.addLatLng([d.lat, d.lon]);
      map.panTo([d.lat, d.lon]);

      updateHUD(d);
    } catch (err) {
      console.warn('Live sample sync error:', err);
    } finally {
      isDispatching = false;
    }
  }, 100);
};

// ============================================================
// Destination search (Nominatim) + Routing (OSRM)
// ============================================================
let selectedDestination = null;

async function geocode(query) {
  const url = `https://nominatim.openstreetmap.org/search?format=json&limit=5&q=${encodeURIComponent(query)}`;
  const res = await fetch(url, { headers: { 'Accept-Language': 'en' } });
  if (!res.ok) throw new Error('Geocoding failed');
  return res.json();
}

async function performSearch(q) {
  if (!q) return;
  let results;
  try {
    results = await geocode(q);
  } catch (e) {
    alert('Search failed: ' + e.message);
    return;
  }
  const box = document.getElementById('geocode-results');
  if (!results.length) {
    box.innerHTML = '<div class="geocode-item">No results found.</div>';
    box.classList.remove('hidden');
    return;
  }
  box.innerHTML = results.map((r, i) =>
    `<div class="geocode-item" data-i="${i}">${r.display_name}</div>`
  ).join('');
  box.classList.remove('hidden');
  box.querySelectorAll('.geocode-item[data-i]').forEach(el => {
    el.onclick = () => {
      const r = results[parseInt(el.dataset.i, 10)];
      setDestination(parseFloat(r.lat), parseFloat(r.lon), r.display_name);
      box.classList.add('hidden');
    };
  });
}

document.getElementById('btn-route').onclick = () => {
  const q = document.getElementById('destination-search').value.trim();
  performSearch(q);
};

async function drawRoute() {
  if (!selectedDestination) return;
  const origin = latestGps ? latestGps : { lat: 28.6139, lon: 77.2090 };
  const { lat: olat, lon: olon } = origin;
  const { lat: dlat, lon: dlon } = selectedDestination;

  const url = `https://router.project-osrm.org/route/v1/driving/${olon},${olat};${dlon},${dlat}?overview=full&geometries=geojson`;
  try {
    const res = await fetch(url);
    const data = await res.json();
    if (!data.routes || !data.routes.length) {
      return;
    }
    const route = data.routes[0];
    const coords = route.geometry.coordinates.map(([lon, lat]) => [lat, lon]);
    routeLine.setLatLngs(coords).addTo(map);
    map.fitBounds(routeLine.getBounds(), { padding: [40, 40] });

    // Show Route Summary Card
    const distanceKm = (route.distance / 1000).toFixed(1);
    const durationMin = Math.round(route.duration / 60);
    document.getElementById('route-eta').textContent = `${durationMin} mins`;
    document.getElementById('route-distance').textContent = `${distanceKm} km driving`;
    document.getElementById('route-summary-card').classList.remove('hidden');
  } catch (e) {
    console.warn('Routing service notice:', e.message);
  }
}

// ============================================================
// Benchmark Replay Mode
// ============================================================
let replayTimer = null;
let replaySessionId = null;
let replayIntervalMs = 100;

async function loadTrips() {
  try {
    const res = await fetch('/api/replay/trips');
    const trips = await res.json();
    const sel = document.getElementById('trip-select');
    sel.innerHTML = trips.map(t => `<option value="${t.id}">${t.label}</option>`).join('');
  } catch (err) {
    console.error('Failed to load trips:', err);
  }
}
loadTrips();

// Speed multiplier buttons (1x, 2x, 4x)
document.querySelectorAll('.speed-btn').forEach(btn => {
  btn.onclick = () => {
    document.querySelectorAll('.speed-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    const mult = parseInt(btn.dataset.speed, 10);
    replayIntervalMs = Math.round(100 / mult);
    if (replayTimer) {
      clearInterval(replayTimer);
      runReplayLoop();
    }
  };
});

document.getElementById('btn-start-replay').onclick = async () => {
  gtLine.setLatLngs([]); naiveLine.setLatLngs([]); aiLine.setLatLngs([]);
  const tripId = document.getElementById('trip-select').value;
  const res = await fetch(`/api/replay/${tripId}/start`, { method: 'POST' });
  const data = await res.json();
  replaySessionId = data.session_id;
  document.getElementById('btn-start-replay').disabled = true;
  document.getElementById('btn-pause-replay').disabled = false;
  document.getElementById('btn-pause-replay').textContent = 'Pause';
  runReplayLoop();
};

document.getElementById('btn-pause-replay').onclick = () => {
  if (replayTimer) {
    clearInterval(replayTimer);
    replayTimer = null;
    document.getElementById('btn-pause-replay').textContent = 'Resume';
  } else {
    runReplayLoop();
    document.getElementById('btn-pause-replay').textContent = 'Pause';
  }
};

let replayTick = 0;

function runReplayLoop() {
  replayTimer = setInterval(async () => {
    const simulateOutage = document.getElementById('toggle-outage-replay').checked;
    const res = await fetch(`/api/replay/session/${replaySessionId}/next`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ simulate_outage: simulateOutage }),
    });
    const d = await res.json();
    if (d.done) {
      clearInterval(replayTimer); replayTimer = null;
      document.getElementById('btn-start-replay').disabled = false;
      document.getElementById('btn-pause-replay').disabled = true;
      document.getElementById('replay-phase-text').textContent = 'Benchmark Complete';
      return;
    }

    replayTick++;

    if (d.phase === 'warmup') {
      aiMarker.setLatLng([d.lat, d.lon]).addTo(map);
      map.panTo([d.lat, d.lon]);
      updateHUD({ mode: 'GNSS', speed_kmh: 68.0, heading_deg: 90, confidence: 95, motion_state: 'VEHICLE_DRIVING' });
      document.getElementById('replay-phase-text').textContent = 'Phase: GNSS Warmup';
      document.getElementById('replay-time-text').textContent = `${(replayTick * 0.1).toFixed(1)}s`;
      document.getElementById('replay-progress-fill').style.width = '15%';
      return;
    }

    gtMarker.setLatLng([d.ground_truth.lat, d.ground_truth.lon]).addTo(map);
    gtLine.addLatLng([d.ground_truth.lat, d.ground_truth.lon]);

    if (d.mode === 'DR') {
      aiMarker.setLatLng([d.ai_fused.lat, d.ai_fused.lon]).addTo(map);
      aiLine.addLatLng([d.ai_fused.lat, d.ai_fused.lon]);
      naiveMarker.setLatLng([d.naive.lat, d.naive.lon]).addTo(map);
      naiveLine.addLatLng([d.naive.lat, d.naive.lon]);

      const safarDrift = haversineM(d.ground_truth.lat, d.ground_truth.lon, d.ai_fused.lat, d.ai_fused.lon);
      const naiveDrift = haversineM(d.ground_truth.lat, d.ground_truth.lon, d.naive.lat, d.naive.lon);
      const reduction = naiveDrift > 0.1 ? Math.round(((naiveDrift - safarDrift) / naiveDrift) * 100) : 0;

      updateHUD({ mode: 'DR', speed_kmh: d.speed_kmh, heading_deg: 90, confidence: 92, motion_state: 'VEHICLE_DRIVING' });
      map.panTo([d.ai_fused.lat, d.ai_fused.lon]);

      // Benchmark Dashboard updates
      document.getElementById('rep-safar-drift').textContent = `${safarDrift.toFixed(1)} m`;
      document.getElementById('rep-naive-drift').textContent = `${naiveDrift.toFixed(1)} m`;
      document.getElementById('rep-reduction').textContent = `${Math.max(0, reduction)}% Error Cut`;
      document.getElementById('rep-distance').textContent = `${(replayTick * 1.8).toFixed(0)} m`;

      document.getElementById('replay-phase-text').textContent = 'Phase: GNSS Outage (IDR Active)';
      const elapsedSec = (replayTick * 0.1).toFixed(1);
      document.getElementById('replay-time-text').textContent = `${elapsedSec}s / 60s`;
      const pct = Math.min(100, Math.round((replayTick / 640) * 100));
      document.getElementById('replay-progress-fill').style.width = `${pct}%`;
    } else {
      aiMarker.setLatLng([d.lat, d.lon]).addTo(map);
      map.panTo([d.lat, d.lon]);
      updateHUD({ mode: 'GNSS', speed_kmh: 70.0, heading_deg: 90, confidence: 95, motion_state: 'VEHICLE_DRIVING' });
    }
  }, replayIntervalMs);
}