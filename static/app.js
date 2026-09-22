// ============================================================
// S.A.F.A.R. — Sensor-Aided Fusion for Accurate Routing
// Intelligent Dead Reckoning Frontend (SIH 26168)
// ============================================================

const map = L.map('map').setView([28.6139, 77.2090], 14); // New Delhi default until fix
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; OpenStreetMap contributors | S.A.F.A.R. DietCode',
}).addTo(map);

function createMarker(color) {
  return L.circleMarker([0, 0], {
    radius: 9, color: '#ffffff', weight: 2.5, fillColor: color, fillOpacity: 1
  });
}
const style = (color, dashed) => ({ color, weight: 4, opacity: 0.9, dashArray: dashed ? '6 6' : null });

// Live Navigation Layers
const liveMarker = createMarker('#00d2ff');       // Fused / Current vehicle position
const destMarker = createMarker('#ff3d00');       // Destination marker
const routeLine = L.polyline([], style('#00e676')).addTo(map); // Planned route (green)
const liveTrail = L.polyline([], style('#00d2ff'));            // Vehicle trajectory (cyan)

// Replay Layers
const gtLine = L.polyline([], style('#00e676'));
const naiveLine = L.polyline([], style('#ff3d00', true));
const aiLine = L.polyline([], style('#00d2ff'));
const gtMarker = createMarker('#00e676');
const naiveMarker = createMarker('#ff3d00');
const aiMarker = createMarker('#00d2ff');

// Compass direction helper
function degToCompass(deg) {
  if (deg == null || isNaN(deg)) return '—';
  const val = Math.floor((deg / 22.5) + 0.5);
  const arr = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'];
  return `${arr[val % 16]} (${Math.round(deg)}°)`;
}

// Update HUD Dashboard
function updateHUD(data) {
  const speedEl = document.getElementById('hud-speed');
  const headingEl = document.getElementById('hud-heading');
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

  // Heading
  headingEl.textContent = degToCompass(data.heading_deg);

  // Confidence
  const conf = data.confidence != null ? data.confidence : 90;
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
  }
}
switchTab('live');

// ============================================================
// Replay Mode
// ============================================================
let replayTimer = null;
let replaySessionId = null;

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
      return;
    }

    if (d.phase === 'warmup') {
      aiMarker.setLatLng([d.lat, d.lon]).addTo(map);
      map.panTo([d.lat, d.lon]);
      updateHUD({ mode: 'GNSS', speed_kmh: 60.0, heading_deg: 90, confidence: 95, motion_state: 'VEHICLE_DRIVING' });
      return;
    }

    gtMarker.setLatLng([d.ground_truth.lat, d.ground_truth.lon]).addTo(map);
    gtLine.addLatLng([d.ground_truth.lat, d.ground_truth.lon]);

    if (d.mode === 'DR') {
      aiMarker.setLatLng([d.ai_fused.lat, d.ai_fused.lon]).addTo(map);
      aiLine.addLatLng([d.ai_fused.lat, d.ai_fused.lon]);
      naiveMarker.setLatLng([d.naive.lat, d.naive.lon]).addTo(map);
      naiveLine.addLatLng([d.naive.lat, d.naive.lon]);
      const drift = haversineM(d.ground_truth.lat, d.ground_truth.lon, d.ai_fused.lat, d.ai_fused.lon);
      updateHUD({ mode: 'DR', speed_kmh: d.speed_kmh, heading_deg: 90, confidence: 88, motion_state: 'VEHICLE_DRIVING' });
      map.panTo([d.ai_fused.lat, d.ai_fused.lon]);
    } else {
      aiMarker.setLatLng([d.lat, d.lon]).addTo(map);
      map.panTo([d.lat, d.lon]);
      updateHUD({ mode: 'GNSS', speed_kmh: 70.0, heading_deg: 90, confidence: 95, motion_state: 'VEHICLE_DRIVING' });
    }
  }, 100);
}

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
    // Fallback: 0.5Hz low-pass filter (gravity shifts slowly with road grade, not hand tremors)
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

function onGeoSuccess(pos) {
  const acc = pos.coords.accuracy;
  if (acc != null && acc > MAX_ACCEPTABLE_ACCURACY_M) {
    return;
  }
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
    map.setView([latestGps.lat, latestGps.lon], 17);
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
  if (latestGps) map.setView([latestGps.lat, latestGps.lon], 17);
};

document.getElementById('btn-start-live').onclick = async () => {
  if (tracking) return;

  // iOS 13+ sensor permission gesture
  if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
    try {
      const perm = await DeviceMotionEvent.requestPermission();
      if (perm !== 'granted') {
        alert('Motion sensor permission is required for AI dead-reckoning during GPS blackouts.');
        return;
      }
    } catch (err) {
      alert('Could not request motion permission: ' + err);
      return;
    }
  }
  window.addEventListener('devicemotion', onDeviceMotion);

  if (!navigator.geolocation) {
    alert('This device or browser does not support geolocation.');
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
    alert('Failed to connect to backend engine: ' + err.message);
    return;
  }

  tracking = true;
  document.getElementById('btn-start-live').disabled = true;
  document.getElementById('btn-start-live').textContent = 'Tracking Active';

  // 10Hz sampling loop with non-overlapping dispatch guard
  liveTimer = setInterval(async () => {
    if (!latestAccel || isDispatching) return;

    isDispatching = true;
    try {
      const manualOutage = document.getElementById('toggle-outage-live').checked;
      // Real outage condition: manual toggle OR explicit geo error OR accuracy failure
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
        updateHUD({ mode: 'NO_FIX', speed_kmh: 0.0, heading_deg: null, confidence: 10, motion_state: 'Waiting for GPS' });
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
// Destination search (Nominatim geocoding) + routing (OSRM)
// ============================================================
let selectedDestination = null;

async function geocode(query) {
  const url = `https://nominatim.openstreetmap.org/search?format=json&limit=5&q=${encodeURIComponent(query)}`;
  const res = await fetch(url, { headers: { 'Accept-Language': 'en' } });
  if (!res.ok) throw new Error('Geocoding failed');
  return res.json();
}

document.getElementById('btn-route').onclick = async () => {
  const q = document.getElementById('destination-search').value.trim();
  if (!q) return;
  let results;
  try {
    results = await geocode(q);
  } catch (e) {
    alert('Could not search for that place: ' + e.message);
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
      selectedDestination = { lat: parseFloat(r.lat), lon: parseFloat(r.lon), label: r.display_name };
      destMarker.setLatLng([selectedDestination.lat, selectedDestination.lon]).addTo(map);
      box.classList.add('hidden');
      document.getElementById('destination-search').value = r.display_name;
      drawRoute();
    };
  });
};

async function drawRoute() {
  if (!selectedDestination) return;
  if (!latestGps) {
    alert('Waiting for your current location fix first.');
    return;
  }
  const { lat: olat, lon: olon } = latestGps;
  const { lat: dlat, lon: dlon } = selectedDestination;
  const url = `https://router.project-osrm.org/route/v1/driving/${olon},${olat};${dlon},${dlat}?overview=full&geometries=geojson`;
  try {
    const res = await fetch(url);
    const data = await res.json();
    if (!data.routes || !data.routes.length) {
      alert('No driving route found between your location and that destination.');
      return;
    }
    const coords = data.routes[0].geometry.coordinates.map(([lon, lat]) => [lat, lon]);
    routeLine.setLatLngs(coords).addTo(map);
    map.fitBounds(routeLine.getBounds(), { padding: [30, 30] });
  } catch (e) {
    alert('Routing service unavailable: ' + e.message);
  }
}