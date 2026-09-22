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
  zoomControl: false, // Clean mobile-first UI
  attributionControl: false,
}).setView(DEFAULT_CENTER, 15);

// OpenStreetMap Standard Tiles
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
  attribution: '&copy; OpenStreetMap | S.A.F.A.R. Team DietCode',
}).addTo(map);

// Add compact attribution at bottom-right
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

// Destination Marker
let destMarker = null;
let selectedDestination = null;

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

  // 2. Rotate the compass needle icon in the bottom HUD
  const needle = document.getElementById('hud-compass-needle');
  if (needle) {
    needle.style.transform = `rotate(${currentRotationDeg}deg)`;
  }

  // 3. Update the text in the bottom HUD
  const headingEl = document.getElementById('hud-heading');
  if (headingEl) {
    headingEl.textContent = degToCompass(currentRotationDeg);
  }
}

function degToCompass(deg) {
  const points = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW'];
  const idx = Math.floor((deg / 22.5) + 0.5) % 16;
  return `${points[idx]} (${Math.round(deg)}°)`;
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
    // Android alpha is counter-clockwise rotation from North
    heading = (360 - e.alpha) % 360;
  }

  if (heading !== null && !isNaN(heading)) {
    // Screen orientation offset (portrait vs landscape)
    const screenAngle = (window.orientation || (screen.orientation && screen.orientation.angle) || 0);
    heading = (heading + screenAngle + 360) % 360;

    latestCompassHeading = heading;
    updateHeadingUI(heading);
  }
}

// Attach compass listeners immediately
function attachOrientationListeners() {
  if ('ondeviceorientationabsolute' in window) {
    window.addEventListener('deviceorientationabsolute', handleDeviceOrientation, true);
  }
  window.addEventListener('deviceorientation', handleDeviceOrientation, true);
}

// Check iOS 13+ sensor permissions
function checkSensorPermissions() {
  if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
    // Show iOS permission modal
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
    // Android, PC, or older iOS — attach immediately
    attachOrientationListeners();
  }
}

// Auto-request on first screen touch as fallback for iOS
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
let hasMotionSensor = false;

function handleDeviceMotion(e) {
  const a = e.accelerationIncludingGravity;
  if (!a || a.x == null) return;
  hasMotionSensor = true;

  const lin = e.acceleration;
  if (lin && lin.x != null) {
    // Hardware-isolated linear acceleration from smartphone IMU chip
    latestAccel = { x: a.x || 0, y: a.y || 0, z: a.z || 0 };
    latestGravityEst = {
      x: (a.x || 0) - (lin.x || 0),
      y: (a.y || 0) - (lin.y || 0),
      z: (a.z || 0) - (lin.z || 0),
    };
  } else {
    // Calibrated 0.5Hz low-pass gravity filter
    latestAccel = { x: a.x || 0, y: a.y || 0, z: a.z || 0 };
    const alpha = 0.94;
    latestGravityEst = {
      x: alpha * latestGravityEst.x + (1 - alpha) * (a.x || 0),
      y: alpha * latestGravityEst.y + (1 - alpha) * (a.y || 0),
      z: alpha * latestGravityEst.z + (1 - alpha) * (a.z || 0),
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
window.addEventListener('devicemotion', handleDeviceMotion, true);

// ------------------------------------------------------------
// 4. Geolocation (GNSS GPS Tracking)
// ------------------------------------------------------------
let latestGps = null;
let hasGpsFix = false;
let userHasPanned = false;
const MAX_ACCEPTABLE_ACCURACY_M = 100;

function onGeoSuccess(pos) {
  const acc = pos.coords.accuracy;
  if (acc != null && acc > MAX_ACCEPTABLE_ACCURACY_M) return;

  const lat = pos.coords.latitude;
  const lon = pos.coords.longitude;
  const speed = pos.coords.speed;
  const heading = pos.coords.heading;

  latestGps = { lat, lon, speed, heading, accuracy: acc };

  if (!hasGpsFix) {
    hasGpsFix = true;
    map.setView([lat, lon], 16, { animate: true });
    liveMarker.setLatLng([lat, lon]);
    liveTrail.setLatLngs([[lat, lon]]);
  }

  // If phone has no compass hardware, fallback to GPS heading if moving
  if (latestCompassHeading == null && heading != null && !isNaN(heading) && (speed || 0) > 1.0) {
    updateHeadingUI(heading);
  }
}

function onGeoError(err) {
  console.warn('Geolocation notice:', err.message);
}

if (navigator.geolocation) {
  navigator.geolocation.watchPosition(onGeoSuccess, onGeoError, {
    enableHighAccuracy: true,
    maximumAge: 1000,
    timeout: 10000,
  });
}

// Detect manual user drag so we don't snap the map back abruptly
map.on('dragstart', () => {
  userHasPanned = true;
  document.getElementById('btn-recenter').classList.remove('active');
});

// Re-center Button
document.getElementById('btn-recenter').onclick = () => {
  userHasPanned = false;
  document.getElementById('btn-recenter').classList.add('active');
  const target = latestGps ? [latestGps.lat, latestGps.lon] : liveMarker.getLatLng();
  map.setView(target, 16, { animate: true });
};

// ------------------------------------------------------------
// 5. Backend Live Session & 10Hz Dead Reckoning Loop
// ------------------------------------------------------------
let liveSessionId = null;
let isDispatching = false;
let isSimulatedOutage = false;

async function initLiveBackendSession() {
  try {
    const res = await fetch('/api/live/session/start', { method: 'POST' });
    if (res.ok) {
      const data = await res.json();
      liveSessionId = data.session_id;
    }
  } catch (err) {
    console.warn('Backend live session init:', err.message);
  }
}
initLiveBackendSession();

// Outage Toggle Switch
const toggleOutage = document.getElementById('toggle-outage');
if (toggleOutage) {
  toggleOutage.onchange = () => {
    isSimulatedOutage = toggleOutage.checked;
    updateModeDisplay(isSimulatedOutage ? 'DR' : 'GNSS');
  };
}

function updateModeDisplay(mode) {
  const badge = document.getElementById('mode-badge');
  const text = document.getElementById('mode-text');
  const banner = document.getElementById('dr-banner');

  if (mode === 'DR') {
    badge.className = 'mode-badge mode-dr';
    text.textContent = 'Dead Reckoning (IDR)';
    banner.classList.remove('hidden');
  } else {
    badge.className = 'mode-badge mode-gnss';
    text.textContent = 'GNSS Active';
    banner.classList.add('hidden');
  }
}

// Update HUD Dashboard
function updateHUD(data) {
  const speedEl = document.getElementById('hud-speed');
  const confEl = document.getElementById('hud-conf');
  const motionEl = document.getElementById('hud-motion');

  // Speed
  if (data.speed_kmh != null) {
    speedEl.textContent = data.speed_kmh.toFixed(1);
  }

  // Heading fallback if no live compass hardware event
  if (latestCompassHeading == null && data.heading_deg != null) {
    updateHeadingUI(data.heading_deg);
  }

  // Confidence Level
  const conf = data.confidence != null ? data.confidence : 95;
  confEl.textContent = `${conf}%`;
  confEl.className = 'hud-bold ' + (conf >= 75 ? 'conf-high' : conf >= 50 ? 'conf-med' : 'conf-low');

  // Motion State
  const motionMap = {
    'STATIONARY': 'Stationary (ZUPT)',
    'HAND_DISTURBANCE': 'Disturbance Filtered',
    'VEHICLE_DRIVING': 'Vehicle Kinematics',
  };
  motionEl.textContent = motionMap[data.motion_state] || (data.motion_state || 'Stationary (ZUPT)');

  // Mode Display
  updateModeDisplay(data.mode);
}

// 10Hz Telemetry & S.A.F.A.R. Fusion Loop
setInterval(async () => {
  if (!liveSessionId || isDispatching) return;

  isDispatching = true;
  try {
    const isOutage = isSimulatedOutage || !hasGpsFix;
    const body = {
      accel: latestAccel,
      gravity: latestGravityEst,
      gyro: latestGyro,
      gps: (!isOutage && latestGps) ? latestGps : null,
      simulate_outage: isSimulatedOutage,
      dt: 0.1,
    };

    const res = await fetch(`/api/live/session/${liveSessionId}/sample`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });

    if (res.ok) {
      const data = await res.json();
      if (data.lat != null && data.lon != null) {
        liveMarker.setLatLng([data.lat, data.lon]);
        liveTrail.addLatLng([data.lat, data.lon]);

        if (!userHasPanned) {
          map.panTo([data.lat, data.lon], { animate: true, duration: 0.1 });
        }
      }
      updateHUD(data);
    }
  } catch (err) {
    console.warn('Live sample sync:', err.message);
  } finally {
    isDispatching = false;
  }
}, 100);

// ------------------------------------------------------------
// 6. Destination Search Autocomplete & Pin-Drop Routing
// ------------------------------------------------------------
const searchInput = document.getElementById('destination-search');
const suggestionsBox = document.getElementById('suggestions-box');
const btnClearSearch = document.getElementById('btn-clear-search');
const routeCard = document.getElementById('route-card');
const btnCancelRoute = document.getElementById('btn-cancel-route');
const btnDropPin = document.getElementById('btn-drop-pin');

let isPinModeActive = false;
let searchDebounceTimer = null;

// Toggle Tap-to-Drop Pin Mode
btnDropPin.onclick = () => {
  isPinModeActive = !isPinModeActive;
  btnDropPin.classList.toggle('active', isPinModeActive);
  map.getContainer().style.cursor = isPinModeActive ? 'crosshair' : '';
};

// Map Click / Tap Handler: Pinpoint Any Destination on Map
map.on('click', async (e) => {
  const lat = e.latlng.lat;
  const lon = e.latlng.lng;
  const label = `Pinned (${lat.toFixed(4)}, ${lon.toFixed(4)})`;

  setDestination(lat, lon, label);

  // Optional: Reverse geocode to get real street name
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

// Set Destination & Calculate Route
async function setDestination(lat, lon, label) {
  selectedDestination = { lat, lon, label };

  // Update or create Red Destination Pin
  if (!destMarker) {
    destMarker = L.marker([lat, lon], { icon: destPinIcon, zIndexOffset: 900 }).addTo(map);
  } else {
    destMarker.setLatLng([lat, lon]).addTo(map);
  }

  // Update Search input & clear button
  searchInput.value = label;
  btnClearSearch.classList.remove('hidden');
  suggestionsBox.classList.add('hidden');

  // Compute driving route
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
    const coords = route.geometry.coordinates.map(([lon, lat]) => [lat, lon]);
    routeLine.setLatLngs(coords);

    // Zoom map to fit the route nicely
    map.fitBounds(routeLine.getBounds(), { padding: [60, 60] });

    // Show Route ETA Card
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

// Cancel Route & Clear Markers
function clearCurrentRoute() {
  selectedDestination = null;
  if (destMarker) destMarker.remove();
  routeLine.setLatLngs([]);
  routeCard.classList.add('hidden');
  searchInput.value = '';
  btnClearSearch.classList.add('hidden');
  suggestionsBox.classList.add('hidden');
}

btnCancelRoute.onclick = clearCurrentRoute;
btnClearSearch.onclick = clearCurrentRoute;

// Search Autocomplete (Nominatim with 250ms Debounce)
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
        suggestionsBox.innerHTML = `<div class="suggestion-item"><div class="sugg-text"><span class="sugg-sub">No places found for "${query}"</span></div></div>`;
        suggestionsBox.classList.remove('hidden');
        return;
      }

      suggestionsBox.innerHTML = results.map((item, idx) => {
        const parts = item.display_name.split(',');
        const main = parts[0].trim();
        const sub = parts.slice(1, 4).join(',').trim();
        return `
          <div class="suggestion-item" data-idx="${idx}">
            <div class="sugg-pin-icon">📍</div>
            <div class="sugg-text">
              <span class="sugg-main">${main}</span>
              <span class="sugg-sub">${sub || item.display_name}</span>
            </div>
          </div>
        `;
      }).join('');

      suggestionsBox.classList.remove('hidden');

      // Bind click handlers to each suggestion
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

// Search on Enter key
searchInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') {
    const firstItem = suggestionsBox.querySelector('.suggestion-item');
    if (firstItem) {
      firstItem.click();
    }
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