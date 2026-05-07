'use strict';

// ═══════════════════════════════════════════════════════
//  CHCNAV X500 GCS — app.js
//  Production Ground Control Station
// ═══════════════════════════════════════════════════════

const GCS = (() => {

  // ── State ──────────────────────────────────────────────
  const state = {
    viewer: null,
    mode: 'IDLE',           // IDLE | WAYPOINT | SURVEY | TERRAIN
    waypoints: [],
    selectedWP: -1,
    missionType: 'grid',    // grid | corridor | lidar
    surveyPolygon: [],
    terrainFollowing: false,
    footprintVisible: false,
    airspaceLoaded: false,
    noFlyZones: [],
    telemetry: {
      battery: 87,
      altitude: 0,
      groundSpeed: 0,
      gpsCount: 18,
      rtkStatus: 'FIXED',
      signal: 5,
      heading: 0,
      verticalSpeed: 0,
      latitude: 30.5728,
      longitude: 104.0668,
      hdop: 0.8,
      temperature: 22,
      voltage: 22.4,
      current: 14.2,
      flightTime: 0,
    },
    mission: {
      name: 'MISSION_001',
      altitude: 80,
      speed: 8,
      overlap: { side: 70, forward: 80 },
      corridorWidth: 50,
      heading: 0,
      turnRadius: 5,
      distance: 0,
      flightTime: 0,
      area: 0,
      lines: 0,
    },
    camera: {
      sensorW: 17.3,
      sensorH: 13.0,
      focalLength: 24,
    },
    entities: {
      waypoints: [],
      flightPath: null,
      surveyLines: [],
      footprint: null,
      noFlyZones: [],
      terrainPath: null,
    },
    logs: [],
    savedMissions: [],
    telemetryTimer: null,
    flightTimer: null,
  };

  // ── Helpers ────────────────────────────────────────────
  const $  = id => document.getElementById(id);
  const $$ = sel => document.querySelectorAll(sel);
  const now = () => new Date().toLocaleTimeString('en-US', { hour12: false });
  const deg = r => r * 180 / Math.PI;
  const rad = d => d * Math.PI / 180;

  function fmt(n, dec = 1) { return Number(n).toFixed(dec); }
  function fmtDist(m) {
    return m >= 1000 ? `${fmt(m/1000,2)} km` : `${fmt(m,0)} m`;
  }
  function fmtTime(sec) {
    const m = Math.floor(sec / 60), s = Math.floor(sec % 60);
    return `${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`;
  }

  // haversine distance meters
  function haversine(lat1, lon1, lat2, lon2) {
    const R = 6371000;
    const φ1 = rad(lat1), φ2 = rad(lat2);
    const Δφ = rad(lat2 - lat1), Δλ = rad(lon2 - lon1);
    const a = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  }

  // bearing degrees
  function bearing(lat1, lon1, lat2, lon2) {
    const φ1 = rad(lat1), φ2 = rad(lat2);
    const Δλ = rad(lon2 - lon1);
    const y = Math.sin(Δλ) * Math.cos(φ2);
    const x = Math.cos(φ1)*Math.sin(φ2) - Math.sin(φ1)*Math.cos(φ2)*Math.cos(Δλ);
    return (deg(Math.atan2(y, x)) + 360) % 360;
  }

  // destination given origin, bearing, distance
  function destination(lat, lon, brg, dist) {
    const R = 6371000;
    const δ = dist / R;
    const φ1 = rad(lat), λ1 = rad(lon), θ = rad(brg);
    const φ2 = Math.asin(Math.sin(φ1)*Math.cos(δ) + Math.cos(φ1)*Math.sin(δ)*Math.cos(θ));
    const λ2 = λ1 + Math.atan2(Math.sin(θ)*Math.sin(δ)*Math.cos(φ1), Math.cos(δ)-Math.sin(φ1)*Math.sin(φ2));
    return { lat: deg(φ2), lon: deg(λ2) };
  }

  function pointInCircle(lat, lon, cLat, cLon, radiusKm) {
    return haversine(lat, lon, cLat, cLon) < radiusKm * 1000;
  }

  function pointInPolygon(lat, lon, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const xi = poly[i][0], yi = poly[i][1];
      const xj = poly[j][0], yj = poly[j][1];
      if (((yi > lon) !== (yj > lon)) && (lat < (xj - xi) * (lon - yi) / (yj - yi) + xi)) {
        inside = !inside;
      }
    }
    return inside;
  }

  function checkAirspaceConflict(lat, lon) {
    for (const zone of state.noFlyZones) {
      const g = zone.geometry;
      const p = zone.properties;
      if (g.type === 'Point') {
        if (pointInCircle(lat, lon, g.coordinates[1], g.coordinates[0], p.radius_km || 5)) return p;
      } else if (g.type === 'Polygon') {
        const ring = g.coordinates[0].map(c => [c[1], c[0]]);
        if (pointInPolygon(lat, lon, ring)) return p;
      }
    }
    return null;
  }

  // ── Logging ────────────────────────────────────────────
  function log(msg, level = 'info') {
    const entry = { time: now(), level, msg };
    state.logs.unshift(entry);
    if (state.logs.length > 200) state.logs.pop();
    renderLogs();
  }

  function renderLogs() {
    const pane = $('console-log-pane');
    if (!pane) return;
    pane.innerHTML = state.logs.map(e => `
      <div class="log-entry">
        <span class="log-time">${e.time}</span>
        <span class="log-level ${e.level}">${e.level.toUpperCase()}</span>
        <span class="log-msg">${e.msg}</span>
      </div>`).join('');
  }

  // ── Time display ───────────────────────────────────────
  function startClock() {
    setInterval(() => {
      const el = $('sys-time');
      if (el) el.textContent = new Date().toLocaleTimeString('en-US', { hour12: false });
    }, 1000);
  }

  // ── Telemetry simulation ───────────────────────────────
  function startTelemetry() {
    state.telemetryTimer = setInterval(updateTelemetry, 3000);
    updateTelemetryUI();
  }

  function updateTelemetry() {
    const t = state.telemetry;
    t.battery = Math.max(0, t.battery - (Math.random() * 0.3));
    t.gpsCount = Math.round(18 + (Math.random() * 4 - 2));
    t.hdop = +(0.6 + Math.random() * 0.4).toFixed(2);
    t.signal = Math.round(Math.max(1, Math.min(5, t.signal + (Math.random() * 2 - 1))));
    t.temperature = +(t.temperature + (Math.random() * 0.4 - 0.2)).toFixed(1);
    t.voltage = +(22.4 - (1 - t.battery/100) * 4).toFixed(2);
    t.current = +(12 + Math.random() * 6).toFixed(1);

    if (state.mode !== 'IDLE') {
      t.groundSpeed = +(state.mission.speed + (Math.random() * 0.5 - 0.25)).toFixed(1);
      t.verticalSpeed = +(Math.random() * 0.4 - 0.2).toFixed(2);
    } else {
      t.groundSpeed = 0;
      t.verticalSpeed = 0;
    }

    if (t.battery < 20) t.rtkStatus = 'FLOAT';
    if (t.battery < 10) t.rtkStatus = 'NONE';

    updateTelemetryUI();

    if (t.battery < 25) log(`Battery low: ${fmt(t.battery,1)}%`, 'warn');
  }

  function updateTelemetryUI() {
    const t = state.telemetry;

    // Battery
    const batPct = fmt(t.battery, 1);
    setTG('tg-battery', batPct, '%', batPct);
    const batColor = t.battery > 50 ? 'var(--green)' : t.battery > 25 ? 'var(--yellow)' : 'var(--red)';
    const batFill = $('tg-battery-fill');
    if (batFill) { batFill.style.width = batPct + '%'; batFill.style.background = batColor; }

    // GPS
    setTG('tg-gps', t.gpsCount, 'SAT', 100);
    const gpsBar = $('tg-gps-fill');
    if (gpsBar) gpsBar.style.width = Math.min(100, t.gpsCount * 5) + '%';

    // Speed
    setTG('tg-speed', fmt(t.groundSpeed, 1), 'm/s', Math.min(100, t.groundSpeed * 10));

    // Altitude
    const alt = fmt(t.altitude, 0);
    setTG('tg-alt', alt, 'm', Math.min(100, t.altitude / 2));

    // Voltage
    setTG('tg-voltage', fmt(t.voltage, 1), 'V', Math.min(100, (t.voltage - 18) / 6 * 100));

    // Signal
    const sigs = $$('.signal-bar');
    sigs.forEach((b, i) => b.classList.toggle('active', i < t.signal));

    // RTK
    const rtkEl = $('rtk-status-badge');
    if (rtkEl) {
      rtkEl.textContent = t.rtkStatus;
      rtkEl.className = 'rtk-status ' + t.rtkStatus.toLowerCase();
    }
    const rtkSats = $('rtk-sats');
    if (rtkSats) rtkSats.innerHTML = `GPS: <span>${t.gpsCount}</span> &nbsp; HDOP: <span>${t.hdop}</span>`;

    // Coords
    const coordEl = $('hud-coords');
    if (coordEl) coordEl.innerHTML = `<span>${fmt(t.latitude,6)}°</span> / <span>${fmt(t.longitude,6)}°</span>`;
  }

  function setTG(id, val, unit, pct) {
    const el = $(id);
    if (!el) return;
    const valEl = el.querySelector('.tg-value');
    const barEl = el.querySelector('.tg-bar-fill');
    if (valEl) valEl.innerHTML = `${val}<span class="tg-unit">${unit}</span>`;
    if (barEl) barEl.style.width = Math.max(0, Math.min(100, pct)) + '%';
  }

  // ── Cesium Init ────────────────────────────────────────
  async function initCesium() {
    try {
      Cesium.Ion.defaultAccessToken = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiJlYWE1OWUxNy1mMWZiLTQzYjYtYTQ0OS1kMWFjYmFkNjc4ZDkiLCJpZCI6NTc3MzMsImlhdCI6MTYyMjY0NDA0Mn0.XcKpgANiY19MC4bdFUXMVEBToBmqS8kuYpUlxJHYZxk';

      state.viewer = new Cesium.Viewer('cesium-container', {
        terrainProvider: await Cesium.createWorldTerrainAsync({
          requestWaterMask: false,
          requestVertexNormals: false,
        }),
        imageryProvider: new Cesium.IonImageryProvider({ assetId: 2 }),
        baseLayerPicker: false,
        geocoder: false,
        homeButton: false,
        sceneModePicker: false,
        navigationHelpButton: false,
        animation: false,
        timeline: false,
        fullscreenButton: false,
        infoBox: false,
        selectionIndicator: false,
        shadows: false,
        skyAtmosphere: new Cesium.SkyAtmosphere(),
        skyBox: new Cesium.SkyBox({
          sources: {
            positiveX: 'https://cesium.com/downloads/cesiumjs/releases/1.118/Build/Cesium/Assets/Textures/SkyBox/tycho2t3_80_px.jpg',
            negativeX: 'https://cesium.com/downloads/cesiumjs/releases/1.118/Build/Cesium/Assets/Textures/SkyBox/tycho2t3_80_nx.jpg',
            positiveY: 'https://cesium.com/downloads/cesiumjs/releases/1.118/Build/Cesium/Assets/Textures/SkyBox/tycho2t3_80_py.jpg',
            negativeY: 'https://cesium.com/downloads/cesiumjs/releases/1.118/Build/Cesium/Assets/Textures/SkyBox/tycho2t3_80_ny.jpg',
            positiveZ: 'https://cesium.com/downloads/cesiumjs/releases/1.118/Build/Cesium/Assets/Textures/SkyBox/tycho2t3_80_pz.jpg',
            negativeZ: 'https://cesium.com/downloads/cesiumjs/releases/1.118/Build/Cesium/Assets/Textures/SkyBox/tycho2t3_80_nz.jpg',
          }
        }),
        creditContainer: document.createElement('div'),
      });

      // Scene settings
      state.viewer.scene.globe.enableLighting = false;
      state.viewer.scene.globe.depthTestAgainstTerrain = true;
      state.viewer.scene.screenSpaceCameraController.enableLook = true;
      state.viewer.scene.fog.enabled = true;
      state.viewer.scene.fog.density = 0.0002;

      // Dark atmosphere tint
      state.viewer.scene.skyAtmosphere.hueShift = -0.97;
      state.viewer.scene.skyAtmosphere.saturationShift = 0.25;
      state.viewer.scene.skyAtmosphere.brightnessShift = -0.4;

      // Fly to default location (Chengdu, China — CHCNAV HQ)
      state.viewer.camera.flyTo({
        destination: Cesium.Cartesian3.fromDegrees(104.0668, 30.5728, 8000),
        orientation: { heading: 0, pitch: Cesium.Math.toRadians(-45), roll: 0 },
        duration: 2.5,
      });

      // Click handler
      const handler = new Cesium.ScreenSpaceEventHandler(state.viewer.scene.canvas);
      handler.setInputAction(e => onMapClick(e), Cesium.ScreenSpaceEventType.LEFT_CLICK);
      handler.setInputAction(e => onMapMove(e), Cesium.ScreenSpaceEventType.MOUSE_MOVE);

      log('Cesium 3D globe initialized', 'success');
      log('World terrain loaded — CesiumIon', 'info');
      log(`Camera: ${fmt(30.5728,4)}°N ${fmt(104.0668,4)}°E — Chengdu`, 'info');

      // Load airspace
      await loadAirspace();

      return true;
    } catch (err) {
      log(`Cesium init error: ${err.message}`, 'error');
      console.error(err);
      return false;
    }
  }

  // ── Map Events ─────────────────────────────────────────
  function onMapClick(e) {
    if (!state.viewer) return;

    const ellipsoid = state.viewer.scene.globe.ellipsoid;
    const cartesian = state.viewer.camera.pickEllipsoid(e.position, ellipsoid);
    if (!cartesian) return;

    const carto = ellipsoid.cartesianToCartographic(cartesian);
    const lon = deg(carto.longitude);
    const lat = deg(carto.latitude);

    if (state.mode === 'WAYPOINT') {
      addWaypoint(lat, lon);
    } else if (state.mode === 'SURVEY') {
      addSurveyVertex(lat, lon);
    }
  }

  function onMapMove(e) {
    if (!state.viewer) return;
    const ellipsoid = state.viewer.scene.globe.ellipsoid;
    const cartesian = state.viewer.camera.pickEllipsoid(e.endPosition, ellipsoid);
    if (!cartesian) return;
    const carto = ellipsoid.cartesianToCartographic(cartesian);
    const lon = deg(carto.longitude);
    const lat = deg(carto.latitude);
    const elev = Math.round(carto.height);

    const hudEl = $('hud-mouse-pos');
    if (hudEl) hudEl.innerHTML = `
      <span class="hud-item">LAT <span>${fmt(lat,5)}°</span></span>
      <span class="hud-item">LON <span>${fmt(lon,5)}°</span></span>
      <span class="hud-item">ELEV <span>${elev} m</span></span>
    `;

    if (state.footprintVisible && state.waypoints.length > 0) {
      updateFootprintPreview(lat, lon);
    }
  }

  // ── Waypoint Management ────────────────────────────────
  async function addWaypoint(lat, lon) {
    const alt = parseInt($('wp-altitude')?.value || state.mission.altitude);
    const speed = parseFloat($('wp-speed')?.value || state.mission.speed);

    // Airspace check
    const conflict = checkAirspaceConflict(lat, lon);
    if (conflict) {
      showAirspaceAlert(conflict.name);
      log(`Waypoint REJECTED — restricted airspace: ${conflict.name}`, 'error');
      return;
    }

    // Terrain elevation
    let groundElev = 0;
    try {
      const positions = [Cesium.Cartographic.fromDegrees(lon, lat)];
      const updated = await Cesium.sampleTerrainMostDetailed(state.viewer.terrainProvider, positions);
      groundElev = Math.round(updated[0].height || 0);
    } catch {}

    const wp = { lat, lon, alt, speed, groundElev, agl: alt };
    state.waypoints.push(wp);

    // Terrain following adjustment
    if (state.terrainFollowing) {
      wp.alt = groundElev + alt;
      wp.agl = alt;
    }

    state.telemetry.altitude = wp.alt;
    state.telemetry.latitude = lat;
    state.telemetry.longitude = lon;

    addWaypointEntity(wp, state.waypoints.length - 1);
    updateFlightPath();
    updateWaypointList();
    updateMissionStats();
    updateTelemetryUI();

    log(`WP${String(state.waypoints.length).padStart(2,'0')} added — ${fmt(lat,5)}°, ${fmt(lon,5)}° ALT:${wp.alt}m GND:${groundElev}m`, 'info');
  }

  function addWaypointEntity(wp, idx) {
    if (!state.viewer) return;

    const entity = state.viewer.entities.add({
      position: Cesium.Cartesian3.fromDegrees(wp.lon, wp.lat, wp.alt),
      point: {
        pixelSize: 12,
        color: Cesium.Color.fromCssColorString('#00d4ff'),
        outlineColor: Cesium.Color.fromCssColorString('#003d55'),
        outlineWidth: 2,
        heightReference: Cesium.HeightReference.NONE,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
      label: {
        text: `WP${String(idx+1).padStart(2,'0')}`,
        font: '10px "Share Tech Mono", monospace',
        fillColor: Cesium.Color.fromCssColorString('#00d4ff'),
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
        pixelOffset: new Cesium.Cartesian2(0, -14),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    });

    // Vertical stem line
    const stem = state.viewer.entities.add({
      polyline: {
        positions: [
          Cesium.Cartesian3.fromDegrees(wp.lon, wp.lat, 0),
          Cesium.Cartesian3.fromDegrees(wp.lon, wp.lat, wp.alt),
        ],
        width: 1,
        material: new Cesium.ColorMaterialProperty(Cesium.Color.fromCssColorString('#00d4ff').withAlpha(0.3)),
        clampToGround: false,
      }
    });

    state.entities.waypoints.push({ point: entity, stem });
  }

  function updateFlightPath() {
    if (!state.viewer) return;

    if (state.entities.flightPath) {
      state.viewer.entities.remove(state.entities.flightPath);
      state.entities.flightPath = null;
    }

    if (state.waypoints.length < 2) return;

    const positions = state.waypoints.map(wp =>
      Cesium.Cartesian3.fromDegrees(wp.lon, wp.lat, wp.alt)
    );

    state.entities.flightPath = state.viewer.entities.add({
      polyline: {
        positions,
        width: 2,
        material: new Cesium.PolylineGlowMaterialProperty({
          glowPower: 0.2,
          color: Cesium.Color.fromCssColorString('#00d4ff'),
        }),
        clampToGround: false,
        depthFailMaterial: new Cesium.ColorMaterialProperty(
          Cesium.Color.fromCssColorString('#00d4ff').withAlpha(0.4)
        ),
      }
    });
  }

  function removeWaypoint(idx) {
    if (idx < 0 || idx >= state.waypoints.length) return;

    const ents = state.entities.waypoints[idx];
    if (ents) {
      state.viewer.entities.remove(ents.point);
      state.viewer.entities.remove(ents.stem);
    }
    state.entities.waypoints.splice(idx, 1);
    state.waypoints.splice(idx, 1);

    // Re-label all waypoints
    state.entities.waypoints.forEach((ent, i) => {
      if (ent.point.label) ent.point.label.text = new Cesium.ConstantProperty(`WP${String(i+1).padStart(2,'0')}`);
    });

    updateFlightPath();
    updateWaypointList();
    updateMissionStats();
    log(`WP${String(idx+1).padStart(2,'0')} removed`, 'warn');
  }

  function clearAllWaypoints() {
    state.entities.waypoints.forEach(ents => {
      state.viewer.entities.remove(ents.point);
      state.viewer.entities.remove(ents.stem);
    });
    state.entities.waypoints = [];
    state.waypoints = [];

    if (state.entities.flightPath) {
      state.viewer.entities.remove(state.entities.flightPath);
      state.entities.flightPath = null;
    }

    clearSurveyLines();
    state.surveyPolygon = [];

    updateWaypointList();
    updateMissionStats();
    log('Mission cleared', 'warn');
  }

  function updateWaypointList() {
    const list = $('waypoint-list');
    if (!list) return;

    if (state.waypoints.length === 0) {
      list.innerHTML = `<div style="text-align:center;padding:20px;font-size:11px;color:var(--text-muted)">
        Click map to add waypoints
      </div>`;
      return;
    }

    list.innerHTML = state.waypoints.map((wp, i) => `
      <div class="waypoint-item${state.selectedWP === i ? ' selected' : ''}" onclick="GCS.selectWP(${i})">
        <div class="wp-index">${i+1}</div>
        <div class="wp-coords">${fmt(wp.lat,4)}°<br>${fmt(wp.lon,4)}°</div>
        <div class="wp-alt">${wp.alt}m</div>
        <button class="wp-del" onclick="event.stopPropagation();GCS.removeWP(${i})" title="Remove">
          <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <line x1="18" y1="6" x2="6" y2="18"/><line x1="6" y1="6" x2="18" y2="18"/>
          </svg>
        </button>
      </div>`).join('');
  }

  function updateMissionStats() {
    let dist = 0;
    for (let i = 1; i < state.waypoints.length; i++) {
      const a = state.waypoints[i-1], b = state.waypoints[i];
      dist += haversine(a.lat, a.lon, b.lat, b.lon);
    }
    state.mission.distance = dist;
    const speed = state.mission.speed || 8;
    state.mission.flightTime = dist / speed;

    // Update stats in UI
    setStatVal('stat-distance', fmtDist(dist));
    setStatVal('stat-flight-time', fmtTime(state.mission.flightTime));
    setStatVal('stat-waypoints', state.waypoints.length);
    setStatVal('stat-altitude', state.mission.altitude + ' m');

    // Right panel mission summary
    setMS('ms-waypoints', state.waypoints.length);
    setMS('ms-distance', fmtDist(dist));
    setMS('ms-duration', fmtTime(state.mission.flightTime));
    setMS('ms-altitude', state.mission.altitude + ' m');
    setMS('ms-speed', state.mission.speed + ' m/s');
  }

  function setStatVal(id, val) { const el = $(id); if (el) el.textContent = val; }
  function setMS(id, val) { const el = $(id); if (el) el.textContent = val; }

  // ── Survey Planning ────────────────────────────────────
  function addSurveyVertex(lat, lon) {
    const conflict = checkAirspaceConflict(lat, lon);
    if (conflict) {
      showAirspaceAlert(conflict.name);
      log(`Survey vertex REJECTED — restricted airspace: ${conflict.name}`, 'error');
      return;
    }

    state.surveyPolygon.push([lat, lon]);

    // Draw boundary polygon
    updateSurveyBoundary();

    if (state.surveyPolygon.length >= 3) {
      generateSurveyLines();
    }

    log(`Survey vertex ${state.surveyPolygon.length} placed`, 'info');
  }

  function updateSurveyBoundary() {
    if (!state.viewer || state.surveyPolygon.length < 2) return;

    // Remove previous boundary
    if (state._boundaryEntity) state.viewer.entities.remove(state._boundaryEntity);

    const poly = [...state.surveyPolygon, state.surveyPolygon[0]];
    const positions = poly.map(p => Cesium.Cartesian3.fromDegrees(p[1], p[0],
      state.mission.altitude + 2));

    state._boundaryEntity = state.viewer.entities.add({
      polyline: {
        positions,
        width: 1.5,
        material: new Cesium.ColorMaterialProperty(
          Cesium.Color.fromCssColorString('#ff6b1a').withAlpha(0.7)),
        clampToGround: false,
      }
    });
  }

  function clearSurveyLines() {
    if (!state.viewer) return;
    state.entities.surveyLines.forEach(e => state.viewer.entities.remove(e));
    state.entities.surveyLines = [];
    if (state._boundaryEntity) { state.viewer.entities.remove(state._boundaryEntity); state._boundaryEntity = null; }
  }

  function generateSurveyLines() {
    if (!state.viewer || state.surveyPolygon.length < 3) return;
    clearSurveyLines();

    const type = state.missionType;
    const alt = parseInt($('survey-alt')?.value || state.mission.altitude);
    state.mission.altitude = alt;

    if (type === 'grid') {
      generateGridSurvey(alt);
    } else if (type === 'corridor') {
      generateCorridorSurvey(alt);
    } else if (type === 'lidar') {
      generateLidarCorridorSurvey(alt);
    }
  }

  function generateGridSurvey(alt) {
    const poly = state.surveyPolygon;
    const sideOverlap = parseInt($('side-overlap')?.value || 70) / 100;
    const fwdOverlap = parseInt($('fwd-overlap')?.value || 80) / 100;
    const headingDeg = parseInt($('survey-heading')?.value || 0);

    // Bounding box
    const lats = poly.map(p => p[0]), lons = poly.map(p => p[1]);
    const minLat = Math.min(...lats), maxLat = Math.max(...lats);
    const minLon = Math.min(...lons), maxLon = Math.max(...lons);
    const cLat = (minLat + maxLat) / 2;
    const cLon = (minLon + maxLon) / 2;

    // Footprint width calculation
    const { sensorW, sensorH, focalLength } = state.camera;
    const gsd_w = (alt * sensorW) / (focalLength * 1000); // meters per pixel side
    const swathW = gsd_w * 4000; // ~4000px sensor
    const swathH = (alt * sensorH) / (focalLength * 1000) * 3000;

    const lineSpacing = swathW * (1 - sideOverlap) || 30;
    const photoSpacing = swathH * (1 - fwdOverlap) || 20;

    // Width/height of area
    const width = haversine(cLat, minLon, cLat, maxLon);
    const height = haversine(minLat, cLon, maxLat, cLon);

    const numLines = Math.ceil(width / lineSpacing) + 1;
    const numPhotos = Math.ceil(height / photoSpacing);

    state.mission.lines = numLines;
    state.mission.area = (width * height) / 1e6; // km²

    let totalDist = 0;
    const allPositions = [];

    for (let i = 0; i < numLines; i++) {
      const offsetDist = (i - numLines/2) * lineSpacing;
      const startPt = destination(cLat, cLon, headingDeg + 90, offsetDist);
      const s = destination(startPt.lat, startPt.lon, headingDeg + 180, height / 2);
      const e2 = destination(startPt.lat, startPt.lon, headingDeg, height / 2);

      const lineStart = i % 2 === 0 ? s : e2;
      const lineEnd   = i % 2 === 0 ? e2 : s;

      const ent = state.viewer.entities.add({
        polyline: {
          positions: [
            Cesium.Cartesian3.fromDegrees(lineStart.lon, lineStart.lat, alt),
            Cesium.Cartesian3.fromDegrees(lineEnd.lon, lineEnd.lat, alt),
          ],
          width: 1.5,
          material: new Cesium.ColorMaterialProperty(
            Cesium.Color.fromCssColorString('#00d4ff').withAlpha(0.6)),
        }
      });
      state.entities.surveyLines.push(ent);

      totalDist += height;
      if (i < numLines - 1) totalDist += lineSpacing;

      if (i === 0) allPositions.push(lineStart);
      allPositions.push(lineEnd);
      if (i < numLines - 1) {
        const next = destination(cLat, cLon, headingDeg + 90, ((i+1) - numLines/2) * lineSpacing);
        allPositions.push(i % 2 === 0
          ? destination(next.lat, next.lon, headingDeg + 180, height / 2)
          : destination(next.lat, next.lon, headingDeg, height / 2));
      }
    }

    state.mission.distance = totalDist;
    state.mission.flightTime = totalDist / state.mission.speed;

    setStatVal('stat-lines', numLines);
    setStatVal('stat-area', fmt(state.mission.area, 3) + ' km²');
    setStatVal('stat-photos', Math.round(numLines * numPhotos));
    setStatVal('stat-distance', fmtDist(totalDist));
    setStatVal('stat-flight-time', fmtTime(state.mission.flightTime));

    setMS('ms-distance', fmtDist(totalDist));
    setMS('ms-duration', fmtTime(state.mission.flightTime));
    setMS('ms-altitude', alt + ' m');

    log(`Grid survey generated: ${numLines} lines, ${fmtDist(totalDist)}, ~${fmtTime(state.mission.flightTime)}`, 'success');
  }

  function generateCorridorSurvey(alt) {
    if (state.surveyPolygon.length < 2) return;
    const width = parseInt($('corridor-width')?.value || 50) / 2;

    for (let i = 0; i < state.surveyPolygon.length - 1; i++) {
      const a = state.surveyPolygon[i], b = state.surveyPolygon[i+1];
      const brg = bearing(a[0], a[1], b[0], b[1]);

      const l1s = destination(a[0], a[1], brg - 90, width);
      const l1e = destination(b[0], b[1], brg - 90, width);
      const l2s = destination(a[0], a[1], brg + 90, width);
      const l2e = destination(b[0], b[1], brg + 90, width);

      const ent = state.viewer.entities.add({
        polyline: {
          positions: [
            Cesium.Cartesian3.fromDegrees(l1s.lon, l1s.lat, alt),
            Cesium.Cartesian3.fromDegrees(l1e.lon, l1e.lat, alt),
            Cesium.Cartesian3.fromDegrees(l2e.lon, l2e.lat, alt),
            Cesium.Cartesian3.fromDegrees(l2s.lon, l2s.lat, alt),
            Cesium.Cartesian3.fromDegrees(l1s.lon, l1s.lat, alt),
          ],
          width: 1.5,
          material: new Cesium.ColorMaterialProperty(
            Cesium.Color.fromCssColorString('#00ff9d').withAlpha(0.6)),
        }
      });
      state.entities.surveyLines.push(ent);
    }

    let dist = 0;
    for (let i = 1; i < state.surveyPolygon.length; i++) {
      const a = state.surveyPolygon[i-1], b = state.surveyPolygon[i];
      dist += haversine(a[0], a[1], b[0], b[1]);
    }

    setStatVal('stat-distance', fmtDist(dist));
    setStatVal('stat-flight-time', fmtTime(dist / state.mission.speed));
    log(`Corridor survey: ${fmtDist(dist)}, width ${width*2}m`, 'success');
  }

  function generateLidarCorridorSurvey(alt) {
    if (state.surveyPolygon.length < 2) return;
    const scanW = parseInt($('lidar-scan-width')?.value || 80) / 2;
    const passes = parseInt($('lidar-passes')?.value || 3);

    for (let pass = 0; pass < passes; pass++) {
      const offset = (pass - Math.floor(passes/2)) * (scanW * 2 / passes);

      for (let i = 0; i < state.surveyPolygon.length - 1; i++) {
        const a = state.surveyPolygon[i], b = state.surveyPolygon[i+1];
        const brg = bearing(a[0], a[1], b[0], b[1]);
        const as = destination(a[0], a[1], brg + 90, offset);
        const ae = destination(b[0], b[1], brg + 90, offset);

        const ent = state.viewer.entities.add({
          polyline: {
            positions: [
              Cesium.Cartesian3.fromDegrees(as.lon, as.lat, alt),
              Cesium.Cartesian3.fromDegrees(ae.lon, ae.lat, alt),
            ],
            width: 2,
            material: new Cesium.ColorMaterialProperty(
              Cesium.Color.fromCssColorString('#ff6b1a').withAlpha(0.7)),
          }
        });
        state.entities.surveyLines.push(ent);
      }
    }

    log(`LiDAR corridor: ${passes} passes, scan width ${scanW*2}m`, 'success');
  }

  // ── Terrain Following ──────────────────────────────────
  async function applyTerrainFollowing() {
    if (!state.viewer || state.waypoints.length === 0) return;
    log('Querying terrain elevation for all waypoints...', 'info');

    const positions = state.waypoints.map(wp =>
      Cesium.Cartographic.fromDegrees(wp.lon, wp.lat)
    );

    try {
      const updated = await Cesium.sampleTerrainMostDetailed(state.viewer.terrainProvider, positions);
      const agl = parseInt($('terrain-agl')?.value || 50);

      updated.forEach((pos, i) => {
        const groundElev = Math.round(pos.height || 0);
        state.waypoints[i].groundElev = groundElev;
        state.waypoints[i].alt = groundElev + agl;
        state.waypoints[i].agl = agl;

        // Update entity position
        const ent = state.entities.waypoints[i];
        if (ent && ent.point) {
          ent.point.position = new Cesium.ConstantPositionProperty(
            Cesium.Cartesian3.fromDegrees(state.waypoints[i].lon, state.waypoints[i].lat, state.waypoints[i].alt)
          );
        }
        if (ent && ent.stem) {
          ent.stem.polyline.positions = new Cesium.ConstantProperty([
            Cesium.Cartesian3.fromDegrees(state.waypoints[i].lon, state.waypoints[i].lat, 0),
            Cesium.Cartesian3.fromDegrees(state.waypoints[i].lon, state.waypoints[i].lat, state.waypoints[i].alt),
          ]);
        }
      });

      updateFlightPath();
      updateWaypointList();
      updateMissionStats();
      log(`Terrain following applied: AGL ${agl}m over ${state.waypoints.length} waypoints`, 'success');
    } catch (err) {
      log(`Terrain query error: ${err.message}`, 'error');
    }
  }

  // ── Camera Footprint ───────────────────────────────────
  function updateFootprintPreview(lat, lon) {
    if (!state.viewer) return;
    const alt = state.mission.altitude;
    const { sensorW, sensorH, focalLength } = state.camera;

    const footW = (alt * sensorW) / focalLength;
    const footH = (alt * sensorH) / focalLength;

    const corners = [
      destination(lat, lon, 315, Math.sqrt(footW**2 + footH**2) / 2),
      destination(lat, lon, 45, Math.sqrt(footW**2 + footH**2) / 2),
      destination(lat, lon, 135, Math.sqrt(footW**2 + footH**2) / 2),
      destination(lat, lon, 225, Math.sqrt(footW**2 + footH**2) / 2),
    ];

    if (state.entities.footprint) state.viewer.entities.remove(state.entities.footprint);

    state.entities.footprint = state.viewer.entities.add({
      polygon: {
        hierarchy: new Cesium.PolygonHierarchy(
          corners.map(c => Cesium.Cartesian3.fromDegrees(c.lon, c.lat, alt))
        ),
        material: Cesium.Color.fromCssColorString('#ff6b1a').withAlpha(0.15),
        outline: true,
        outlineColor: Cesium.Color.fromCssColorString('#ff6b1a').withAlpha(0.7),
        outlineWidth: 1,
        perPositionHeight: true,
      }
    });

    const fpInfo = $('footprint-info');
    if (fpInfo) {
      fpInfo.className = 'visible';
      fpInfo.innerHTML = `
        <span>FOOTPRINT</span>
        <span class="text-orange">${fmt(footW,1)} × ${fmt(footH,1)} m</span>
        <span class="text-muted">GSD: ${fmt((alt * sensorW) / (focalLength * 100), 1)} cm/px</span>
      `;
    }
  }

  function toggleFootprint() {
    state.footprintVisible = !state.footprintVisible;
    const btn = $('btn-footprint');
    if (btn) btn.classList.toggle('active', state.footprintVisible);

    if (!state.footprintVisible && state.entities.footprint) {
      state.viewer.entities.remove(state.entities.footprint);
      state.entities.footprint = null;
      const fpInfo = $('footprint-info');
      if (fpInfo) fpInfo.className = '';
    }

    log(`Camera footprint ${state.footprintVisible ? 'enabled' : 'disabled'}`, 'info');
  }

  // ── Airspace ───────────────────────────────────────────
  async function loadAirspace() {
    try {
      const res = await fetch('data/no_fly_zones.geojson');
      const geojson = await res.json();
      state.noFlyZones = geojson.features;
      state.airspaceLoaded = true;

      renderNoFlyZones(geojson);
      log(`Airspace data loaded: ${state.noFlyZones.length} restricted zones`, 'success');
    } catch (err) {
      log(`Airspace load warning: ${err.message}`, 'warn');
    }
  }

  function renderNoFlyZones(geojson) {
    if (!state.viewer) return;

    geojson.features.forEach(f => {
      const g = f.geometry;
      const p = f.properties;
      const color = p.restriction === 'PROHIBITED' ? '#ff0033' :
                    p.restriction === 'NO_FLY'    ? '#ff3355' : '#ffaa00';

      if (g.type === 'Point') {
        const radiusM = (p.radius_km || 5) * 1000;
        const ent = state.viewer.entities.add({
          position: Cesium.Cartesian3.fromDegrees(g.coordinates[0], g.coordinates[1], 0),
          ellipse: {
            semiMajorAxis: radiusM,
            semiMinorAxis: radiusM,
            material: Cesium.Color.fromCssColorString(color).withAlpha(0.12),
            outline: true,
            outlineColor: Cesium.Color.fromCssColorString(color).withAlpha(0.8),
            outlineWidth: 2,
            height: 0,
            extrudedHeight: p.altitude_limit || 300,
          },
          label: {
            text: p.name,
            font: '10px "Share Tech Mono"',
            fillColor: Cesium.Color.fromCssColorString(color),
            outlineColor: Cesium.Color.BLACK,
            outlineWidth: 2,
            style: Cesium.LabelStyle.FILL_AND_OUTLINE,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
            scale: 0.8,
          }
        });
        state.entities.noFlyZones.push(ent);
      } else if (g.type === 'Polygon') {
        const ring = g.coordinates[0];
        const positions = ring.map(c => Cesium.Cartesian3.fromDegrees(c[0], c[1], p.altitude_limit || 300));
        const ent = state.viewer.entities.add({
          polygon: {
            hierarchy: new Cesium.PolygonHierarchy(positions),
            material: Cesium.Color.fromCssColorString(color).withAlpha(0.15),
            outline: true,
            outlineColor: Cesium.Color.fromCssColorString(color).withAlpha(0.8),
            outlineWidth: 2,
            perPositionHeight: true,
          }
        });
        state.entities.noFlyZones.push(ent);
      }
    });
  }

  function toggleAirspace(show) {
    state.entities.noFlyZones.forEach(e => { e.show = show; });
    log(`Airspace ${show ? 'shown' : 'hidden'}`, 'info');
  }

  function showAirspaceAlert(zoneName) {
    const el = $('airspace-alert');
    if (!el) return;
    el.querySelector('span').textContent = `RESTRICTED AIRSPACE: ${zoneName}`;
    el.classList.add('visible');
    setTimeout(() => el.classList.remove('visible'), 4000);
  }

  // ── Export ─────────────────────────────────────────────
  function exportGeoJSON() {
    const features = state.waypoints.map((wp, i) => ({
      type: 'Feature',
      properties: { index: i+1, altitude: wp.alt, speed: wp.speed, groundElev: wp.groundElev },
      geometry: { type: 'Point', coordinates: [wp.lon, wp.lat, wp.alt] }
    }));

    if (state.waypoints.length >= 2) {
      features.push({
        type: 'Feature',
        properties: { type: 'flightPath', missionName: state.mission.name },
        geometry: {
          type: 'LineString',
          coordinates: state.waypoints.map(wp => [wp.lon, wp.lat, wp.alt])
        }
      });
    }

    download(JSON.stringify({ type: 'FeatureCollection', features }, null, 2),
      `${state.mission.name}.geojson`, 'application/geo+json');
    log(`Exported GeoJSON: ${state.mission.name}.geojson`, 'success');
  }

  function exportKML() {
    const wps = state.waypoints.map((wp, i) => `
    <Placemark>
      <name>WP${String(i+1).padStart(2,'0')}</name>
      <Point><coordinates>${wp.lon},${wp.lat},${wp.alt}</coordinates></Point>
      <ExtendedData>
        <Data name="speed"><value>${wp.speed}</value></Data>
        <Data name="altitude"><value>${wp.alt}</value></Data>
      </ExtendedData>
    </Placemark>`).join('');

    const coords = state.waypoints.map(wp => `${wp.lon},${wp.lat},${wp.alt}`).join(' ');
    const path = state.waypoints.length >= 2 ? `
    <Placemark>
      <name>Flight Path</name>
      <LineString><altitudeMode>absolute</altitudeMode><coordinates>${coords}</coordinates></LineString>
    </Placemark>` : '';

    const kml = `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>${state.mission.name}</name>
    <description>CHCNAV X500 Mission — Generated by GCS</description>
${wps}${path}
  </Document>
</kml>`;

    download(kml, `${state.mission.name}.kml`, 'application/vnd.google-earth.kml+xml');
    log(`Exported KML: ${state.mission.name}.kml`, 'success');
  }

  function exportJSON() {
    const data = {
      missionName: state.mission.name,
      platform: 'CHCNAV X500',
      generated: new Date().toISOString(),
      mission: state.mission,
      camera: state.camera,
      waypoints: state.waypoints,
      stats: {
        totalDistance: state.mission.distance,
        estimatedFlightTime: state.mission.flightTime,
        waypointCount: state.waypoints.length,
      }
    };
    download(JSON.stringify(data, null, 2), `${state.mission.name}.json`, 'application/json');
    log(`Exported JSON: ${state.mission.name}.json`, 'success');
  }

  function download(content, filename, type) {
    const blob = new Blob([content], { type });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = filename;
    document.body.appendChild(a); a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  // ── Mission Log ────────────────────────────────────────
  function saveMission() {
    if (state.waypoints.length === 0 && state.surveyPolygon.length === 0) {
      log('Nothing to save — plan a mission first', 'warn');
      return;
    }

    const record = {
      id: Date.now(),
      name: state.mission.name,
      type: state.missionType,
      altitude: state.mission.altitude,
      distance: fmtDist(state.mission.distance),
      flightTime: fmtTime(state.mission.flightTime),
      waypoints: state.waypoints.length,
      date: new Date().toLocaleDateString(),
      time: new Date().toLocaleTimeString(),
    };

    state.savedMissions.unshift(record);
    if (state.savedMissions.length > 50) state.savedMissions.pop();

    try {
      localStorage.setItem('gcs_missions', JSON.stringify(state.savedMissions));
    } catch {}

    renderMissionLog();
    log(`Mission "${record.name}" saved`, 'success');
  }

  function loadSavedMissions() {
    try {
      const data = localStorage.getItem('gcs_missions');
      if (data) {
        state.savedMissions = JSON.parse(data);
        renderMissionLog();
        log(`${state.savedMissions.length} saved missions loaded`, 'info');
      }
    } catch {}
  }

  function renderMissionLog() {
    const tbody = $('mission-log-body');
    if (!tbody) return;

    if (state.savedMissions.length === 0) {
      tbody.innerHTML = `<tr><td colspan="7" style="text-align:center;color:var(--text-muted);padding:16px">No saved missions</td></tr>`;
      return;
    }

    tbody.innerHTML = state.savedMissions.map(m => `
      <tr>
        <td>${m.name}</td>
        <td style="text-transform:uppercase;letter-spacing:1px">${m.type}</td>
        <td>${m.altitude} m</td>
        <td>${m.distance}</td>
        <td>${m.flightTime}</td>
        <td>${m.waypoints}</td>
        <td>${m.date}</td>
      </tr>`).join('');
  }

  // ── Mode Switching ─────────────────────────────────────
  function setMode(mode) {
    state.mode = mode;
    const labels = { IDLE: 'IDLE', WAYPOINT: 'WAYPOINT EDIT', SURVEY: 'SURVEY PLAN', TERRAIN: 'TERRAIN FOLLOW' };
    const modeEl = $('mode-label');
    if (modeEl) modeEl.textContent = labels[mode] || mode;

    $$('.tb-btn').forEach(b => b.classList.remove('active'));
    const modeBtn = $(`btn-mode-${mode.toLowerCase()}`);
    if (modeBtn) modeBtn.classList.add('active');

    if (mode === 'WAYPOINT') log('Waypoint mode: click map to place waypoints', 'info');
    if (mode === 'SURVEY') log('Survey mode: click map to define survey area boundary', 'info');
  }

  // ── Camera Controls ────────────────────────────────────
  function flyHome() {
    if (!state.viewer) return;
    state.viewer.camera.flyTo({
      destination: Cesium.Cartesian3.fromDegrees(104.0668, 30.5728, 8000),
      orientation: { heading: 0, pitch: Cesium.Math.toRadians(-45), roll: 0 },
      duration: 2,
    });
  }

  function setView(type) {
    if (!state.viewer) return;
    const cam = state.viewer.camera;
    if (type === '2d') {
      cam.flyTo({ destination: Cesium.Cartesian3.fromDegrees(104.0668, 30.5728, 15000),
        orientation: { heading: 0, pitch: Cesium.Math.toRadians(-90), roll: 0 }, duration: 1.5 });
    } else if (type === '3d') {
      cam.flyTo({ destination: Cesium.Cartesian3.fromDegrees(104.0668, 30.5728, 8000),
        orientation: { heading: 0, pitch: Cesium.Math.toRadians(-45), roll: 0 }, duration: 1.5 });
    }
  }

  function flyToWaypoints() {
    if (!state.viewer || state.waypoints.length === 0) return;
    state.viewer.flyTo(state.entities.waypoints.map(e => e.point));
  }

  // ── Tabs ───────────────────────────────────────────────
  function switchTab(group, tabId) {
    $$(`[data-tab-group="${group}"]`).forEach(el => el.classList.remove('active'));
    $$(`[data-tab-content-group="${group}"]`).forEach(el => el.classList.remove('active'));
    const btn = document.querySelector(`[data-tab-group="${group}"][data-tab="${tabId}"]`);
    const cnt = document.querySelector(`[data-tab-content-group="${group}"][data-tab-content="${tabId}"]`);
    if (btn) btn.classList.add('active');
    if (cnt) cnt.classList.add('active');
  }

  function switchConsoleTab(tabId) {
    $$('.console-tab').forEach(el => el.classList.remove('active'));
    $$('.console-pane').forEach(el => el.classList.remove('active'));
    const btn = document.querySelector(`.console-tab[data-tab="${tabId}"]`);
    const pane = $(`console-${tabId}-pane`);
    if (btn) btn.classList.add('active');
    if (pane) pane.classList.add('active');
  }

  // ── UI Event Binding ───────────────────────────────────
  function bindEvents() {
    // Mode buttons
    $('btn-mode-waypoint')?.addEventListener('click', () => setMode('WAYPOINT'));
    $('btn-mode-survey')?.addEventListener('click', () => setMode('SURVEY'));
    $('btn-footprint')?.addEventListener('click', toggleFootprint);
    $('btn-clear')?.addEventListener('click', clearAllWaypoints);
    $('btn-save')?.addEventListener('click', saveMission);
    $('btn-fly-home')?.addEventListener('click', flyHome);
    $('btn-view-2d')?.addEventListener('click', () => setView('2d'));
    $('btn-view-3d')?.addEventListener('click', () => setView('3d'));
    $('btn-fly-wps')?.addEventListener('click', flyToWaypoints);

    // Export menu
    $('btn-export')?.addEventListener('click', () => {
      const menu = $('export-menu');
      if (menu) menu.classList.toggle('visible');
    });

    $('export-geojson')?.addEventListener('click', () => { exportGeoJSON(); $('export-menu')?.classList.remove('visible'); });
    $('export-kml')?.addEventListener('click', () => { exportKML(); $('export-menu')?.classList.remove('visible'); });
    $('export-json')?.addEventListener('click', () => { exportJSON(); $('export-menu')?.classList.remove('visible'); });

    document.addEventListener('click', (e) => {
      if (!e.target.closest('#btn-export') && !e.target.closest('#export-menu')) {
        $('export-menu')?.classList.remove('visible');
      }
    });

    // Airspace toggle
    $('toggle-airspace')?.addEventListener('change', (e) => toggleAirspace(e.target.checked));

    // Terrain following
    $('toggle-terrain')?.addEventListener('change', (e) => {
      state.terrainFollowing = e.target.checked;
      log(`Terrain following ${state.terrainFollowing ? 'enabled' : 'disabled'}`, 'info');
    });

    $('btn-apply-terrain')?.addEventListener('click', applyTerrainFollowing);

    // Survey type buttons
    $$('.mission-type-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        $$('.mission-type-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        state.missionType = btn.dataset.type;
        showSurveyPanel(state.missionType);
        if (state.surveyPolygon.length >= 3) generateSurveyLines();
        log(`Survey type: ${state.missionType.toUpperCase()}`, 'info');
      });
    });

    // Range inputs live update
    $$('input[type=range]').forEach(input => {
      const valEl = document.querySelector(`[data-range-val="${input.id}"]`);
      input.addEventListener('input', () => {
        if (valEl) valEl.textContent = input.value + (input.dataset.unit || '');
        const key = input.dataset.mission;
        if (key) {
          const keys = key.split('.');
          if (keys.length === 2) state.mission[keys[0]][keys[1]] = +input.value;
          else state.mission[key] = +input.value;
        }
        if (state.surveyPolygon.length >= 3) generateSurveyLines();
      });
    });

    // Console tabs
    $$('.console-tab').forEach(tab => {
      tab.addEventListener('click', () => switchConsoleTab(tab.dataset.tab));
    });

    // Left panel tabs
    $$('[data-tab]').forEach(btn => {
      btn.addEventListener('click', () => {
        const group = btn.dataset.tabGroup;
        if (group) switchTab(group, btn.dataset.tab);
      });
    });

    // Mission name input
    $('mission-name-input')?.addEventListener('change', (e) => {
      state.mission.name = e.target.value.trim().replace(/\s+/g, '_').toUpperCase() || 'MISSION_001';
    });

    // Generate survey button
    $('btn-generate-survey')?.addEventListener('click', () => {
      if (state.surveyPolygon.length < 3) {
        log('Define survey area first (click map to place boundary points)', 'warn');
        return;
      }
      generateSurveyLines();
    });

    // Clear survey area
    $('btn-clear-survey')?.addEventListener('click', () => {
      clearSurveyLines();
      state.surveyPolygon = [];
      log('Survey area cleared', 'warn');
    });

    // Close airspace alert on click
    $('airspace-alert')?.addEventListener('click', () => {
      $('airspace-alert')?.classList.remove('visible');
    });

    // Clear log
    $('btn-clear-log')?.addEventListener('click', () => { state.logs = []; renderLogs(); });

    // Camera params
    ['focal-length', 'sensor-w', 'sensor-h'].forEach(id => {
      $(id)?.addEventListener('input', (e) => {
        if (id === 'focal-length') state.camera.focalLength = +e.target.value;
        if (id === 'sensor-w') state.camera.sensorW = +e.target.value;
        if (id === 'sensor-h') state.camera.sensorH = +e.target.value;
      });
    });
  }

  function showSurveyPanel(type) {
    $$('.survey-subpanel').forEach(p => p.classList.add('hidden'));
    $(`survey-panel-${type}`)?.classList.remove('hidden');
  }

  // ── Loading Screen ─────────────────────────────────────
  async function runLoadingSequence() {
    const messages = [
      'Initializing GCS v2.5.0...',
      'Loading CesiumJS 3D engine...',
      'Connecting terrain server...',
      'Loading CHCNAV X500 profiles...',
      'Fetching airspace database...',
      'Calibrating mission planner...',
      'GCS ready.',
    ];

    const statusEl = $('loading-status');
    for (const msg of messages) {
      if (statusEl) statusEl.textContent = msg;
      await new Promise(r => setTimeout(r, 360));
    }

    await new Promise(r => setTimeout(r, 400));
    const screen = $('loading-screen');
    if (screen) {
      screen.style.opacity = '0';
      screen.style.transition = 'opacity 0.6s ease';
      setTimeout(() => { screen.style.display = 'none'; }, 600);
    }
  }

  // ── Main Init ──────────────────────────────────────────
  async function init() {
    startClock();
    loadSavedMissions();
    bindEvents();

    // Init default survey panel
    showSurveyPanel('grid');

    // Boot log
    log('CHCNAV X500 Ground Control Station v2.5.0', 'system');
    log('Platform: Web Browser — CesiumJS 3D Engine', 'system');
    log('Operator: READY', 'info');
    log('RTK: FIXED | GPS: 18 SAT | Battery: 87%', 'info');

    // Start Cesium and loading simultaneously
    const [cesiumOk] = await Promise.all([
      initCesium(),
      runLoadingSequence(),
    ]);

    if (cesiumOk) {
      setMode('IDLE');
      startTelemetry();
      log('All systems nominal — GCS operational', 'success');
    }

    // Update waypoint list placeholder
    updateWaypointList();
    updateMissionStats();
  }

  // Public API
  return {
    init,
    selectWP: (i) => { state.selectedWP = i; updateWaypointList(); },
    removeWP: removeWaypoint,
    setMode,
    exportGeoJSON,
    exportKML,
    exportJSON,
  };

})();

window.GCS = GCS;
document.addEventListener('DOMContentLoaded', () => GCS.init());
