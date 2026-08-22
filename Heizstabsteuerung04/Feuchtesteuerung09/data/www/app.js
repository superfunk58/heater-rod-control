// =============================================
// Feuchtesteuerung - Web UI Application
// =============================================

var statusInterval = null;
var historyInterval = null;
var statusRetryCount = 0;
var lastHistoryPoints = [];
var lastHistoryEpoch = 0;
var chartMeta = null;
var chart2Meta = null;
var schedEnabled = false;
var cyclicEnabled = false;
var absaugenCyclicEnabled = false;
var configData = null;

// ---- Tab Navigation ----
function showTab(name) {
  document.querySelectorAll('.tab').forEach(function(el) { el.classList.remove('active'); });
  document.querySelectorAll('.tab-content').forEach(function(el) { el.classList.remove('active'); });
  var tab = document.querySelector('.tab[data-tab="' + name + '"]');
  if (tab) tab.classList.add('active');
  var content = document.getElementById('tab-' + name);
  if (content) content.classList.add('active');
  if (name === 'wifi') {
    loadWifiInfo();
  }
  if (name === 'sensors') {
    loadSensorInfo();
  }
  if (name === 'firmware') {
    updateFirmwareInfo();
  }
  if (name === 'verlauf') {
    loadHistory();
  }
  if (name === 'settings') {
    loadConfig();
    var dur = localStorage.getItem('manualDuration');
    if (dur) {
      var el = document.getElementById('manualDurationSetting');
      if (el) el.value = dur;
    }
  }
}

// ---- API Helpers ----
function postForm(url, params) {
  return fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: params.toString()
  });
}

// ---- Fan/Heater/Auto Controls ----
function toggleFan(state) {
  var params = new URLSearchParams();
  if (state === 'off') {
    params.append('manual_secs', '0');
  } else {
    var dur = localStorage.getItem('manualDuration') || 300;
    params.append('manual_fan', 'on');
    params.append('manual_secs', dur);
  }
  postForm('/manual', params).then(function(res) { if (res.ok) loadStatus(); }).catch(function(e) { console.error(e); });
}

function toggleHeater(state) {
  var params = new URLSearchParams();
  if (state === 'off') {
    params.append('manual_secs', '0');
  } else {
    var dur = localStorage.getItem('manualDuration') || 300;
    params.append('manual_heater', 'on');
    params.append('manual_secs', dur);
  }
  postForm('/manual', params).then(function(res) { if (res.ok) loadStatus(); }).catch(function(e) { console.error(e); });
}

function toggleFanDirection() {
  var dirBtn = document.getElementById('dirBtn');
  if (!dirBtn) return;
  var isAbsaugen = dirBtn.textContent.indexOf('ABSAUGEN') >= 0;
  var newDirection = isAbsaugen ? 'false' : 'true';
  var params = new URLSearchParams();
  params.append('fan_direction', newDirection);
  postForm('/fan_direction', params).then(function(res) { if (res.ok) loadStatus(); }).catch(function(e) { console.error(e); });
}

function toggleAbsaugenCycle() {
  var params = new URLSearchParams();
  params.append('fan_direction', 'true');
  postForm('/fan_direction', params).then(function(res) { if (res.ok) loadStatus(); }).catch(function(e) { console.error(e); });
}

function setAuto() {
  var params = new URLSearchParams();
  params.append('manual_secs', '0');
  postForm('/manual', params).then(function(res) { if (res.ok) loadStatus(); }).catch(function(e) { console.error(e); });
}

// ---- Config ----
function loadConfig() {
  fetch('/config').then(function(res) { return res.json(); }).then(function(cfg) {
    configData = cfg;
    var fields = {
      'cfg_humidity_on': cfg.humidity_on,
      'cfg_humidity_off': cfg.humidity_off,
      'cfg_heater_on_temp_diff': cfg.heater_on_temp_diff,
      'cfg_heater_off_temp_diff': cfg.heater_off_temp_diff,
      'cfg_fan_speed_percent': cfg.fan_speed_percent,
      'cfg_absaugen_duration_secs': cfg.absaugen_duration_secs,
      'cfg_absaugen_speed_percent': cfg.absaugen_speed_percent,
      'cfg_bath_pause_mins': cfg.bath_pause_mins,
      'cfg_heater_min_room_temp': cfg.heater_min_room_temp,
      'cfg_heater_max_temp': cfg.heater_max_temp,
      'cfg_temp_safety_diff': cfg.temp_safety_diff,
      'cfg_temp_safety_hyst': cfg.temp_safety_hyst,
      'cfg_mqtt_base_topic': cfg.mqtt_base_topic,
      'cfg_mqtt_interval_secs': cfg.mqtt_interval_secs,
      'cfg_hum_descent_min_drop': cfg.hum_descent_min_drop,
      'cfg_hum_descent_time_secs': cfg.hum_descent_time_secs,
      'energyHeaterWatts': cfg.heater_watts,
      'energyPriceKwh': cfg.energy_price_kwh
    };
    for (var id in fields) {
      var el = document.getElementById(id);
      if (el && fields[id] !== undefined) el.value = fields[id];
    }
    var hdSel = document.getElementById('cfg_hum_descent_heater_enable');
    if (hdSel) hdSel.value = cfg.hum_descent_heater_enable ? 'true' : 'false';
    var heaterSel = document.getElementById('cfg_heater_enabled');
    if (heaterSel) heaterSel.value = cfg.heater_enabled ? 'true' : 'false';
    var dirSel = document.getElementById('cfg_fan_direction');
    if (dirSel) dirSel.value = cfg.fan_direction ? 'true' : 'false';

    // Firmware info
    var fi = document.getElementById('firmwareInfo');
    if (fi && cfg.firmware_version) {
      fi.innerHTML = '<div><strong>Version:</strong> ' + cfg.firmware_version + '</div>' +
        '<div><strong>Ordner:</strong> ' + cfg.firmware_folder + '</div>' +
        '<div><strong>Kompiliert:</strong> ' + cfg.firmware_build_date + ' ' + cfg.firmware_build_time + '</div>' +
        '<div><strong>Device:</strong> ' + cfg.device_hostname + '</div>';
    }

    // Schedule
    schedEnabled = !!cfg.schedule_enabled;
    var schedBtn = document.getElementById('schedEnableBtn');
    if (schedBtn) {
      schedBtn.textContent = schedEnabled ? 'AKTIV' : 'INAKTIV';
      schedBtn.className = 'toggle-btn' + (schedEnabled ? ' active' : '');
    }
    buildScheduleTable(cfg.schedule || [0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF]);

    // Cyclic
    cyclicEnabled = !!cfg.cyclic_enabled;
    var cyclicBtn = document.getElementById('cyclicEnableBtn');
    if (cyclicBtn) {
      cyclicBtn.textContent = cyclicEnabled ? 'AKTIV' : 'INAKTIV';
      cyclicBtn.className = 'toggle-btn' + (cyclicEnabled ? ' active' : '');
    }
    var cyclicFields = {
      'cyclicInterval': cfg.cyclic_interval_mins,
      'cyclicDuration': cfg.cyclic_duration_mins,
      'cyclicSpeed': cfg.cyclic_speed_percent
    };
    for (var cid in cyclicFields) {
      var cel = document.getElementById(cid);
      if (cel && cyclicFields[cid] !== undefined) cel.value = cyclicFields[cid];
    }
    var chSel = document.getElementById('cyclicHeater');
    if (chSel) chSel.value = cfg.cyclic_heater ? 'true' : 'false';
    var chfSel = document.getElementById('cyclicHeaterForce');
    if (chfSel) chfSel.value = cfg.cyclic_heater_force ? 'true' : 'false';
    var csm = document.getElementById('cyclicStartMin');
    if (csm && cfg.cyclic_start_min !== undefined) csm.value = cfg.cyclic_start_min;

    // Absaugen cyclic
    absaugenCyclicEnabled = !!cfg.absaugen_cyclic_enabled;
    var acBtn = document.getElementById('absaugenCyclicEnableBtn');
    if (acBtn) {
      acBtn.textContent = absaugenCyclicEnabled ? 'AKTIV' : 'INAKTIV';
      acBtn.className = 'toggle-btn' + (absaugenCyclicEnabled ? ' active' : '');
    }
    var acFields = {
      'absaugenCyclicInterval': cfg.absaugen_cyclic_interval_mins,
      'absaugenCyclicDuration': cfg.absaugen_cyclic_duration_mins
    };
    for (var acid in acFields) {
      var acel = document.getElementById(acid);
      if (acel && acFields[acid] !== undefined) acel.value = acFields[acid];
    }
    var acsm = document.getElementById('absaugenCyclicStartMin');
    if (acsm && cfg.absaugen_cyclic_start_min !== undefined) acsm.value = cfg.absaugen_cyclic_start_min;

    // Reset speed button label
    var rsBtn = document.getElementById('resetSpeedBtn');
    if (rsBtn) rsBtn.textContent = 'Standard (' + cfg.fan_speed_percent + '%)';

  }).catch(function(e) { console.error('Config load error:', e); });
}

function saveConfig(e) {
  e.preventDefault();
  var form = document.getElementById('cfgForm');
  var formData = new FormData(form);
  var params = new URLSearchParams(formData);
  postForm('/cfg', params).then(function(res) {
    if (res.ok) {
      var st = document.getElementById('cfgStatus');
      if (st) { st.textContent = '\u2713 Gespeichert'; setTimeout(function() { st.textContent = ''; }, 3000); }
    }
  }).catch(function(e) { console.error(e); });
  return false;
}

// ---- Status Updates ----
function loadStatus() {
  fetch('/status').then(function(res) {
    if (!res.ok) { statusRetryCount++; return null; }
    return res.json();
  }).then(function(s) {
    if (!s) return;
    statusRetryCount = 0;
    updateStatusCard(s);
  }).catch(function(e) {
    statusRetryCount++;
    console.error('Status error:', e);
    if (statusRetryCount > 10) {
      clearInterval(statusInterval);
      console.error('Too many failures, stopped polling. Reload page to restart.');
    }
  });
}

function setLed(id, cls) {
  var d = document.getElementById(id);
  if (!d) return;
  d.className = 'led-dot' + (cls ? ' ' + cls : '');
}

function formatDuration(secs) {
  var h = Math.floor(secs / 3600);
  var m = Math.floor((secs % 3600) / 60);
  var s = secs % 60;
  var parts = [];
  if (h > 0) parts.push(h + ' Std');
  if (m > 0) parts.push(m + ' Min');
  if (s > 0 || parts.length === 0) parts.push(s + ' Sek');
  return parts.join(' ') + ' (' + secs + ' s)';
}

function updateEnergyDisplay(heaterSecs, trackingStartEpoch) {
  var runtimeEl = document.getElementById('heaterRuntimeValue');
  var energyEl = document.getElementById('heaterEnergyValue');
  var costEl = document.getElementById('energyCostValue');
  var periodEl = document.getElementById('trackingPeriodValue');
  
  if (runtimeEl) runtimeEl.textContent = formatDuration(heaterSecs);
  
  var watts = parseFloat(document.getElementById('energyHeaterWatts')?.value) || 2000;
  var priceKwh = parseFloat(document.getElementById('energyPriceKwh')?.value) || 0.35;
  
  var hours = heaterSecs / 3600;
  var kwh = (watts * hours) / 1000;
  var cost = kwh * priceKwh;
  
  if (energyEl) energyEl.textContent = kwh.toFixed(2) + ' kWh';
  if (costEl) costEl.textContent = cost.toFixed(2) + ' €';
  
  // Show tracking period
  if (periodEl && trackingStartEpoch > 0) {
    var startDate = new Date(trackingStartEpoch * 1000);
    var now = new Date();
    var daysDiff = Math.floor((now - startDate) / (1000 * 60 * 60 * 24));
    var startStr = startDate.toLocaleDateString('de-DE', { day: '2-digit', month: '2-digit', year: 'numeric' });
    periodEl.textContent = 'Seit ' + startStr + ' (' + daysDiff + ' Tage)';
  } else if (periodEl) {
    periodEl.textContent = 'Warte auf NTP-Sync...';
  }
}

function updateStatusCard(s) {
  if (!s) return;

  // Update energy display
  if (typeof s.heater_on_total_secs === 'number') {
    updateEnergyDisplay(s.heater_on_total_secs, s.heater_tracking_start_epoch || 0);
  }

  // DateTime
  var dtBar = document.getElementById('datetimeBar');
  if (dtBar) {
    dtBar.innerHTML = s.current_time ? '<span>' + s.current_time + '</span>' : (s.ntp_synced ? '--' : 'NTP sync...');
  }

  // Sensor values
  var vals = {
    tempValue: typeof s.temperature_c === 'number' ? s.temperature_c.toFixed(1) + ' \u00b0C' : '-',
    humValue: typeof s.humidity === 'number' ? s.humidity.toFixed(1) + ' %' : '-',
    tempBeforeFanValue: typeof s.temp_before_fan_c === 'number' ? s.temp_before_fan_c.toFixed(1) + ' \u00b0C' : '-',
    tempAfterHeaterValue: typeof s.temp_after_heater_c === 'number' ? s.temp_after_heater_c.toFixed(1) + ' \u00b0C' : '-'
  };
  for (var vid in vals) {
    var vel = document.getElementById(vid);
    if (vel) vel.textContent = vals[vid];
  }

  // Fan button with remaining time inside
  var fanBtn = document.getElementById('fanBtn');
  if (fanBtn && typeof s.fan_on === 'boolean') {
    fanBtn.style.background = s.fan_on ? 'linear-gradient(135deg,#22c55e,#16a34a)' : 'linear-gradient(135deg,#f97316,#ef4444)';
    var fanLabel = s.fan_on ? 'L\u00fcfter AN' : 'L\u00fcfter AUS';
    if (s.control_mode === 'manual' && typeof s.manual_secs_remaining === 'number' && s.manual_secs_remaining > 0 && s.manual_secs_remaining < 999999) {
      var mins = Math.floor(s.manual_secs_remaining / 60);
      var secs = s.manual_secs_remaining % 60;
      fanLabel += ' \u2022 ' + mins + 'm ' + secs + 's';
    }
    fanBtn.textContent = fanLabel;
  }

  // Heater button
  var heaterBtn = document.getElementById('heaterBtn');
  if (heaterBtn) {
    if (s.fan_direction === true) {
      heaterBtn.disabled = true;
      heaterBtn.style.opacity = '0.5';
      heaterBtn.style.cursor = 'not-allowed';
      heaterBtn.style.background = '#9ca3af';
      heaterBtn.textContent = 'Heizung (nur EINBLASEN)';
    } else {
      heaterBtn.disabled = false;
      heaterBtn.style.opacity = '1';
      heaterBtn.style.cursor = 'pointer';
      heaterBtn.style.background = s.heater_on ? 'linear-gradient(135deg,#ef4444,#dc2626)' : 'linear-gradient(135deg,#3b82f6,#2563eb)';
      heaterBtn.textContent = s.heater_on ? 'Heizung AN' : 'Heizung AUS';
    }
  }

  // Direction button
  var dirBtn = document.getElementById('dirBtn');
  if (dirBtn && typeof s.fan_direction === 'boolean') {
    dirBtn.style.background = s.fan_direction ? 'linear-gradient(135deg,#3b82f6,#2563eb)' : 'linear-gradient(135deg,#f59e0b,#d97706)';
    dirBtn.textContent = s.fan_direction ? '\u2193 ABSAUGEN' : '\u2191 EINBLASEN';
  }

  // Absaugen button with countdown inside
  var absBtn = document.getElementById('absaugenBtn');
  if (absBtn) {
    // Block absaugen when schedule is inactive
    if (s.schedule_enabled && !s.schedule_active) {
      absBtn.textContent = 'Absaugen (Zeitplan inaktiv)';
      absBtn.style.background = '#4b5563';
      absBtn.style.opacity = '0.6';
      absBtn.style.cursor = 'not-allowed';
      absBtn.disabled = true;
    } else if (s.fan_direction === true && s.absaugen_secs_remaining > 0) {
      var am = Math.floor(s.absaugen_secs_remaining / 60);
      var as2 = s.absaugen_secs_remaining % 60;
      absBtn.textContent = 'Absaugen: ' + am + 'm ' + as2 + 's';
      absBtn.style.background = 'linear-gradient(135deg,#3b82f6,#2563eb)';
      absBtn.style.opacity = '1';
      absBtn.style.cursor = 'pointer';
      absBtn.disabled = false;
    } else {
      absBtn.textContent = 'Absaugen-Zyklus';
      absBtn.style.background = 'linear-gradient(135deg,#8b5cf6,#6d28d9)';
      absBtn.style.opacity = '1';
      absBtn.style.cursor = 'pointer';
      absBtn.disabled = false;
    }
  }

  // Fan speed
  var fanSpeedSlider = document.getElementById('fanSpeedSlider');
  var fanSpeedValue = document.getElementById('fanSpeedValue');
  if (fanSpeedSlider && fanSpeedValue && typeof s.fan_speed_percent === 'number') {
    fanSpeedSlider.value = s.fan_speed_percent;
    fanSpeedValue.textContent = s.fan_speed_percent + '%';
  }

  // LEDs - only green or gray
  setLed('ledFan', s.fan_on ? 'on-green' : '');
  setLed('ledHeater', s.heater_on ? 'on-green' : '');
  setLed('ledTempSafety', s.temp_safety_active ? 'on-green' : '');
  setLed('ledSchedule', s.schedule_active ? 'on-green' : '');
  setLed('ledDirChange', s.direction_change_active ? 'on-green' : '');
  setLed('ledManual', s.control_mode === 'manual' ? 'on-green' : '');
  setLed('ledManualTimer', (s.control_mode === 'manual' && typeof s.manual_secs_remaining === 'number' && s.manual_secs_remaining > 0 && s.manual_secs_remaining < 999999) ? 'on-green' : '');
  setLed('ledHeaterEnabled', s.heater_enabled ? 'on-green' : '');
  setLed('ledHumidity', s.humidity_active ? 'on-green' : '');
  setLed('ledDirection', s.fan_direction ? 'on-green' : '');
  setLed('ledCyclic', s.cyclic_enabled ? 'on-green' : '');
  setLed('ledCyclicRun', s.cyclic_fan_on ? 'on-green' : '');
  setLed('ledNtp', s.ntp_synced ? 'on-green' : '');
  setLed('ledAbsaugen', (s.fan_direction && s.absaugen_secs_remaining > 0) ? 'on-green' : '');
  setLed('ledHumDescent', s.hum_descent_heater_forced ? 'on-green' : '');

  // Cyclic countdown
  var cc = document.getElementById('cyclicCountdown');
  var ccm = document.getElementById('cyclicCountdownMain');
  var ccText = '', ccColor = '';
  if (s.cyclic_fan_on && typeof s.cyclic_secs_remaining === 'number') {
    var cm = Math.floor(s.cyclic_secs_remaining / 60);
    var cs = s.cyclic_secs_remaining % 60;
    ccText = 'Zyklus l\u00e4uft: ' + cm + 'm ' + cs + 's verbleibend';
    ccColor = '#38bdf8';
  } else if (s.cyclic_enabled && typeof s.cyclic_next_secs === 'number') {
    var cnm = Math.floor(s.cyclic_next_secs / 60);
    var cns = s.cyclic_next_secs % 60;
    ccText = 'N\u00e4chster Zyklus in: ' + cnm + 'm ' + cns + 's';
    ccColor = '#94a3b8';
  }
  if (cc) { cc.textContent = ccText; cc.style.color = ccColor; }
  if (ccm) {
    ccm.textContent = ccText; ccm.style.color = ccColor;
    if (ccText) { ccm.style.display = 'block'; ccm.className = 'countdown glow'; }
    else { ccm.style.display = 'none'; ccm.className = 'countdown'; }
  }

  // Bath pause button with countdown inside
  var bpBtn = document.getElementById('bathPauseBtn');
  if (bpBtn) {
    if (s.bath_pause_active && typeof s.bath_pause_secs_remaining === 'number' && s.bath_pause_secs_remaining > 0) {
      var bm = Math.floor(s.bath_pause_secs_remaining / 60);
      var bs = s.bath_pause_secs_remaining % 60;
      bpBtn.textContent = 'Badepause: ' + bm + 'm ' + bs + 's';
      bpBtn.style.background = 'linear-gradient(135deg,#ef4444,#dc2626)';
    } else if (s.bath_pause_active) {
      bpBtn.textContent = 'Badepause BEENDEN';
      bpBtn.style.background = 'linear-gradient(135deg,#ef4444,#dc2626)';
    } else {
      bpBtn.textContent = 'Badepause';
      bpBtn.style.background = 'linear-gradient(135deg,#a855f7,#7c3aed)';
    }
  }

  // Auto toggle button - show mode status
  var atBtn = document.getElementById('autoToggleBtn');
  if (atBtn) {
    if (s.auto_enabled) {
      var modeLabel = 'Automatik AN';
      if (s.control_mode === 'manual') modeLabel += ' (Manuell)';
      atBtn.textContent = modeLabel;
      atBtn.style.background = 'linear-gradient(135deg,#22c55e,#16a34a)';
    } else {
      atBtn.textContent = 'Automatik AUS';
      atBtn.style.background = 'linear-gradient(135deg,#f97316,#ef4444)';
    }
  }

  // Absaugen cyclic countdown (in schedule tab only)
  var acc = document.getElementById('absaugenCyclicCountdown');
  var accText = '', accColor = '';
  if (s.absaugen_cyclic_running && typeof s.absaugen_cyclic_secs_remaining === 'number') {
    var acm = Math.floor(s.absaugen_cyclic_secs_remaining / 60);
    var acs = s.absaugen_cyclic_secs_remaining % 60;
    accText = 'Absaugen l\u00e4uft: ' + acm + 'm ' + acs + 's verbleibend';
    accColor = '#ef4444';
  } else if (s.absaugen_cyclic_enabled && typeof s.absaugen_cyclic_next_secs === 'number') {
    var anm = Math.floor(s.absaugen_cyclic_next_secs / 60);
    var ans = s.absaugen_cyclic_next_secs % 60;
    accText = 'N\u00e4chster Absaugen-Zyklus in: ' + anm + 'm ' + ans + 's';
    accColor = '#94a3b8';
  }
  if (acc) { acc.textContent = accText; acc.style.color = accColor; }
}

// ---- History Chart ----
function loadHistory() {
  fetch('/history').then(function(res) {
    if (!res.ok) return null;
    return res.json();
  }).then(function(data) {
    if (data) {
      var pts = data.points || data;
      var epoch = data.now_epoch || 0;
      lastHistoryPoints = pts;
      lastHistoryEpoch = epoch;
      renderBothCharts();
    }
  }).catch(function(e) { console.error('History error:', e); });
}

function filterPointsByMinutes(points, mins) {
  if (!points || !points.length) return points;
  var cutoff = -mins * 60; // dt_sec is negative (seconds ago)
  return points.filter(function(p) { return p.dt_sec >= cutoff; });
}

function renderBothCharts() {
  // Chart 1: long-term
  var timeSel1 = document.getElementById('chart1TimeMins');
  var mins1 = timeSel1 ? parseInt(timeSel1.value) : 240;
  var pts1 = filterPointsByMinutes(lastHistoryPoints, mins1);
  chartMeta = renderHistoryCanvas('humChart', 'chartHeightSlider', pts1, lastHistoryEpoch);

  // Chart 2: short-term
  var timeSel2 = document.getElementById('chart2TimeMins');
  var mins2 = timeSel2 ? parseInt(timeSel2.value) : 30;
  var pts2 = filterPointsByMinutes(lastHistoryPoints, mins2);
  chart2Meta = renderHistoryCanvas('humChart2', 'chart2HeightSlider', pts2, lastHistoryEpoch);
}

function renderHistoryCanvas(canvasId, heightSliderId, points, nowEpoch) {
  var c = document.getElementById(canvasId);
  if (!c) return null;

  // Dynamic height from slider
  var heightSlider = document.getElementById(heightSliderId);
  if (heightSlider) {
    c.style.height = heightSlider.value + 'px';
  }

  var dpr = window.devicePixelRatio || 1;
  var rect = c.getBoundingClientRect();
  var w = rect.width * dpr;
  var h = rect.height ? rect.height * dpr : 350 * dpr;
  c.width = w;
  c.height = h;
  var ctx = c.getContext('2d');
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, w, h);

  if (!points || !points.length) return null;

  var padL = 40, padR = 40, padT = 18, padB = 50;
  var iw = (w / dpr) - padL - padR;
  var ih = (h / dpr) - padT - padB;

  var minH = null, maxH = null, minT = null, maxT = null;
  var minTime = points[0].dt_sec, maxTime = points[0].dt_sec;

  for (var i = 0; i < points.length; i++) {
    var p = points[i];
    if (typeof p.humidity === 'number' && !isNaN(p.humidity)) {
      if (minH === null || p.humidity < minH) minH = p.humidity;
      if (maxH === null || p.humidity > maxH) maxH = p.humidity;
    }
    if (typeof p.temperature === 'number' && !isNaN(p.temperature)) {
      if (minT === null || p.temperature < minT) minT = p.temperature;
      if (maxT === null || p.temperature > maxT) maxT = p.temperature;
    }
    if (p.dt_sec < minTime) minTime = p.dt_sec;
    if (p.dt_sec > maxTime) maxTime = p.dt_sec;
  }

  if (minH === null) { minH = 0; maxH = 100; }
  if (minT === null) { minT = 0; maxT = 30; }
  if (maxH === minH) { maxH += 1; minH -= 1; }
  if (maxT === minT) { maxT += 1; minT -= 1; }
  if (maxTime === minTime) { maxTime += 1; minTime -= 1; }

  var hasHumidity = (minH !== 0 || maxH !== 100);
  var hasTemp = (minT !== 0 || maxT !== 30);

  var meta = { padL: padL, padR: padR, padT: padT, padB: padB, iw: iw, ih: ih, minH: minH, maxH: maxH, minT: minT, maxT: maxT, minTime: minTime, maxTime: maxTime, hasHumidity: hasHumidity, hasTemp: hasTemp, dpr: dpr, w: w / dpr, h: h / dpr, points: points };

  // Background bars for fan direction and heater
  for (var j = 0; j < points.length - 1; j++) {
    var pp = points[j];
    var pn = points[j + 1];
    var x1 = padL + ((pp.dt_sec - minTime) / (maxTime - minTime)) * iw;
    var x2 = padL + ((pn.dt_sec - minTime) / (maxTime - minTime)) * iw;
    if (pp.fan_on) {
      // Einblasen = blau, Absaugen = grün
      ctx.fillStyle = (pp.fan_dir === 'absaugen') ? 'rgba(22,163,74,0.25)' : 'rgba(37,99,235,0.25)';
      ctx.fillRect(x1, padT, x2 - x1, ih);
    }
    if (pp.heater_on) {
      ctx.fillStyle = 'rgba(220,38,38,0.25)';
      ctx.fillRect(x1, padT, x2 - x1, ih);
    }
  }

  // Gridlines
  ctx.strokeStyle = 'rgba(100,116,139,0.25)';
  ctx.lineWidth = 0.5;
  var numGridH = 5;
  for (var gi = 1; gi < numGridH; gi++) {
    var gy = padT + (gi / numGridH) * ih;
    ctx.beginPath();
    ctx.moveTo(padL, gy);
    ctx.lineTo(padL + iw, gy);
    ctx.stroke();
  }
  var numGridV = 6;
  for (var gj = 1; gj < numGridV; gj++) {
    var gx = padL + (gj / numGridV) * iw;
    ctx.beginPath();
    ctx.moveTo(gx, padT);
    ctx.lineTo(gx, padT + ih);
    ctx.stroke();
  }

  // Day boundary markers (only for >24h views)
  var timeSpanSecs = maxTime - minTime;
  var dayNames = ['So','Mo','Di','Mi','Do','Fr','Sa'];
  if (timeSpanSecs > 86400 && nowEpoch) {
    // Find midnight boundaries within the time range
    var startEpoch = nowEpoch + minTime;
    var endEpoch = nowEpoch + maxTime;
    // Find first midnight after startEpoch
    var firstMidnight = new Date(startEpoch * 1000);
    firstMidnight.setHours(0, 0, 0, 0);
    firstMidnight = new Date(firstMidnight.getTime() + 86400000); // next day 00:00
    for (var md = firstMidnight.getTime() / 1000; md < endEpoch; md += 86400) {
      var dtMd = md - nowEpoch;
      var mx = padL + ((dtMd - minTime) / (maxTime - minTime)) * iw;
      if (mx > padL && mx < padL + iw) {
        // Dashed vertical line
        ctx.save();
        ctx.setLineDash([4, 4]);
        ctx.strokeStyle = 'rgba(250,204,21,0.5)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(mx, padT);
        ctx.lineTo(mx, padT + ih);
        ctx.stroke();
        ctx.restore();
        // Day name label at top
        var dayDate = new Date(md * 1000);
        var dayLabel = dayNames[dayDate.getDay()];
        ctx.fillStyle = 'rgba(250,204,21,0.8)';
        ctx.font = 'bold 11px system-ui';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'bottom';
        ctx.fillText(dayLabel, mx, padT - 1);
      }
    }
  }

  // Axes
  ctx.strokeStyle = 'rgba(51,65,85,0.9)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(padL, padT);
  ctx.lineTo(padL, padT + ih);
  ctx.lineTo(padL + iw, padT + ih);
  ctx.lineTo(padL + iw + padR, padT + ih);
  ctx.stroke();

  ctx.font = '11px system-ui';

  // Y-axis labels
  if (hasHumidity) {
    ctx.fillStyle = '#38bdf8';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (var hl = 0; hl <= numGridH; hl++) {
      var hVal = minH + (1 - hl / numGridH) * (maxH - minH);
      var hy2 = padT + (hl / numGridH) * ih;
      ctx.fillText(hVal.toFixed(0) + ' %', padL - 6, hy2);
    }
  }
  if (hasTemp) {
    ctx.fillStyle = '#f97316';
    ctx.textAlign = 'left';
    for (var tl = 0; tl <= numGridH; tl++) {
      var tVal = minT + (1 - tl / numGridH) * (maxT - minT);
      var ty2 = padT + (tl / numGridH) * ih;
      ctx.fillText(tVal.toFixed(0) + ' \u00b0C', (w / dpr) - padR + 6, ty2);
    }
  }

  // X-axis time labels
  ctx.fillStyle = '#9ca3af';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  ctx.font = '10px system-ui';
  var numLabels = Math.min(8, points.length);
  if (numLabels > 1 && nowEpoch) {
    for (var li = 0; li < numLabels; li++) {
      var frac = li / (numLabels - 1);
      var dtAtLabel = minTime + frac * (maxTime - minTime);
      var epochAtLabel = nowEpoch + dtAtLabel;
      var d = new Date(epochAtLabel * 1000);
      var lbl = ('0' + d.getHours()).slice(-2) + ':' + ('0' + d.getMinutes()).slice(-2);
      if (timeSpanSecs > 86400) {
        lbl = dayNames[d.getDay()] + ' ' + lbl;
      }
      var lx2 = padL + frac * iw;
      ctx.fillText(lbl, lx2, padT + ih + 6);
    }
  } else {
    ctx.fillText('Zeit', (w / dpr) / 2, padT + ih + 6);
  }

  // Humidity line
  if (hasHumidity) {
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2;
    ctx.beginPath();
    var started = false;
    for (var hi = 0; hi < points.length; hi++) {
      var hp = points[hi];
      if (typeof hp.humidity === 'number' && !isNaN(hp.humidity)) {
        var hx = padL + ((hp.dt_sec - minTime) / (maxTime - minTime)) * iw;
        var hy = padT + ih - ((hp.humidity - minH) / (maxH - minH)) * ih;
        if (!started) { ctx.moveTo(hx, hy); started = true; }
        else ctx.lineTo(hx, hy);
      }
    }
    ctx.stroke();
  }

  // Temperature line
  if (hasTemp) {
    ctx.strokeStyle = '#f97316';
    ctx.lineWidth = 2;
    ctx.beginPath();
    var tStarted = false;
    for (var ti = 0; ti < points.length; ti++) {
      var tp = points[ti];
      if (typeof tp.temperature === 'number' && !isNaN(tp.temperature)) {
        var tx = padL + ((tp.dt_sec - minTime) / (maxTime - minTime)) * iw;
        var ty = padT + ih - ((tp.temperature - minT) / (maxT - minT)) * ih;
        if (!tStarted) { ctx.moveTo(tx, ty); tStarted = true; }
        else ctx.lineTo(tx, ty);
      }
    }
    ctx.stroke();
  }

  // Legend
  ctx.font = '10px system-ui';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  var legendY = padT + ih + 28;
  var lx = padL;
  if (hasHumidity) {
    ctx.fillStyle = '#38bdf8';
    ctx.fillRect(lx, legendY, 12, 3);
    ctx.fillText('Feuchte', lx + 16, legendY - 2);
    lx += 70;
  }
  if (hasTemp) {
    ctx.fillStyle = '#f97316';
    ctx.fillRect(lx, legendY, 12, 3);
    ctx.fillText('Temperatur', lx + 16, legendY - 2);
    lx += 85;
  }
  ctx.fillStyle = '#1e3a5f';
  ctx.fillRect(lx, legendY - 2, 12, 8);
  ctx.fillStyle = '#38bdf8';
  ctx.fillText('L\u00fcfter', lx + 16, legendY - 2);
  lx += 60;
  ctx.fillStyle = '#5f1e1e';
  ctx.fillRect(lx, legendY - 2, 12, 8);
  ctx.fillStyle = '#ef4444';
  ctx.fillText('Heizung', lx + 16, legendY - 2);

  return meta;
}

// ---- Schedule Table ----
function buildScheduleTable(schedule) {
  var thead = document.querySelector('#schedTable thead tr');
  if (!thead) return;
  thead.innerHTML = '<th style="padding:4px 2px;font-size:11px;color:#9ca3af;"></th>';
  for (var h = 0; h < 24; h++) {
    thead.innerHTML += '<th style="padding:4px 2px;font-size:11px;color:#9ca3af;min-width:22px;text-align:center;">' + h + '</th>';
  }

  var dayNames = ['So', 'Mo', 'Di', 'Mi', 'Do', 'Fr', 'Sa'];
  var body = document.getElementById('schedBody');
  if (!body) return;
  body.innerHTML = '';

  for (var d = 0; d < 7; d++) {
    var tr = document.createElement('tr');
    tr.innerHTML = '<td style="padding:4px 6px;font-size:13px;font-weight:600;color:#cbd5f5;white-space:nowrap;">' + dayNames[d] + '</td>';
    for (var hh = 0; hh < 24; hh++) {
      var active = (schedule[d] & (1 << hh)) !== 0;
      var td = document.createElement('td');
      td.className = 'sched-cell';
      td.dataset.d = d;
      td.dataset.h = hh;
      td.style.cssText = 'padding:2px;text-align:center;cursor:pointer;';
      var div = document.createElement('div');
      div.style.cssText = 'width:20px;height:20px;border-radius:4px;margin:auto;background:' + (active ? '#22c55e' : '#334155') + ';';
      td.appendChild(div);
      td.addEventListener('click', function() {
        var innerDiv = this.querySelector('div');
        if (!innerDiv) return;
        var isOn = innerDiv.style.background === 'rgb(34, 197, 94)';
        innerDiv.style.background = isOn ? '#334155' : '#22c55e';
      });
      tr.appendChild(td);
    }
    body.appendChild(tr);
  }
}

// ---- Setup Event Listeners ----
function setupButtons() {
  var fanBtn = document.getElementById('fanBtn');
  if (fanBtn) {
    fanBtn.addEventListener('click', function() {
      var state = fanBtn.textContent.indexOf('AN') >= 0 ? 'off' : 'on';
      toggleFan(state);
    });
  }

  var heaterBtn = document.getElementById('heaterBtn');
  if (heaterBtn) {
    heaterBtn.addEventListener('click', function() {
      if (heaterBtn.disabled) return;
      var state = heaterBtn.textContent.indexOf('AN') >= 0 ? 'off' : 'on';
      toggleHeater(state);
    });
  }

  var dirBtn = document.getElementById('dirBtn');
  if (dirBtn) dirBtn.addEventListener('click', toggleFanDirection);

  var absBtn = document.getElementById('absaugenBtn');
  if (absBtn) absBtn.addEventListener('click', toggleAbsaugenCycle);

}

function setupFanSpeed() {
  var slider = document.getElementById('fanSpeedSlider');
  var value = document.getElementById('fanSpeedValue');
  if (!slider || !value) return;

  slider.addEventListener('input', function() {
    value.textContent = slider.value + '%';
  });
  slider.addEventListener('change', function() {
    var params = new URLSearchParams();
    params.append('fan_speed', slider.value);
    postForm('/fan_speed', params).catch(function(e) { console.error(e); });
  });

  var resetBtn = document.getElementById('resetSpeedBtn');
  if (resetBtn) {
    resetBtn.addEventListener('click', function() {
      // Fetch config fresh to ensure we have the latest default speed
      fetch('/config').then(function(res) { return res.json(); }).then(function(cfg) {
        configData = cfg;
        var defaultSpeed = cfg.fan_speed_percent || 75;
        slider.value = defaultSpeed;
        value.textContent = defaultSpeed + '%';
        var params = new URLSearchParams();
        params.append('fan_speed', defaultSpeed);
        postForm('/fan_speed', params).then(function() { loadStatus(); }).catch(function(e) { console.error(e); });
        // Update button label
        resetBtn.textContent = 'Standard (' + defaultSpeed + '%)';
      }).catch(function(e) { console.error(e); });
    });
  }
}

function setupSchedule() {
  var enBtn = document.getElementById('schedEnableBtn');
  if (enBtn) {
    enBtn.addEventListener('click', function() {
      schedEnabled = !schedEnabled;
      this.textContent = schedEnabled ? 'AKTIV' : 'INAKTIV';
      this.className = 'toggle-btn' + (schedEnabled ? ' active' : '');
    });
  }

  var saveBtn = document.getElementById('schedSaveBtn');
  if (saveBtn) {
    saveBtn.addEventListener('click', function() {
      var sched = [0, 0, 0, 0, 0, 0, 0];
      document.querySelectorAll('.sched-cell').forEach(function(cell) {
        var d = parseInt(cell.dataset.d);
        var h = parseInt(cell.dataset.h);
        var div = cell.querySelector('div');
        if (div && div.style.background === 'rgb(34, 197, 94)') {
          sched[d] |= (1 << h);
        }
      });
      var params = new URLSearchParams();
      params.append('schedule_enabled', schedEnabled ? 'true' : 'false');
      for (var d = 0; d < 7; d++) params.append('schedule_' + d, sched[d].toString());
      postForm('/schedule', params).then(function(res) {
        if (res.ok) {
          var st = document.getElementById('schedStatus');
          if (st) { st.textContent = '\u2713 Gespeichert'; setTimeout(function() { st.textContent = ''; }, 3000); }
        }
      }).catch(function(e) { console.error(e); });
    });
  }

  var allOnBtn = document.getElementById('schedAllOnBtn');
  if (allOnBtn) {
    allOnBtn.addEventListener('click', function() {
      document.querySelectorAll('.sched-cell div').forEach(function(d) { d.style.background = '#22c55e'; });
    });
  }

  var allOffBtn = document.getElementById('schedAllOffBtn');
  if (allOffBtn) {
    allOffBtn.addEventListener('click', function() {
      document.querySelectorAll('.sched-cell div').forEach(function(d) { d.style.background = '#334155'; });
    });
  }
}

function setupAbsaugenCyclic() {
  var enBtn = document.getElementById('absaugenCyclicEnableBtn');
  if (enBtn) {
    enBtn.addEventListener('click', function() {
      absaugenCyclicEnabled = !absaugenCyclicEnabled;
      this.textContent = absaugenCyclicEnabled ? 'AKTIV' : 'INAKTIV';
      this.className = 'toggle-btn' + (absaugenCyclicEnabled ? ' active' : '');
    });
  }

  var saveBtn = document.getElementById('absaugenCyclicSaveBtn');
  if (saveBtn) {
    saveBtn.addEventListener('click', function() {
      var params = new URLSearchParams();
      params.append('absaugen_cyclic_enabled', absaugenCyclicEnabled ? 'true' : 'false');
      params.append('absaugen_cyclic_interval_mins', document.getElementById('absaugenCyclicInterval').value);
      params.append('absaugen_cyclic_duration_mins', document.getElementById('absaugenCyclicDuration').value);
      params.append('absaugen_cyclic_start_min', document.getElementById('absaugenCyclicStartMin').value);
      postForm('/cfg', params).then(function(res) {
        if (res.ok) {
          var st = document.getElementById('absaugenCyclicStatus');
          if (st) { st.textContent = '\u2713 Gespeichert'; setTimeout(function() { st.textContent = ''; }, 3000); }
        }
      }).catch(function(e) { console.error(e); });
    });
  }

  var manualBtn = document.getElementById('absaugenManualBtn');
  if (manualBtn) {
    manualBtn.addEventListener('click', function() {
      var params = new URLSearchParams();
      params.append('fan_direction', 'true');
      postForm('/fan_direction', params).then(function(res) { if (res.ok) loadStatus(); }).catch(function(e) { console.error(e); });
    });
  }
}

function setupCyclic() {
  var enBtn = document.getElementById('cyclicEnableBtn');
  if (enBtn) {
    enBtn.addEventListener('click', function() {
      cyclicEnabled = !cyclicEnabled;
      this.textContent = cyclicEnabled ? 'AKTIV' : 'INAKTIV';
      this.className = 'toggle-btn' + (cyclicEnabled ? ' active' : '');
    });
  }

  var saveBtn = document.getElementById('cyclicSaveBtn');
  if (saveBtn) {
    saveBtn.addEventListener('click', function() {
      var params = new URLSearchParams();
      params.append('cyclic_enabled', cyclicEnabled ? 'true' : 'false');
      params.append('cyclic_interval_mins', document.getElementById('cyclicInterval').value);
      params.append('cyclic_duration_mins', document.getElementById('cyclicDuration').value);
      params.append('cyclic_heater', document.getElementById('cyclicHeater').value);
      params.append('cyclic_heater_force', document.getElementById('cyclicHeaterForce').value);
      params.append('cyclic_speed_percent', document.getElementById('cyclicSpeed').value);
      params.append('cyclic_start_min', document.getElementById('cyclicStartMin').value);
      postForm('/cyclic', params).then(function(res) {
        if (res.ok) {
          var st = document.getElementById('cyclicStatus');
          if (st) { st.textContent = '\u2713 Gespeichert'; setTimeout(function() { st.textContent = ''; }, 3000); }
        }
      }).catch(function(e) { console.error(e); });
    });
  }
}

// ---- WiFi Manager ----
var wifiScanTimer = null;

function rssiToPercent(rssi) {
  if (rssi >= -50) return 100;
  if (rssi <= -100) return 0;
  return Math.round(2 * (rssi + 100));
}

function rssiToColor(pct) {
  if (pct >= 60) return '#22c55e';
  if (pct >= 30) return '#f59e0b';
  return '#ef4444';
}

function rssiToLabel(rssi) {
  if (rssi >= -50) return 'Sehr gut';
  if (rssi >= -60) return 'Gut';
  if (rssi >= -70) return 'Ok';
  if (rssi >= -80) return 'Schwach';
  return 'Sehr schwach';
}

function loadWifiInfo() {
  fetch('/wifi/info').then(function(res) { return res.json(); }).then(function(w) {
    // Network type info
    var ntEl = document.getElementById('networkTypeInfo');
    if (ntEl) {
      var netType = w.network_type || 'wifi';
      if (netType === 'ethernet') {
        ntEl.innerHTML = '<span style="color:#22c55e;">\u2713 Ethernet (W5500)</span> \u2014 LAN-Verbindung aktiv';
      } else {
        ntEl.innerHTML = '<span style="color:#38bdf8;">\u2713 WLAN</span> \u2014 WiFi-Verbindung aktiv';
      }
    }
    var el = document.getElementById('wifiInfo');
    if (el) {
      var rows = '';
      rows += '<div><strong>Verbindungstyp:</strong> ' + ((w.network_type === 'ethernet') ? 'Ethernet (W5500)' : 'WLAN') + '</div>';
      rows += '<div><strong>SSID:</strong> ' + (w.ssid || '--') + '</div>';
      rows += '<div><strong>Status:</strong> ' + (w.connected ? '<span style="color:#22c55e">Verbunden</span>' : '<span style="color:#ef4444">Nicht verbunden</span>') + '</div>';
      rows += '<div><strong>IP-Adresse:</strong> ' + (w.ip || '--') + '</div>';
      rows += '<div><strong>Gateway:</strong> ' + (w.gateway || '--') + '</div>';
      rows += '<div><strong>Subnetz:</strong> ' + (w.subnet || '--') + '</div>';
      rows += '<div><strong>DNS:</strong> ' + (w.dns || '--') + '</div>';
      rows += '<div><strong>MAC:</strong> ' + (w.mac || '--') + '</div>';
      rows += '<div><strong>Kanal:</strong> ' + (w.channel || '--') + '</div>';
      rows += '<div><strong>Hostname:</strong> ' + (w.hostname || '--') + '.local</div>';
      rows += '<div><strong>Konfig SSID:</strong> ' + (w.cfg_ssid || '--') + '</div>';
      el.innerHTML = rows;
    }
    var pct = rssiToPercent(w.rssi || -100);
    var fill = document.getElementById('wifiRssiFill');
    var txt = document.getElementById('wifiRssiText');
    if (fill) {
      fill.style.width = pct + '%';
      fill.style.background = rssiToColor(pct);
    }
    if (txt) txt.textContent = (w.rssi || '--') + ' dBm (' + rssiToLabel(w.rssi || -100) + ')';
  }).catch(function(e) { console.error('WiFi info error:', e); });
}

function startWifiScan() {
  var statusEl = document.getElementById('wifiScanStatus');
  var listEl = document.getElementById('wifiNetworkList');
  var scanBtn = document.getElementById('wifiScanBtn');
  if (statusEl) statusEl.textContent = 'Suche l\u00e4uft...';
  if (scanBtn) { scanBtn.disabled = true; scanBtn.style.opacity = '0.5'; }
  if (listEl) listEl.innerHTML = '';

  fetch('/wifi/scan').then(function(res) { return res.json(); }).then(function(data) {
    if (data.scanning) {
      if (wifiScanTimer) clearInterval(wifiScanTimer);
      wifiScanTimer = setInterval(pollWifiScan, 2000);
    } else {
      renderWifiNetworks(data.networks || []);
      if (scanBtn) { scanBtn.disabled = false; scanBtn.style.opacity = '1'; }
      if (statusEl) statusEl.textContent = '';
    }
  }).catch(function(e) {
    console.error('WiFi scan error:', e);
    if (scanBtn) { scanBtn.disabled = false; scanBtn.style.opacity = '1'; }
    if (statusEl) statusEl.textContent = 'Fehler beim Scannen';
  });
}

function pollWifiScan() {
  var statusEl = document.getElementById('wifiScanStatus');
  var scanBtn = document.getElementById('wifiScanBtn');
  fetch('/wifi/scan').then(function(res) { return res.json(); }).then(function(data) {
    if (!data.scanning) {
      if (wifiScanTimer) { clearInterval(wifiScanTimer); wifiScanTimer = null; }
      renderWifiNetworks(data.networks || []);
      if (scanBtn) { scanBtn.disabled = false; scanBtn.style.opacity = '1'; }
      if (statusEl) statusEl.textContent = '';
    }
  }).catch(function(e) {
    if (wifiScanTimer) { clearInterval(wifiScanTimer); wifiScanTimer = null; }
    if (scanBtn) { scanBtn.disabled = false; scanBtn.style.opacity = '1'; }
    if (statusEl) statusEl.textContent = 'Fehler';
  });
}

function renderWifiNetworks(networks) {
  var listEl = document.getElementById('wifiNetworkList');
  if (!listEl) return;
  if (!networks.length) {
    listEl.innerHTML = '<div style="color:#64748b;font-size:13px;padding:8px;">Keine Netzwerke gefunden.</div>';
    return;
  }
  // Sort by RSSI descending
  networks.sort(function(a, b) { return b.rssi - a.rssi; });
  // Remove duplicates
  var seen = {};
  var unique = [];
  for (var i = 0; i < networks.length; i++) {
    if (!networks[i].ssid || seen[networks[i].ssid]) continue;
    seen[networks[i].ssid] = true;
    unique.push(networks[i]);
  }
  listEl.innerHTML = '';
  for (var j = 0; j < unique.length; j++) {
    var n = unique[j];
    var pct = rssiToPercent(n.rssi);
    var color = rssiToColor(pct);
    var lockIcon = n.encryption === 'open' ? '' : ' &#128274;';
    var div = document.createElement('div');
    div.style.cssText = 'display:flex;align-items:center;gap:10px;padding:10px 12px;background:rgba(30,41,59,0.7);border-radius:8px;cursor:pointer;transition:background .2s;';
    div.innerHTML = '<div style="flex:1;"><div style="font-size:14px;font-weight:500;color:#e2e8f0;">' + n.ssid + lockIcon + '</div>' +
      '<div style="font-size:11px;color:#64748b;margin-top:2px;">Kanal ' + n.channel + ' \u00b7 ' + n.rssi + ' dBm \u00b7 ' + rssiToLabel(n.rssi) + '</div></div>' +
      '<div style="width:50px;height:6px;background:#1e293b;border-radius:3px;overflow:hidden;"><div style="height:100%;width:' + pct + '%;background:' + color + ';border-radius:3px;"></div></div>';
    div.dataset.ssid = n.ssid;
    div.addEventListener('mouseenter', function() { this.style.background = 'rgba(51,65,85,0.8)'; });
    div.addEventListener('mouseleave', function() { this.style.background = 'rgba(30,41,59,0.7)'; });
    div.addEventListener('click', function() {
      var ssidInput = document.getElementById('wifiSsidInput');
      var passInput = document.getElementById('wifiPassInput');
      if (ssidInput) ssidInput.value = this.dataset.ssid;
      if (passInput) { passInput.value = ''; passInput.focus(); }
    });
    listEl.appendChild(div);
  }
}

function connectWifi() {
  var ssid = document.getElementById('wifiSsidInput');
  var pass = document.getElementById('wifiPassInput');
  var statusEl = document.getElementById('wifiConnectStatus');
  if (!ssid || !ssid.value.trim()) {
    if (statusEl) { statusEl.textContent = 'Bitte SSID eingeben'; statusEl.style.color = '#ef4444'; }
    return;
  }
  if (statusEl) { statusEl.textContent = 'Verbinde...'; statusEl.style.color = '#f59e0b'; }
  var params = new URLSearchParams();
  params.append('ssid', ssid.value.trim());
  if (pass && pass.value) params.append('password', pass.value);
  postForm('/wifi/connect', params).then(function(res) { return res.json(); }).then(function(data) {
    if (data.success) {
      if (statusEl) { statusEl.textContent = '\u2713 Gespeichert! Neuverbindung l\u00e4uft...'; statusEl.style.color = '#22c55e'; }
      setTimeout(function() { loadWifiInfo(); }, 5000);
    } else {
      if (statusEl) { statusEl.textContent = 'Fehler: ' + (data.message || 'Unbekannt'); statusEl.style.color = '#ef4444'; }
    }
  }).catch(function(e) {
    console.error('WiFi connect error:', e);
    if (statusEl) { statusEl.textContent = 'Verbindungsfehler - Seite neu laden wenn IP sich \u00e4ndert'; statusEl.style.color = '#f59e0b'; }
  });
}

function setupWifi() {
  var scanBtn = document.getElementById('wifiScanBtn');
  if (scanBtn) scanBtn.addEventListener('click', startWifiScan);

  var connectBtn = document.getElementById('wifiConnectBtn');
  if (connectBtn) connectBtn.addEventListener('click', connectWifi);

  var showPassBtn = document.getElementById('wifiShowPassBtn');
  if (showPassBtn) {
    showPassBtn.addEventListener('click', function() {
      var passInput = document.getElementById('wifiPassInput');
      if (!passInput) return;
      if (passInput.type === 'password') {
        passInput.type = 'text';
        showPassBtn.textContent = 'Verbergen';
      } else {
        passInput.type = 'password';
        showPassBtn.textContent = 'Zeigen';
      }
    });
  }
}

// ---- Sensor Management ----
function loadSensorInfo() {
  fetch('/sensor/info').then(function(res) { return res.json(); }).then(function(data) {
    var el = document.getElementById('sensorList');
    if (!el) return;
    var html = '';
    if (data.count === 0) {
      html = '<div style="color:#ef4444;font-size:14px;">Keine DS18B20-Sensoren gefunden.</div>';
    } else {
      html += '<div style="margin-bottom:8px;font-size:13px;color:#94a3b8;">Gefundene Sensoren: ' + data.count + '</div>';
      var sensors = data.sensors || [];
      for (var i = 0; i < sensors.length; i++) {
        var s = sensors[i];
        var role = '';
        if (s.id === data.before_fan_id) role = '<span style="color:#38bdf8;font-weight:600;"> \u2190 Vor L\u00fcfter (Zuluft)</span>';
        else if (s.id === data.after_heater_id) role = '<span style="color:#f97316;font-weight:600;"> \u2190 Nach Heizung (Wand)</span>';
        html += '<div style="padding:8px 12px;margin-bottom:6px;background:rgba(30,41,59,0.7);border-radius:8px;font-family:monospace;">';
        html += '<div style="font-size:13px;color:#e2e8f0;">ID: ' + s.id + role + '</div>';
        html += '<div style="font-size:12px;color:#94a3b8;margin-top:2px;">Temp: ' + (typeof s.temp === 'number' ? s.temp.toFixed(1) + ' \u00b0C' : '--') + '</div>';
        html += '</div>';
      }
      if (data.assigned) {
        html += '<div style="margin-top:8px;font-size:12px;color:#64748b;">';
        html += 'Vor L\u00fcfter: <span style="color:#38bdf8;font-family:monospace;">' + (data.before_fan_id || '--') + '</span><br>';
        html += 'Nach Heizung: <span style="color:#f97316;font-family:monospace;">' + (data.after_heater_id || '--') + '</span>';
        html += '</div>';
      }
    }
    el.innerHTML = html;
  }).catch(function(e) {
    console.error('Sensor info error:', e);
    var el = document.getElementById('sensorList');
    if (el) el.innerHTML = '<div style="color:#ef4444;">Fehler beim Laden der Sensorinformationen.</div>';
  });
}

function setupSensors() {
  var swapBtn = document.getElementById('sensorSwapBtn');
  if (swapBtn) {
    swapBtn.addEventListener('click', function() {
      postForm('/sensor/swap', new URLSearchParams()).then(function(res) {
        if (res.ok) {
          var st = document.getElementById('sensorStatus');
          if (st) { st.textContent = '\u2713 Sensoren getauscht'; setTimeout(function() { st.textContent = ''; }, 3000); }
          loadSensorInfo();
          loadStatus();
        }
      }).catch(function(e) { console.error(e); });
    });
  }

  var refreshBtn = document.getElementById('sensorRefreshBtn');
  if (refreshBtn) {
    refreshBtn.addEventListener('click', loadSensorInfo);
  }
}

// ---- New Buttons: Reboot, Bath Pause, Auto Toggle, Chart Height ----
function setupNewButtons() {
  // Reboot
  var rebootBtn = document.getElementById('rebootBtn');
  if (rebootBtn) {
    rebootBtn.addEventListener('click', function() {
      if (confirm('Ger\u00e4t wirklich neu starten?')) {
        postForm('/reboot', new URLSearchParams()).then(function() {
          rebootBtn.textContent = 'Neustart l\u00e4uft...';
          rebootBtn.disabled = true;
        }).catch(function(e) { console.error(e); });
      }
    });
  }

  // Bath pause
  var bpBtn = document.getElementById('bathPauseBtn');
  if (bpBtn) {
    bpBtn.addEventListener('click', function() {
      var params = new URLSearchParams();
      // Check if pause is active - if so, cancel it (text shows countdown or BEENDEN)
      if (bpBtn.textContent.indexOf('BEENDEN') >= 0 || bpBtn.textContent.indexOf('Badepause:') >= 0) {
        params.append('action', 'stop');
      } else {
        var slider = document.getElementById('bathPauseMinsSlider');
        var mins = slider ? slider.value : 30;
        params.append('minutes', mins);
      }
      postForm('/bath_pause', params).then(function(res) { if (res.ok) loadStatus(); }).catch(function(e) { console.error(e); });
    });
  }

  // Bath pause slider
  var bpSlider = document.getElementById('bathPauseMinsSlider');
  var bpValue = document.getElementById('bathPauseMinsValue');
  if (bpSlider && bpValue) {
    bpSlider.addEventListener('input', function() {
      bpValue.textContent = bpSlider.value;
    });
  }

  // Auto toggle
  var atBtn = document.getElementById('autoToggleBtn');
  if (atBtn) {
    atBtn.addEventListener('click', function() {
      postForm('/auto_toggle', new URLSearchParams()).then(function(res) { if (res.ok) loadStatus(); }).catch(function(e) { console.error(e); });
    });
  }

  // Chart 1 height slider
  var chSlider = document.getElementById('chartHeightSlider');
  var chValue = document.getElementById('chartHeightValue');
  if (chSlider && chValue) {
    var savedHeight = localStorage.getItem('chartHeight');
    if (savedHeight) {
      chSlider.value = savedHeight;
      chValue.textContent = savedHeight + 'px';
      var canvas = document.getElementById('humChart');
      if (canvas) canvas.style.height = savedHeight + 'px';
    }
    chSlider.addEventListener('input', function() {
      chValue.textContent = chSlider.value + 'px';
      var canvas = document.getElementById('humChart');
      if (canvas) canvas.style.height = chSlider.value + 'px';
    });
    chSlider.addEventListener('change', function() {
      localStorage.setItem('chartHeight', chSlider.value);
      if (lastHistoryPoints.length > 0) renderBothCharts();
    });
  }

  // Chart 2 height slider
  var ch2Slider = document.getElementById('chart2HeightSlider');
  var ch2Value = document.getElementById('chart2HeightValue');
  if (ch2Slider && ch2Value) {
    var savedHeight2 = localStorage.getItem('chart2Height');
    if (savedHeight2) {
      ch2Slider.value = savedHeight2;
      ch2Value.textContent = savedHeight2 + 'px';
      var canvas2 = document.getElementById('humChart2');
      if (canvas2) canvas2.style.height = savedHeight2 + 'px';
    }
    ch2Slider.addEventListener('input', function() {
      ch2Value.textContent = ch2Slider.value + 'px';
      var canvas2 = document.getElementById('humChart2');
      if (canvas2) canvas2.style.height = ch2Slider.value + 'px';
    });
    ch2Slider.addEventListener('change', function() {
      localStorage.setItem('chart2Height', ch2Slider.value);
      if (lastHistoryPoints.length > 0) renderBothCharts();
    });
  }

  // Time window selectors
  var timeSel1 = document.getElementById('chart1TimeMins');
  if (timeSel1) {
    var saved1 = localStorage.getItem('chart1TimeMins');
    if (saved1) timeSel1.value = saved1;
    timeSel1.addEventListener('change', function() {
      localStorage.setItem('chart1TimeMins', timeSel1.value);
      if (lastHistoryPoints.length > 0) renderBothCharts();
    });
  }
  var timeSel2 = document.getElementById('chart2TimeMins');
  if (timeSel2) {
    var saved2 = localStorage.getItem('chart2TimeMins');
    if (saved2) timeSel2.value = saved2;
    timeSel2.addEventListener('change', function() {
      localStorage.setItem('chart2TimeMins', timeSel2.value);
      if (lastHistoryPoints.length > 0) renderBothCharts();
    });
  }
}

function setupEnergy() {
  // Energy settings are now loaded from server config in loadConfig()
  var wattsEl = document.getElementById('energyHeaterWatts');
  var priceEl = document.getElementById('energyPriceKwh');
  
  // Save settings to server on change
  if (wattsEl) wattsEl.addEventListener('change', function() {
    var params = new URLSearchParams();
    params.append('heater_watts', this.value);
    postForm('/cfg', params);
  });
  if (priceEl) priceEl.addEventListener('change', function() {
    var params = new URLSearchParams();
    params.append('energy_price_kwh', this.value);
    postForm('/cfg', params);
  });
  
  // Reset button
  var resetBtn = document.getElementById('energyResetBtn');
  if (resetBtn) {
    resetBtn.addEventListener('click', function() {
      if (confirm('Heizungszähler wirklich zurücksetzen?')) {
        postForm('/heater_reset', new URLSearchParams()).then(function(res) {
          if (res.ok) {
            var st = document.getElementById('energyStatus');
            if (st) { st.textContent = '✓ Zurückgesetzt'; setTimeout(function() { st.textContent = ''; }, 3000); }
            loadStatus();
          }
        }).catch(function(e) { console.error(e); });
      }
    });
  }
}

function setupConfigForm() {
  var form = document.getElementById('cfgForm');
  if (form) form.addEventListener('submit', saveConfig);

  var durInput = document.getElementById('manualDurationSetting');
  if (durInput) {
    durInput.addEventListener('change', function() {
      localStorage.setItem('manualDuration', this.value);
    });
  }
}

// ---- Init ----
function initApp() {
  setupButtons();
  setupFanSpeed();
  setupSchedule();
  setupAbsaugenCyclic();
  setupCyclic();
  setupConfigForm();
  setupWifi();
  setupSensors();
  setupNewButtons();
  setupEnergy();

  // Generic chart tooltip setup
  function setupChartTooltip(canvasId, tooltipId, getMetaFn) {
    var canvas = document.getElementById(canvasId);
    var tooltip = document.getElementById(tooltipId);
    if (!canvas || !tooltip) return;
    canvas.addEventListener('mousemove', function(e) {
      var m = getMetaFn();
      if (!m || !m.points || !m.points.length) { tooltip.style.display = 'none'; return; }
      var rect = canvas.getBoundingClientRect();
      var mx = e.clientX - rect.left;
      var my = e.clientY - rect.top;
      if (mx < m.padL || mx > m.padL + m.iw || my < m.padT || my > m.padT + m.ih) { tooltip.style.display = 'none'; return; }
      var timeFrac = (mx - m.padL) / m.iw;
      var targetTime = m.minTime + timeFrac * (m.maxTime - m.minTime);
      var closest = null, closestDist = Infinity;
      for (var ci = 0; ci < m.points.length; ci++) {
        var dist = Math.abs(m.points[ci].dt_sec - targetTime);
        if (dist < closestDist) { closestDist = dist; closest = m.points[ci]; }
      }
      if (!closest) { tooltip.style.display = 'none'; return; }
      var lines = [];
      if (lastHistoryEpoch) {
        var d = new Date((lastHistoryEpoch + closest.dt_sec) * 1000);
        lines.push('<div style="color:#9ca3af;margin-bottom:4px;">' + ('0'+d.getHours()).slice(-2) + ':' + ('0'+d.getMinutes()).slice(-2) + '</div>');
      }
      if (m.hasHumidity && typeof closest.humidity === 'number' && !isNaN(closest.humidity)) {
        lines.push('<div><span style="color:#38bdf8;">\u25cf</span> Feuchte: <strong>' + closest.humidity.toFixed(1) + ' %</strong></div>');
      }
      if (m.hasTemp && typeof closest.temperature === 'number' && !isNaN(closest.temperature)) {
        lines.push('<div><span style="color:#f97316;">\u25cf</span> Temp: <strong>' + closest.temperature.toFixed(1) + ' \u00b0C</strong></div>');
      }
      var fanInfo = closest.fan_on ? ('L\u00fcfter AN' + (closest.fan_dir === 'absaugen' ? ' <span style="color:#22c55e;">(Absaugen)</span>' : ' <span style="color:#3b82f6;">(Einblasen)</span>')) : 'L\u00fcfter AUS';
      lines.push('<div style="font-size:11px;color:#64748b;">' + fanInfo + (closest.heater_on ? ' | Heizung AN' : '') + '</div>');
      tooltip.innerHTML = lines.join('');
      tooltip.style.display = 'block';
      var ttLeft = mx + 12;
      if (ttLeft + 160 > rect.width) ttLeft = mx - 170;
      tooltip.style.left = ttLeft + 'px';
      tooltip.style.top = (my - 10) + 'px';
    });
    canvas.addEventListener('mouseleave', function() {
      tooltip.style.display = 'none';
    });
  }

  setupChartTooltip('humChart', 'chartTooltip', function() { return chartMeta; });
  setupChartTooltip('humChart2', 'chart2Tooltip', function() { return chart2Meta; });

  // Initial loads
  loadStatus();
  loadConfig();
  loadHistory();

  // Periodic updates
  statusInterval = setInterval(loadStatus, 2000);
  historyInterval = setInterval(loadHistory, 60000);

  // Resize handler for both charts
  window.addEventListener('resize', function() {
    if (lastHistoryPoints.length > 0) renderBothCharts();
  });

  // Online/offline handlers
  window.addEventListener('online', function() {
    statusRetryCount = 0;
    loadStatus();
  });

  window.addEventListener('focus', loadStatus);
}

// ---- Firmware Tab ----
function updateFirmwareInfo() {
  var el = function(id) { return document.getElementById(id); };
  fetch('/status').then(function(r) { return r.json(); }).then(function(s) {
    if (el('fwVersion')) el('fwVersion').textContent = s.fw_version || '-';
    if (el('fwBuild')) el('fwBuild').textContent = s.fw_build || '-';
    if (el('fwHeap') && typeof s.free_heap === 'number') el('fwHeap').textContent = (s.free_heap / 1024).toFixed(1) + ' KB';
    if (el('fwUptime') && typeof s.uptime_secs === 'number') {
      var u = s.uptime_secs;
      var d = Math.floor(u / 86400); u %= 86400;
      var h = Math.floor(u / 3600); u %= 3600;
      var m = Math.floor(u / 60);
      el('fwUptime').textContent = (d > 0 ? d + 'd ' : '') + h + 'h ' + m + 'm';
    }
  }).catch(function() {});
}

function uploadFirmware() {
  var fileInput = document.getElementById('fwFile');
  if (!fileInput || !fileInput.files.length) {
    document.getElementById('fwStatus').textContent = 'Bitte Datei ausw\u00e4hlen!';
    document.getElementById('fwStatus').style.color = '#ef4444';
    return;
  }
  var file = fileInput.files[0];
  var formData = new FormData();
  formData.append('update', file, file.name);

  var xhr = new XMLHttpRequest();
  var progressDiv = document.getElementById('fwProgress');
  var progressBar = document.getElementById('fwProgressBar');
  var statusDiv = document.getElementById('fwStatus');
  var btn = document.getElementById('fwUploadBtn');

  progressDiv.style.display = 'block';
  btn.disabled = true;
  btn.style.opacity = '0.5';
  statusDiv.textContent = 'Upload l\u00e4uft...';
  statusDiv.style.color = '#f59e0b';

  xhr.upload.addEventListener('progress', function(e) {
    if (e.lengthComputable) {
      var pct = Math.round((e.loaded / e.total) * 100);
      progressBar.style.width = pct + '%';
      progressBar.textContent = pct + '%';
    }
  });

  xhr.addEventListener('load', function() {
    btn.disabled = false;
    btn.style.opacity = '1';
    try {
      var resp = JSON.parse(xhr.responseText);
      if (resp.success) {
        statusDiv.textContent = resp.msg || 'Update erfolgreich!';
        statusDiv.style.color = '#22c55e';
        progressBar.style.background = 'linear-gradient(90deg,#22c55e,#16a34a)';
        setTimeout(function() {
          statusDiv.textContent = 'Ger\u00e4t startet neu... Seite wird in 10s neu geladen.';
          setTimeout(function() { location.reload(); }, 10000);
        }, 1000);
      } else {
        statusDiv.textContent = resp.msg || 'Update fehlgeschlagen!';
        statusDiv.style.color = '#ef4444';
        progressBar.style.background = 'linear-gradient(90deg,#ef4444,#dc2626)';
      }
    } catch (e) {
      statusDiv.textContent = 'Antwort-Fehler: ' + xhr.responseText;
      statusDiv.style.color = '#ef4444';
    }
  });

  xhr.addEventListener('error', function() {
    btn.disabled = false;
    btn.style.opacity = '1';
    statusDiv.textContent = 'Verbindungsfehler beim Upload.';
    statusDiv.style.color = '#ef4444';
  });

  xhr.open('POST', '/update');
  xhr.send(formData);
}

function doReboot() {
  if (!confirm('Ger\u00e4t wirklich neustarten?')) return;
  fetch('/reboot', { method: 'POST' }).then(function() {
    document.getElementById('fwStatus').textContent = 'Neustart... Seite wird in 10s neu geladen.';
    document.getElementById('fwStatus').style.color = '#f59e0b';
    setTimeout(function() { location.reload(); }, 10000);
  });
}

// Start app - script is at end of body so DOM is ready
initApp();
