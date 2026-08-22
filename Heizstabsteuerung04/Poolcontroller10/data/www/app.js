// ============================================================
// Pool Controller - Web UI
// Visual style: ported from example/data/www (Feuchtesteuerung).
// ============================================================
const $  = (sel) => document.querySelector(sel);
const $$ = (sel) => document.querySelectorAll(sel);

// ---------- Tabs ----------
$$('.tab').forEach(btn => btn.addEventListener('click', () => {
  $$('.tab').forEach(b => b.classList.remove('active'));
  $$('.tab-content').forEach(p => p.classList.remove('active'));
  btn.classList.add('active');
  $('#tab-' + btn.dataset.tab).classList.add('active');
  if (btn.dataset.tab === 'settings' || btn.dataset.tab === 'system') loadConfig();
  if (btn.dataset.tab === 'schedule') loadScheduleUI();
}));

// ---------- Draggable tab order (persisted in NVS via /api/tabs) ----------
// Order is a CSV of data-tab values. Drag a tab header onto another to
// reorder. The new order is POSTed back to the device so every browser
// sees the same arrangement.
(() => {
  const nav = document.querySelector('.tabs');
  if (!nav) return;

  function currentOrder() {
    return Array.from(nav.querySelectorAll('.tab'))
      .map(t => t.dataset.tab).join(',');
  }

  async function saveOrder() {
    try {
      const order = currentOrder();
      console.log('[Tabs] Saving order:', order);
      const r = await fetch('/api/tabs', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'order=' + encodeURIComponent(order),
      });
      const d = await r.json();
      console.log('[Tabs] Save response:', d);
    } catch (e) {
      console.error('[Tabs] Save failed:', e);
    }
  }

  function applyOrder(csv) {
    if (!csv) return;
    const wanted = csv.split(',').filter(Boolean);
    // Re-append tabs in the wanted order; unknown / missing ones keep
    // their natural DOM position at the end.
    for (const key of wanted) {
      const el = nav.querySelector(`.tab[data-tab="${key}"]`);
      if (el) nav.appendChild(el);
    }
  }

  // --- Drag-and-drop wiring --------------------------------------------
  let dragged = null;
  for (const tab of nav.querySelectorAll('.tab')) {
    tab.setAttribute('draggable', 'true');
    tab.addEventListener('dragstart', e => {
      dragged = tab;
      tab.classList.add('dragging');
      e.dataTransfer.effectAllowed = 'move';
      // Firefox needs some data set to actually start a drag.
      try { e.dataTransfer.setData('text/plain', tab.dataset.tab); } catch (_) {}
    });
    tab.addEventListener('dragend', () => {
      tab.classList.remove('dragging');
      dragged = null;
      saveOrder();
    });
    tab.addEventListener('dragover', e => {
      if (!dragged || dragged === tab) return;
      e.preventDefault();
      e.dataTransfer.dropEffect = 'move';
      // Insert BEFORE or AFTER depending on horizontal midpoint.
      const r = tab.getBoundingClientRect();
      const before = (e.clientX - r.left) < r.width / 2;
      nav.insertBefore(dragged, before ? tab : tab.nextSibling);
    });
    tab.addEventListener('drop', e => { e.preventDefault(); });
  }

  // Load persisted order on startup.
  fetch('/api/tabs').then(r => r.json()).then(d => applyOrder(d && d.order))
    .catch(() => {});
})();

// ---------- Utilities ----------
function fmtTemp(v)  { return (v == null) ? '—' : (Number(v).toFixed(1) + ' °C'); }
function fmtMv(v)    { return (v == null) ? '—' : (Math.round(v) + ' mV'); }
function fmtVolt(v)  { return (v == null) ? '—' : (Number(v).toFixed(4) + ' V'); }
function fmtRssi(v)  { return (v == null) ? '—' : (v + ' dBm'); }
function fmtUptime(s){
  s = Math.floor(Number(s) || 0);
  const d = Math.floor(s / 86400); s %= 86400;
  const h = Math.floor(s / 3600);  s %= 3600;
  const m = Math.floor(s / 60);    s %= 60;
  return (d ? d + 'd ' : '')
       + String(h).padStart(2,'0') + ':'
       + String(m).padStart(2,'0') + ':'
       + String(s).padStart(2,'0');
}
function led(id, on, cls = 'on-green') {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.remove('on-green', 'on-amber', 'on-red');
  if (on) el.classList.add(cls);
}
async function jget(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(r.statusText);
  return r.json();
}
async function jpost(url, params) {
  const body = new URLSearchParams(params || {}).toString();
  const r = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body,
  });
  if (!r.ok) throw new Error(r.statusText);
  return r.json();
}

// ---------- Datetime bar (server clock, re-synced from /api/status) ----
// `g_serverEpoch` holds the last epoch received from the ESP32;
// `g_serverAnchorMs` is the browser millis() at which it arrived.
// Between status polls we advance the display using performance.now().
let g_serverEpoch   = null;
let g_serverAnchorMs = 0;
let g_ntpSynced    = false;

function setServerClock(epochSeconds, synced) {
  g_ntpSynced = !!synced;
  if (typeof epochSeconds === 'number' && epochSeconds > 1700000000) {
    g_serverEpoch    = epochSeconds;
    g_serverAnchorMs = performance.now();
  }
}

function pad2(n) { return String(n).padStart(2, '0'); }
const WD = ['So','Mo','Di','Mi','Do','Fr','Sa'];
function tickClock() {
  const el = document.getElementById('metaClock');
  if (!el) return;
  if (!g_ntpSynced || g_serverEpoch == null) {
    el.textContent = 'NTP …';
    el.style.color = '#94a3b8';
    return;
  }
  const delta = (performance.now() - g_serverAnchorMs) / 1000;
  const d = new Date((g_serverEpoch + delta) * 1000);
  el.textContent =
    WD[d.getDay()] + ' ' +
    pad2(d.getDate()) + '.' + pad2(d.getMonth()+1) + '.' + d.getFullYear() + ' ' +
    pad2(d.getHours()) + ':' + pad2(d.getMinutes()) + ':' + pad2(d.getSeconds());
  el.style.color = '#e5e7eb';
}
setInterval(tickClock, 1000);

// ---------- Toggle buttons (Pumpe / Ely A/B/K) ----------
function setToggleBtn(btn, on, labelBase) {
  btn.textContent = labelBase + (on ? ' AN' : ' AUS');
  btn.classList.toggle('active', !!on);
}
const TOGGLE_BTNS = [
  { id:'btnPump', cmd:'pump',  label:'Pumpe' },
  { id:'btnElyA', cmd:'ely_a', label:'Ely A' },
  { id:'btnElyB', cmd:'ely_b', label:'Ely B' },
  { id:'btnElyK', cmd:'ely_k', label:'Ely K' },
];
TOGGLE_BTNS.forEach(b => {
  const el = document.getElementById(b.id);
  if (!el) return;
  el.addEventListener('click', async () => {
    const willOn = !el.classList.contains('active');
    const params = {}; params[b.cmd] = willOn ? 'on' : 'off';
    try {
      await jpost('/api/cmd', params);
      setToggleBtn(el, willOn, b.label);
      refreshStatus();
    } catch (e) { alert('Fehler: ' + e.message); }
  });
});

// ---------- Status: one-shot fetch + SSE stream ----------
// `applyStatus(s)` is the pure DOM-update routine, called from:
//   * refreshStatus()  - one-shot GET /api/status (initial load + post-action)
//   * EventSource      - push from ESP32 on state change or 2 s heartbeat
async function refreshStatus() {
  try {
    const s = await jget('/api/status');
    applyStatus(s);
  } catch (e) {
    const m = $('#metaMqttState'); if (m) m.textContent = 'Verbindung verloren';
    const p = $('#metaMqtt');      if (p) p.classList.remove('on');
  }
}
function applyStatus(s) {
  try {
    // Header bar
    const elHost = $('#metaHost');
    if (elHost) elHost.textContent = s.hostname || 'Pool Controller';
    const elMqttState = $('#metaMqttState');
    if (elMqttState) elMqttState.textContent = s.mqtt_connected ? 'online' : 'offline';
    const elMqtt = $('#metaMqtt');
    if (elMqtt) elMqtt.classList.toggle('on', !!s.mqtt_connected);

    // Metrics
    $('#vTemp').textContent   = fmtTemp(s.temp_pool);
    $('#vTempAir').textContent = fmtTemp(s.temp_air);
    $('#vOrp').textContent    = fmtMv(s.orp_mv);
    $('#vVolt').textContent   = fmtVolt(s.voltage);
    $('#vUptime').textContent = fmtUptime(s.uptime_s);
    $('#vRssi').textContent   = fmtRssi(s.wifi_rssi);
    $('#vTrafo').textContent  = s.transformer ? 'AN' : 'AUS';
    $('#vFlow').textContent   = s.flow ? 'OK' : 'kein Flow';

    // LED bar
    led('ledPump',   s.pump);
    led('ledFlow',   s.flow);
    led('ledFlowErr',s.flow_error, 'on-red');
    led('ledElyA',   s.ely_a);
    led('ledElyB',   s.ely_b);
    led('ledElyK',   s.ely_k);
    led('ledTrafo',  s.transformer);
    led('ledDs',     s.sensor_ds18b20_ok);
    led('ledAds',    s.sensor_ads1115_ok);
    led('ledMqtt',   s.mqtt_connected);
    led('ledSchedule', s.schedule_active);
    led('ledAuto',     s.ely_ab_auto);
    led('ledNtp',      s.ntp_synced);
    setServerClock(s.now_epoch, s.ntp_synced);

    // Zeitplan live
    const live = $('#schedLive');
    if (live) {
      if (!s.schedule_enabled)     { live.textContent = 'Plan deaktiviert'; live.style.color = '#94a3b8'; }
      else if (s.schedule_active)  { live.textContent = 'diese Stunde: AKTIV'; live.style.color = '#22c55e'; }
      else                          { live.textContent = 'diese Stunde: GESPERRT'; live.style.color = '#ef4444'; }
    }

    // Ely A/B phase display
    const phase = s.ely_ab_phase || 0;
    const phaseEl = document.getElementById('vElyABPhase');
    const cdEl    = document.getElementById('vElyABCountdown');
    if (phaseEl) {
      phaseEl.textContent = phase === 1 ? 'A aktiv' : phase === 2 ? 'B aktiv' : 'beide AUS';
      phaseEl.style.color = phase === 0 ? '#94a3b8' : '#22c55e';
    }
    if (cdEl) {
      if (phase !== 0 && s.ely_ab_next_switch_secs != null) {
        cdEl.textContent = s.ely_ab_next_switch_secs + ' s';
      } else {
        cdEl.textContent = '—';
      }
    }

    // Toggle buttons
    TOGGLE_BTNS.forEach(b => {
      const el = document.getElementById(b.id);
      if (el && !el.matches(':active')) setToggleBtn(el, !!s[b.cmd], b.label);
    });

    // System card
    $('#sysFw').textContent    = s.firmware || '—';
    $('#sysBuild').textContent = s.build || '—';
    $('#sysHost').textContent  = s.hostname || '—';
    $('#sysDs').textContent    = s.sensor_ds18b20_ok ? 'OK' : 'FEHLT';
    $('#sysAds').textContent   = s.sensor_ads1115_ok ? 'OK' : 'FEHLT';

    // HomeKit card
    const hkSetup = $('#hkSetup');
    if (hkSetup && s.homekit_setup_code) {
      // Format 46637726 -> "466-37-726" like Apple shows on stickers/QRs.
      const c = String(s.homekit_setup_code);
      hkSetup.textContent = c.length === 8
        ? `${c.slice(0,3)}-${c.slice(3,5)}-${c.slice(5)}`
        : c;
    }
    const hkPaired = $('#hkPaired');
    if (hkPaired) {
      hkPaired.textContent = s.homekit_paired ? 'gekoppelt' : 'noch nicht gekoppelt';
      hkPaired.style.color = s.homekit_paired ? '#22c55e' : '#94a3b8';
    }

    // Pinout live state
    updatePinoutLive(s);

    // WiFi banner
    updateWifiBanner(s);

    // Discovered DS18B20 sensors
    renderDiscoveredSensors(s);
  } catch (e) {
    // offline: turn MQTT LED + pill off
    led('ledMqtt', false);
    const p = document.getElementById('metaMqtt');
    if (p) p.classList.remove('on');
    const ps = document.getElementById('metaMqttState');
    if (ps) ps.textContent = 'offline';
    console.warn('status', e);
  }
}

// ---------- Config ----------
async function loadConfig() {
  try {
    const c = await jget('/api/config');
    $('#cfgWifiSsid').value     = c.wifi_ssid || '';
    $('#cfgMqttHost').value     = c.mqtt_host || '';
    $('#cfgMqttPort').value     = c.mqtt_port || 1883;
    $('#cfgMqttUser').value     = c.mqtt_user || '';
    $('#cfgMqttBase').value     = c.mqtt_base_topic || 'poolcontroller';
    $('#cfgMqttInterval').value = c.mqtt_interval_secs || 10;
    $('#cfgOrpCal').value       = c.orp_calibration || 0;
    $('#cfgElyABSwitch').value  = c.ely_ab_switch_secs || 600;
    const elAuto = $('#cfgElyABAuto'); if (elAuto) elAuto.checked = !!c.ely_ab_auto;
    $('#cfgPumpStartOn').value  = (c.pump_start_on === false) ? 'false' : 'true';
    $('#cfgTempPoolId').value   = c.temp_sensor_pool_id || '';
    $('#cfgTempAirId').value    = c.temp_sensor_air_id || '';
  } catch (e) { console.warn('cfg', e); }
}

function formToObject(form) {
  const data = Object.fromEntries(new FormData(form).entries());
  // strip empty password so we don't overwrite the stored one
  for (const k of Object.keys(data)) {
    if (/password/i.test(k) && !data[k]) delete data[k];
  }
  return data;
}

$('#formWifi').addEventListener('submit', async (e) => {
  e.preventDefault();
  await jpost('/api/config', formToObject(e.target));
  const s = $('#wifiStatus'); s.textContent = 'WiFi gespeichert. Neustart erforderlich.';
  setTimeout(() => s.textContent = '', 4000);
});

$('#formStart').addEventListener('submit', async (e) => {
  e.preventDefault();
  await jpost('/api/config', formToObject(e.target));
  const s = $('#startStatus'); s.textContent = 'Gespeichert. Wirkt beim nächsten Neustart.';
  setTimeout(() => s.textContent = '', 3500);
});

$('#formElyAB').addEventListener('submit', async (e) => {
  e.preventDefault();
  const data = formToObject(e.target);
  // Unchecked checkboxes are not in FormData -> emit explicit "false"
  data.ely_ab_auto = $('#cfgElyABAuto').checked ? 'true' : 'false';
  await jpost('/api/config', data);
  const s = $('#elyABStatus'); s.textContent = 'Gespeichert.';
  setTimeout(() => s.textContent = '', 3000);
});

function renderDiscoveredSensors(s) {
  const wrap = document.getElementById('sensorDiscoveredList');
  if (!wrap) return;
  const list = Array.isArray(s.temp_discovered) ? s.temp_discovered : [];
  const poolRom = (s.temp_pool_rom || '').toUpperCase();
  const airRom  = (s.temp_air_rom  || '').toUpperCase();
  if (list.length === 0) {
    wrap.innerHTML = '<div class="field-hint" style="color:#ef4444;">Keine DS18B20 am Bus erkannt.</div>';
    return;
  }
  wrap.innerHTML = '';
  list.forEach((rom) => {
    const row = document.createElement('div');
    row.style.cssText = 'display:flex;align-items:center;gap:8px;padding:8px 10px;background:rgba(15,23,42,0.6);border:1px solid #334155;border-radius:6px;flex-wrap:wrap;';
    const ROM = rom.toUpperCase();
    const isPool = ROM === poolRom;
    const isAir  = ROM === airRom;
    const role   = isPool ? '<span style="color:#22c55e;font-weight:600;">Pool</span>'
                 : isAir  ? '<span style="color:#38bdf8;font-weight:600;">Luft</span>'
                 : '<span style="color:#94a3b8;">—</span>';
    row.innerHTML =
      `<code style="font-family:ui-monospace,monospace;flex:1;min-width:170px;font-size:12px;color:#e2e8f0;">${ROM}</code>` +
      `<span style="min-width:50px;text-align:center;">${role}</span>` +
      `<button type="button" class="btn-secondary" data-rom="${ROM}" data-target="pool" style="padding:4px 10px;font-size:12px;">→ Pool</button>` +
      `<button type="button" class="btn-secondary" data-rom="${ROM}" data-target="air"  style="padding:4px 10px;font-size:12px;">→ Luft</button>`;
    wrap.appendChild(row);
  });
  // Wire up assign buttons
  wrap.querySelectorAll('button[data-rom]').forEach(btn => {
    btn.addEventListener('click', () => {
      const rom = btn.dataset.rom;
      if (btn.dataset.target === 'pool') {
        document.getElementById('cfgTempPoolId').value = rom;
      } else {
        document.getElementById('cfgTempAirId').value = rom;
      }
    });
  });
}

$('#formSensors').addEventListener('submit', async (e) => {
  e.preventDefault();
  const data = formToObject(e.target);
  // Convert to uppercase for consistency
  if (data.temp_sensor_pool_id) data.temp_sensor_pool_id = data.temp_sensor_pool_id.toUpperCase();
  if (data.temp_sensor_air_id) data.temp_sensor_air_id = data.temp_sensor_air_id.toUpperCase();
  await jpost('/api/config', data);
  const s = $('#sensorStatus'); s.textContent = 'Sensor-Zuordnung gespeichert.';
  setTimeout(() => s.textContent = '', 3000);
});

$('#btnScanSensors').addEventListener('click', async () => {
  const r = $('#sensorScanResult');
  r.textContent = 'Scanne 1-Wire Bus...';
  try {
    const resp = await jpost('/api/1w/scan', {});
    if (resp.success) {
      let msg = `Gefunden: Pool=${resp.pool_found ? 'ja' : 'nein'}, Luft=${resp.air_found ? 'ja' : 'nein'}`;
      if (resp.pool_found) msg += ` (Pool-ROM: ${resp.pool_rom})`;
      if (resp.air_found) msg += ` (Luft-ROM: ${resp.air_rom})`;
      if (resp.discovered && resp.discovered.length > 0) {
        msg += `. Alle Sensoren: ${resp.discovered.join(', ')}`;
      }
      r.textContent = msg;
      r.style.color = '#22c55e';
      // Reload status to update ROM fields
      updateStatus();
    } else {
      r.textContent = 'Keine Sensoren gefunden';
      r.style.color = '#ef4444';
    }
  } catch (e) {
    r.textContent = 'Scan fehlgeschlagen';
    r.style.color = '#ef4444';
  }
});

$('#formMqtt').addEventListener('submit', async (e) => {
  e.preventDefault();
  await jpost('/api/config', formToObject(e.target));
  const s = $('#mqttStatus'); s.textContent = 'MQTT gespeichert.';
  setTimeout(() => s.textContent = '', 4000);
});

// ---------- WiFi scan ----------
function rssiToBars(rssi) {
  if (rssi >= -55) return 4;
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  if (rssi >= -85) return 1;
  return 0;
}
function rssiQuality(rssi) {
  if (rssi >= -55) return 'ausgezeichnet';
  if (rssi >= -65) return 'gut';
  if (rssi >= -75) return 'mittel';
  if (rssi >= -85) return 'schwach';
  return 'sehr schwach';
}
function wifiCardHtml(n, curSsid) {
  const bars = rssiToBars(n.rssi);
  const active = (n.ssid && n.ssid === curSsid) ? ' wifi-card-active' : '';
  const barsHtml = [4,3,2,1].map(b =>
    `<span class="wifi-bar ${b <= bars ? 'on' : ''}" style="height:${b*3+3}px"></span>`
  ).join('');
  const lock = n.secure ? '<span class="wifi-lock" title="Verschlüsselt">🔒</span>'
                        : '<span class="wifi-lock open" title="Offen">🔓</span>';
  return `
    <button type="button" class="wifi-card${active}" data-ssid="${(n.ssid||'').replace(/"/g,'&quot;')}" data-secure="${n.secure?1:0}">
      <span class="wifi-card-bars" title="${rssiQuality(n.rssi)}">${barsHtml}</span>
      <span class="wifi-card-main">
        <span class="wifi-card-ssid">${n.ssid ? escapeHtml(n.ssid) : '<i>(versteckt)</i>'}</span>
        <span class="wifi-card-sub">Kanal ${n.channel} · ${n.rssi} dBm · ${rssiQuality(n.rssi)}</span>
      </span>
      ${lock}
    </button>`;
}
function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, m => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[m]));
}

async function scanWifi() {
  const grid    = $('#scanResult');
  const spin    = $('#scanSpinner');
  const btnLbl  = $('#scanBtnLabel');
  const btn     = $('#btnScan');
  spin.hidden   = false;
  btn.disabled  = true;
  btnLbl.textContent = 'Scanne …';
  grid.innerHTML = '<div class="wifi-placeholder">Scanne Umgebung …</div>';
  try {
    for (let i = 0; i < 12; i++) {
      const r = await jget('/api/wifi/scan');
      if (r.scanning) { await new Promise(res => setTimeout(res, 1500)); continue; }
      const nets = (r.networks || []).sort((a,b) => b.rssi - a.rssi);
      if (!nets.length) {
        grid.innerHTML = '<div class="wifi-placeholder">Keine Netzwerke gefunden.</div>';
        return;
      }
      const curSsid = $('#cfgWifiSsid').value;
      grid.innerHTML = nets.map(n => wifiCardHtml(n, curSsid)).join('');
      grid.querySelectorAll('.wifi-card').forEach(c => {
        c.addEventListener('click', () => {
          $('#cfgWifiSsid').value = c.dataset.ssid;
          grid.querySelectorAll('.wifi-card').forEach(x => x.classList.remove('wifi-card-active'));
          c.classList.add('wifi-card-active');
          if (c.dataset.secure === '1') $('#cfgWifiPassword').focus();
        });
      });
      return;
    }
    grid.innerHTML = '<div class="wifi-placeholder">Timeout – bitte erneut versuchen.</div>';
  } catch (e) {
    grid.innerHTML = '<div class="wifi-placeholder" style="color:#ef4444;">Fehler: ' + escapeHtml(e.message) + '</div>';
  } finally {
    spin.hidden  = true;
    btn.disabled = false;
    btnLbl.textContent = 'Netzwerke scannen';
  }
}
$('#btnScan').addEventListener('click', scanWifi);

// Password show/hide toggle
$('#btnPwToggle').addEventListener('click', () => {
  const inp = $('#cfgWifiPassword');
  inp.type = (inp.type === 'password') ? 'text' : 'password';
});

// Live WiFi banner signal quality – driven by refreshStatus().
function updateWifiBanner(s) {
  const ssid = s.wifi_ssid || '—';
  const rssi = (typeof s.wifi_rssi === 'number') ? s.wifi_rssi : null;
  $('#wifiCurSsid').textContent = ssid;
  $('#wifiCurIp').textContent   = 'IP ' + (s.ip || '—');
  $('#wifiCurRssi').textContent = (rssi == null) ? '— dBm' : (rssi + ' dBm · ' + rssiQuality(rssi));
  $('#wifiCurHost').textContent = 'host ' + (s.hostname || '—');
  const icon = $('#wifiBannerIcon');
  const bars = (rssi == null) ? 0 : rssiToBars(rssi);
  icon.classList.remove('bars-0','bars-1','bars-2','bars-3','bars-4');
  icon.classList.add('bars-' + bars);
}

// ---------- ORP calibration ----------
$('#btnCal').addEventListener('click', async () => {
  const s = $('#calStatus'); s.style.color = '#38bdf8'; s.textContent = 'Kalibriere…';
  try {
    const r = await jpost('/api/orp/cal', {});
    s.style.color = '#22c55e';
    s.textContent = `OK. Offset = ${r.calibration} mV (Raw ${Number(r.raw_mv).toFixed(1)} mV)`;
    $('#cfgOrpCal').value = r.calibration;
  } catch (e) { s.style.color = '#ef4444'; s.textContent = 'Fehler: ' + e.message; }
});
$('#btnSaveCal').addEventListener('click', async () => {
  await jpost('/api/config', { orp_calibration: $('#cfgOrpCal').value });
  const s = $('#calStatus'); s.style.color = '#22c55e'; s.textContent = 'Offset gespeichert.';
  setTimeout(() => s.textContent = '', 3000);
});

// ---------- Reboot ----------
$('#btnReboot').addEventListener('click', async () => {
  if (!confirm('Wirklich neu starten?')) return;
  await jpost('/api/reboot', {});
  alert('Neustart ausgelöst.');
});

// ---------- HomeKit: pairing reset ----------
const btnHkReset = document.getElementById('btnHkReset');
if (btnHkReset) btnHkReset.addEventListener('click', async () => {
  if (!confirm(
    'Wirklich das HomeKit-Pairing zurücksetzen?\n\n' +
    'Alle gekoppelten iPhones/HomePods/Apple-TVs verlieren die Verbindung. ' +
    'Der Controller startet anschließend neu und muss in der Apple Home-App ' +
    'einmal entfernt und neu hinzugefügt werden.'
  )) return;
  const s = document.getElementById('hkStatus');
  try {
    await jpost('/api/homekit/reset', {});
    if (s) { s.style.color = '#22c55e'; s.textContent = 'Pairing gelöscht, starte neu…'; }
  } catch (e) {
    if (s) { s.style.color = '#ef4444'; s.textContent = 'Fehler: ' + e.message; }
  }
});

// ======================= PINOUT tab =======================
const PIN_TOP = [
  { pin:'VIN',    alt:'5V in',       cat:'power',  note:'' },
  { pin:'GND',    alt:'',            cat:'gnd',    note:'' },
  { pin:'GPIO13', alt:'LCD MOSI',    cat:'spi',    note:'HSPI' },
  { pin:'GPIO12', alt:'ELY_K',       cat:'relay',  note:'',               liveKey:'ely_k', cmdKey:'ely_k' },
  { pin:'GPIO14', alt:'LCD SCK',     cat:'spi',    note:'HSPI' },
  { pin:'GPIO27', alt:'LCD CS',      cat:'spi',    note:'' },
  { pin:'GPIO26', alt:'I²C SCL',     cat:'i2c',    note:'ADS1115' },
  { pin:'GPIO25', alt:'I²C SDA',     cat:'i2c',    note:'ADS1115' },
  { pin:'GPIO33', alt:'reserve',     cat:'gpio',   note:'' },
  { pin:'GPIO32', alt:'reserve',     cat:'gpio',   note:'' },
  { pin:'GPIO35', alt:'reserve',     cat:'gpio',   note:'IN-only' },
  { pin:'GPIO34', alt:'reserve',     cat:'gpio',   note:'IN-only' },
  { pin:'GPIO39', alt:'VN reserve',  cat:'gpio',   note:'IN-only' },
  { pin:'GPIO36', alt:'VP reserve',  cat:'gpio',   note:'IN-only' },
  { pin:'EN',     alt:'reset',       cat:'en',     note:'' },
];
const PIN_BOT = [
  { pin:'3V3',    alt:'3.3V out',    cat:'power',  note:'' },
  { pin:'GND',    alt:'',            cat:'gnd',    note:'' },
  { pin:'GPIO15', alt:'reserve',     cat:'strap',  note:'strap ⚠' },
  { pin:'GPIO2',  alt:'Link LED',    cat:'led',    note:'onboard (WiFi+MQTT)' },
  { pin:'GPIO4',  alt:'FLOW',        cat:'sensor', note:'INPUT_PULLUP',   liveKey:'flow' },
  { pin:'GPIO16', alt:'BUTTON',      cat:'button', note:'Ely A/B toggle' },
  { pin:'GPIO17', alt:'reserve',     cat:'gpio',   note:'' },
  { pin:'GPIO5',  alt:'DS18B20',     cat:'sensor', note:'1-wire 4.7k pull-up', liveKey:'sensor_ds18b20_ok' },
  { pin:'GPIO18', alt:'TRANSFORMER', cat:'relay',  note:'auto',           liveKey:'transformer' },
  { pin:'GPIO19', alt:'reserve',     cat:'gpio',   note:'' },
  { pin:'GPIO21', alt:'ELY_B',       cat:'relay',  note:'',               liveKey:'ely_b', cmdKey:'ely_b' },
  { pin:'GPIO3',  alt:'UART0 RX',    cat:'uart',   note:'do not load' },
  { pin:'GPIO1',  alt:'UART0 TX',    cat:'uart',   note:'do not load' },
  { pin:'GPIO22', alt:'ELY_A',       cat:'relay',  note:'',               liveKey:'ely_a', cmdKey:'ely_a' },
  { pin:'GPIO23', alt:'PUMP',        cat:'relay',  note:'',               liveKey:'pump',  cmdKey:'pump' },
];
const PIN_DESC = {
  'PUMP':'Pool-Pumpen-Relais. Active HIGH. Steuerbar per /api/cmd oder MQTT (poolcontroller/cmd/pump).',
  'ELY_A':'Elektrolyse A Relais. Active HIGH. Flow erforderlich; Trafo wird automatisch aktiviert.',
  'ELY_B':'Elektrolyse B Relais. Active HIGH. Flow erforderlich; Trafo wird automatisch aktiviert.',
  'ELY_K':'Elektrolyse Klein Relais. Active HIGH. Flow erforderlich; Trafo wird automatisch aktiviert.',
  'TRANSFORMER':'Trafo-Relais. Automatik: AN sobald eine Elektrolyse gewünscht ist UND Flow vorhanden. AUS nach 10 s Flow-Error.',
  'FLOW':'Flow-Schalter. INPUT_PULLUP, active LOW (LOW = Flow). 200 ms Debounce.',
  'DS18B20':'Pool-Temperatur. 1-Wire (GPIO5). Externer 4.7 kΩ Pull-up zu 3V3 erforderlich. Auto-Rescan alle 10 s bei Sensorverlust.',
  'BUTTON':'Taster zum Toggeln der Ely A/B Phase (0 → A → B → 0). INPUT_PULLUP, active LOW, 50 ms Debounce.',
  'Link LED':'Onboard-LED (GPIO2). Leuchtet nur wenn WiFi UND MQTT verbunden. Strap-Pin – nur als Output verwendet.',
  'LCD MOSI':'ST7920 Display-Daten. HSPI MOSI. 5 V Modul: 3,3 V-Variante nutzen oder Level-Shifter.',
  'LCD SCK':'ST7920 Display-Takt. HSPI CLK.',
  'LCD CS':'ST7920 Chip-Select. Idle HIGH.',
  'I²C SDA':'I²C-Daten zum ADS1115. 4.7 kΩ Pull-up zu 3V3.',
  'I²C SCL':'I²C-Takt zum ADS1115. 4.7 kΩ Pull-up zu 3V3.',
  'UART0 RX':'USB-Seriell-Eingang vom CP2102. Nicht belasten.',
  'UART0 TX':'USB-Seriell-Ausgang zum CP2102. Nicht belasten.',
  'reset':'EN (Reset). Intern auf HIGH; LOW ziehen löst Reset aus.',
  'reserve':'Ungenutzter GPIO. Reserve für Erweiterungen.',
  'VN reserve':'GPIO39 (Sensor VN), nur Eingang, ADC1_CH3. Firmware nutzt ihn nicht.',
  'VP reserve':'GPIO36 (Sensor VP), nur Eingang, ADC1_CH0. Firmware nutzt ihn nicht.',
  '5V in':'VIN. 5 V Eingang von USB oder ext. Netzteil. Speist den internen 3,3 V Regler.',
  '3.3V out':'Geregelter 3,3 V Ausgang. ~600 mA max. Für I²C / 1-Wire Pull-ups nutzen.',
};
const CAT_LABEL = {
  power:'Power', gnd:'GND', en:'EN / Reset', gpio:'GPIO reserve',
  strap:'Strap-Pin', uart:'UART0 (Log)',
  relay:'Relais-Ausgang', sensor:'Sensor', button:'Taster',
  spi:'SPI (LCD)', i2c:'I²C', led:'Onboard-LED',
};
// Pin rows emerge at the top and bottom edges of the PCB (y=130 and y=770).
// Board width 1120 matches the header-strip width.
const PIN_BOARD = { x:140, w:1120, topY:141, botY:629 };
const SVG_NS = 'http://www.w3.org/2000/svg';
let g_pinsByLive = {}, g_pinsByPin = {}, g_usedPills = [];
let g_liveEnabled = true, g_usedOnly = false;

function makePinPill(row, idx, info) {
  const svg = $('#pin-svg');
  const slot = PIN_BOARD.w / 15;
  const x = PIN_BOARD.x + slot/2 + idx*slot;
  const isTop = row === 'top';
  const y0 = isTop ? PIN_BOARD.topY : PIN_BOARD.botY;
  const dir = isTop ? -1 : 1;
  const leadLen = 12, pillW = Math.max(slot - 6, 50), pillH = 30;
  const py = isTop ? (y0 - leadLen - pillH) : (y0 + leadLen);

  const line = document.createElementNS(SVG_NS, 'line');
  line.setAttribute('x1', x); line.setAttribute('y1', y0);
  line.setAttribute('x2', x); line.setAttribute('y2', y0 + dir*leadLen);
  line.setAttribute('class', 'lead'); svg.appendChild(line);

  const dot = document.createElementNS(SVG_NS, 'circle');
  dot.setAttribute('cx', x); dot.setAttribute('cy', y0); dot.setAttribute('r', 2.8);
  dot.setAttribute('class', 'dot'); svg.appendChild(dot);

  const rect = document.createElementNS(SVG_NS, 'rect');
  rect.setAttribute('x', x - pillW/2); rect.setAttribute('y', py);
  rect.setAttribute('width', pillW);   rect.setAttribute('height', pillH);
  rect.setAttribute('rx', 4);
  rect.setAttribute('class', 'pill cat-' + info.cat);
  svg.appendChild(rect);

  const t1 = document.createElementNS(SVG_NS, 'text');
  t1.setAttribute('x', x); t1.setAttribute('y', py + 11);
  t1.setAttribute('class', 'pill-txt'); t1.textContent = info.pin;
  svg.appendChild(t1);
  const t2 = document.createElementNS(SVG_NS, 'text');
  t2.setAttribute('x', x); t2.setAttribute('y', py + 22);
  t2.setAttribute('class', 'pill-sub'); t2.textContent = info.alt || '';
  svg.appendChild(t2);
  if (info.note) {
    const tn = document.createElementNS(SVG_NS, 'text');
    tn.setAttribute('x', x);
    tn.setAttribute('y', isTop ? (py - 4) : (py + pillH + 10));
    tn.setAttribute('class', 'pill-note'); tn.textContent = info.note;
    svg.appendChild(tn);
  }

  rect.addEventListener('mouseenter', (e) => showTip(e, info));
  rect.addEventListener('mousemove',  (e) => moveTip(e));
  rect.addEventListener('mouseleave', hideTip);
  rect.addEventListener('click',      () => onPinClick(info));

  g_pinsByPin[info.pin] = rect;
  if (info.liveKey) (g_pinsByLive[info.liveKey] = g_pinsByLive[info.liveKey] || []).push(rect);
  const used = ['relay','sensor','button','spi','i2c','led'];
  if (used.includes(info.cat)) g_usedPills.push(rect);
}
function showTip(e, info) {
  const tip = $('#pin-tip');
  const desc = PIN_DESC[info.alt] || '';
  tip.innerHTML =
    `<div><b>${info.pin}</b> &nbsp; <span style="color:#94a3b8">${CAT_LABEL[info.cat] || ''}</span></div>` +
    `<div style="margin-top:4px;"><b>${info.alt || '—'}</b></div>` +
    (info.note ? `<div style="color:#94a3b8;margin-top:2px;">${info.note}</div>` : '') +
    (desc ? `<div style="margin-top:6px;color:#cbd5e1;">${desc}</div>` : '');
  tip.hidden = false; moveTip(e);
}
function moveTip(e) {
  const tip = $('#pin-tip');
  const wrap = $('#pin-board-wrap').getBoundingClientRect();
  let x = e.clientX - wrap.left + 14, y = e.clientY - wrap.top + 14;
  const maxX = wrap.width - 280; if (x > maxX) x = maxX;
  tip.style.left = x + 'px'; tip.style.top = y + 'px';
}
function hideTip() { $('#pin-tip').hidden = true; }

async function onPinClick(info) {
  if (info.cmdKey) {
    const pill = g_pinsByPin[info.pin];
    const isOn = pill.classList.contains('live-on');
    try {
      const params = {}; params[info.cmdKey] = isOn ? 'off' : 'on';
      await jpost('/api/cmd', params); refreshStatus();
    } catch (e) { alert('Fehler: ' + e.message); }
    return;
  }
  const card = $('#pin-detail-card');
  $('#pin-detail-title').textContent = info.pin + (info.alt ? ' — ' + info.alt : '');
  const desc = PIN_DESC[info.alt] || PIN_DESC[info.pin] || '—';
  $('#pin-detail-body').innerHTML =
    '<table>' +
    `<tr><td>Kategorie</td><td>${CAT_LABEL[info.cat] || info.cat}</td></tr>` +
    `<tr><td>Funktion</td><td>${info.alt || '—'}</td></tr>` +
    (info.note ? `<tr><td>Hinweis</td><td>${info.note}</td></tr>` : '') +
    `<tr><td>Beschreibung</td><td>${desc}</td></tr>` +
    '</table>';
  card.hidden = false;
  card.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
}
function updatePinoutLive(s) {
  if (!g_liveEnabled) {
    Object.values(g_pinsByLive).flat().forEach(p => p.classList.remove('live-on'));
    return;
  }
  for (const [key, pills] of Object.entries(g_pinsByLive)) {
    const on = !!s[key];
    pills.forEach(p => p.classList.toggle('live-on', on));
  }
}
function applyUsedFilter() {
  const usedSet = new Set(g_usedPills);
  $$('#pin-svg .pill').forEach(p => p.classList.toggle('dimmed', g_usedOnly && !usedSet.has(p)));
}
PIN_TOP.forEach((d, i) => makePinPill('top', i, d));
PIN_BOT.forEach((d, i) => makePinPill('bot', i, d));
$('#pinToggleLive').addEventListener('change', e => { g_liveEnabled = e.target.checked; refreshStatus(); });
$('#pinToggleUsed').addEventListener('change', e => { g_usedOnly    = e.target.checked; applyUsedFilter(); });

// ---------- SVG download (client-side, always reflects current board) ---
// Collect all stylesheet rules whose selector is scoped to #pin-svg so the
// exported file renders standalone with the exact same look as on screen.
function collectPinSvgCss() {
  let css = '';
  for (const sheet of document.styleSheets) {
    let rules;
    try { rules = sheet.cssRules; } catch (_) { continue; } // CORS-protected sheets
    if (!rules) continue;
    for (const r of rules) {
      if (!r.selectorText) continue;
      if (r.selectorText.includes('#pin-svg') ||
          r.selectorText.includes('.pill')    ||
          r.selectorText.includes('@keyframes')) {
        css += r.cssText + '\n';
      }
    }
    // @keyframes live in CSSKeyframesRule without selectorText
    for (const r of rules) {
      if (r.type === CSSRule.KEYFRAMES_RULE) css += r.cssText + '\n';
    }
  }
  return css;
}

function downloadPinoutSvg() {
  const src = document.getElementById('pin-svg');
  if (!src) return;
  const svg = src.cloneNode(true);
  svg.setAttribute('xmlns',       'http://www.w3.org/2000/svg');
  svg.setAttribute('xmlns:xlink', 'http://www.w3.org/1999/xlink');
  // Give it a fixed render size so it opens nicely in image viewers.
  svg.setAttribute('width',  '1400');
  svg.setAttribute('height', '560');

  // Inline the relevant CSS.
  const style = document.createElementNS('http://www.w3.org/2000/svg', 'style');
  style.textContent =
    '/* Exported ' + new Date().toISOString() + ' */\n' +
    'svg{background:radial-gradient(circle at 30% 20%,#0b2540,#020617);}\n' +
    collectPinSvgCss();
  svg.insertBefore(style, svg.firstChild);

  const xml  = new XMLSerializer().serializeToString(svg);
  const blob = new Blob(
    ['<?xml version="1.0" encoding="UTF-8"?>\n', xml],
    { type: 'image/svg+xml;charset=utf-8' }
  );
  const url = URL.createObjectURL(blob);
  const a   = document.createElement('a');
  const ts  = new Date().toISOString().replace(/[:.]/g,'-').slice(0,19);
  a.href     = url;
  a.download = 'ESP32-Pinout_' + ts + '.svg';
  document.body.appendChild(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(url); a.remove(); }, 0);
}
const btnDlSvg = document.getElementById('btnDownloadSvg');
if (btnDlSvg) btnDlSvg.addEventListener('click', downloadPinoutSvg);

// ======================= ZEITPLAN tab =======================
// 7×24 bit grid. Day 0=Sunday..6=Saturday. Cell green = hour enabled.
const DAY_NAMES = ['So','Mo','Di','Mi','Do','Fr','Sa'];
let g_schedule = [0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF];
let g_schedEnabled = false;

function renderScheduleGrid() {
  const head = $('#schedHead');
  const body = $('#schedBody');
  if (!head || !body) return;
  head.innerHTML = '<th style="padding:4px 2px;font-size:11px;color:#9ca3af;"></th>';
  for (let h = 0; h < 24; h++) {
    head.innerHTML += `<th style="padding:4px 2px;font-size:11px;color:#9ca3af;min-width:22px;text-align:center;">${h}</th>`;
  }
  body.innerHTML = '';
  for (let d = 0; d < 7; d++) {
    const tr = document.createElement('tr');
    tr.innerHTML =
      `<td style="padding:4px 6px;font-size:13px;font-weight:600;color:#cbd5f5;white-space:nowrap;">${DAY_NAMES[d]}</td>`;
    for (let h = 0; h < 24; h++) {
      const active = (g_schedule[d] & (1 << h)) !== 0;
      const td = document.createElement('td');
      td.className = 'sched-cell';
      td.dataset.d = d; td.dataset.h = h;
      td.style.cssText = 'padding:2px;text-align:center;cursor:pointer;';
      td.innerHTML =
        `<div style="width:20px;height:20px;border-radius:4px;margin:auto;background:${active ? '#22c55e' : '#334155'};"></div>`;
      td.addEventListener('click', () => {
        const div = td.querySelector('div');
        const nowOn = div.style.background === 'rgb(34, 197, 94)';
        div.style.background = nowOn ? '#334155' : '#22c55e';
        if (nowOn) g_schedule[d] &= ~(1 << h);
        else       g_schedule[d] |=  (1 << h);
      });
      tr.appendChild(td);
    }
    body.appendChild(tr);
  }
}

function setSchedEnableBtn() {
  const btn = $('#schedEnableBtn');
  if (!btn) return;
  btn.textContent = g_schedEnabled ? 'AKTIV' : 'INAKTIV';
  btn.classList.toggle('active', g_schedEnabled);
}

async function loadScheduleUI() {
  try {
    const c = await jget('/api/config');
    g_schedEnabled = !!c.schedule_enabled;
    if (Array.isArray(c.schedule) && c.schedule.length === 7) {
      g_schedule = c.schedule.map(v => (v >>> 0) & 0xFFFFFF);
    }
  } catch (e) { console.warn('sched load', e); }
  renderScheduleGrid();
  setSchedEnableBtn();
}

async function saveSchedule() {
  const params = { schedule_enabled: g_schedEnabled ? 'true' : 'false' };
  for (let d = 0; d < 7; d++) params['schedule_' + d] = (g_schedule[d] >>> 0).toString();
  await jpost('/api/schedule', params);
  const s = $('#schedStatus');
  s.textContent = '✓ Gespeichert';
  setTimeout(() => s.textContent = '', 3000);
}

// Wire up schedule buttons (script is at end of body so DOM is ready)
(() => {
  const enBtn = $('#schedEnableBtn');
  if (enBtn) enBtn.addEventListener('click', () => {
    g_schedEnabled = !g_schedEnabled;
    setSchedEnableBtn();
  });
  const saveBtn = $('#schedSaveBtn');
  if (saveBtn) saveBtn.addEventListener('click', saveSchedule);

  const allOn = $('#schedAllOnBtn');
  if (allOn) allOn.addEventListener('click', () => {
    g_schedule = [0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF,0xFFFFFF];
    renderScheduleGrid();
  });
  const allOff = $('#schedAllOffBtn');
  if (allOff) allOff.addEventListener('click', () => {
    g_schedule = [0,0,0,0,0,0,0];
    renderScheduleGrid();
  });
})();

// ======================= DISPLAY MIRROR =================================
// Fetches the virtual display recorder's op list from /api/lcd.json and
// replays it onto a blue Canvas. No framebuffer layout guesswork — every
// draw call on the ESP32 is mirrored as a typed op (text or box).
(() => {
  const canvas = document.getElementById('lcdCanvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  // Blue LCD palette --------------------------------------------------
  const BG = '#0a1c42';
  const FG = '#88c6ff';

  // Render scale factor: the logical LCD dims (e.g. 64x128) are tiny, so
  // we blow the canvas up internally so text renders crisp without CSS
  // smoothing of a sub-pixel bitmap. SCALE=2 already yields a 128x256 px
  // canvas — big enough for a readable monospace render but 4x cheaper to
  // repaint than SCALE=4 (important because a CSS drop-shadow filter +
  // image-rendering:pixelated on the canvas is costly on mobile GPUs).
  const SCALE = 2;

  // Cached last frame so rotation/flip/zoom can re-render without refetch.
  let lastFrame = null;

  function renderOps(data) {
    const W = (data.w || 64) * SCALE;
    const H = (data.h || 128) * SCALE;
    if (canvas.width !== W || canvas.height !== H) {
      canvas.width = W; canvas.height = H;
    }
    ctx.imageSmoothingEnabled = false;

    // Background
    ctx.fillStyle = BG;
    ctx.fillRect(0, 0, W, H);

    const cell = (data.cell || 7) * SCALE;    // glyph advance
    const line = (data.line || 13);           // glyph / box reference height
    // Font sized to the line height. Bold monospace approximates u8g2 7x13B.
    ctx.font = `bold ${line * SCALE}px ui-monospace, "SF Mono", "Cascadia Mono", Menlo, Consolas, monospace`;
    ctx.textBaseline = 'alphabetic';
    ctx.textAlign    = 'left';

    const ops = data.ops || [];
    for (const op of ops) {
      const col = op.i ? BG : FG;
      if (op.t === 'b') {
        ctx.fillStyle = col;
        ctx.fillRect(op.x * SCALE, op.y * SCALE, op.w * SCALE, op.h * SCALE);
      } else if (op.t === 't') {
        ctx.fillStyle = col;
        const s = op.s || '';
        // Force fixed cell width so the rendering matches u8g2's monospace
        // placement regardless of the browser's native glyph advance.
        for (let i = 0; i < s.length; i++) {
          ctx.fillText(s[i], op.x * SCALE + i * cell, op.y * SCALE);
        }
      }
    }
    applyScale();
  }

  function updateDebugPanel(data) {
    const dbg = document.getElementById('lcdDebug');
    if (!dbg) return;
    const ops = data.ops || [];
    const samples = ops.slice(0, 5).map((o, i) => {
      if (o.t === 't') return `  [${i}] text @(${o.x},${o.y}) "${o.s}"${o.i?' inv':''}`;
      if (o.t === 'b') return `  [${i}] box  @(${o.x},${o.y}) ${o.w}x${o.h}${o.i?' inv':''}`;
      return `  [${i}] ?`;
    }).join('\n');
    dbg.textContent =
      `w=${data.w}  h=${data.h}  font=${data.font}  frame=${data.frame}  ops=${ops.length}\n` +
      samples;
  }

  let busy = false;
  async function refresh(silent) {
    if (busy) return;
    busy = true;
    const st = document.getElementById('lcdStatus');
    try {
      const r = await fetch('/api/lcd.json', { cache: 'no-store' });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      const data = await r.json();
      lastFrame = data;
      renderOps(data);
      updateDebugPanel(data);
      if (st && !silent) {
        st.style.color = '#22c55e';
        st.textContent = `OK · ${data.ops ? data.ops.length : 0} ops · ` +
                         new Date().toLocaleTimeString();
      }
    } catch (e) {
      if (st) { st.style.color = '#ef4444'; st.textContent = 'Fehler: ' + e.message; }
    } finally {
      busy = false;
    }
  }

  // Test pattern: drives the renderer with a synthetic op list. If this
  // looks like a proper screen (labels, box, "ON"), the client pipeline
  // works and real frames pointing problem at the ESP side.
  function loadTestPattern() {
    const data = {
      w: 64, h: 128, font: '7x13B', cell: 7, line: 13, frame: 0,
      ops: [
        { t: 't', x: 0,  y: 12, s: 'PUMP' },
        { t: 'b', x: 41, y: 1,  w: 28, h: 13 },
        { t: 't', x: 45, y: 12, s: 'ON', i: 1 },
        { t: 't', x: 0,  y: 26, s: 'EL A' },
        { t: 't', x: 41, y: 26, s: 'OFF' },
        { t: 't', x: 0,  y: 40, s: 'H2O 23.4' },
        { t: 't', x: 0,  y: 54, s: 'ORP  712' },
      ],
    };
    lastFrame = data;
    renderOps(data);
    updateDebugPanel(data);
    const st = document.getElementById('lcdStatus');
    if (st) { st.style.color = '#38bdf8'; st.textContent = 'Testmuster geladen.'; }
  }

  let timer = null;
  function isActive() {
    const t = document.getElementById('tab-display');
    return t && t.classList.contains('active');
  }
  function startPolling() {
    if (timer) return;
    refresh(true);
    // Matches the firmware's DISPLAY_REFRESH_INTERVAL (200 ms) so the
    // row-9 quote ticker scrolls smoothly. Only active while the display
    // tab is in the foreground, so idle cost is zero.
    timer = setInterval(() => { if (isActive()) refresh(true); }, 200);
  }
  function stopPolling() { if (timer) { clearInterval(timer); timer = null; } }

  const mo = new MutationObserver(() => {
    if (isActive()) startPolling(); else stopPolling();
  });
  const tabEl = document.getElementById('tab-display');
  if (tabEl) mo.observe(tabEl, { attributes: true, attributeFilter: ['class'] });

  // Controls ------------------------------------------------------------
  const auto = document.getElementById('lcdToggleAuto');
  if (auto) auto.addEventListener('change', () => {
    if (auto.checked && isActive()) startPolling(); else stopPolling();
  });
  const btn = document.getElementById('btnLcdRefresh');
  if (btn) btn.addEventListener('click', () => refresh(false));
  const btnTest = document.getElementById('btnLcdTest');
  if (btnTest) btnTest.addEventListener('click', () => {
    // Stop auto-refresh so the test pattern sticks until the user refreshes.
    const a = document.getElementById('lcdToggleAuto');
    if (a && a.checked) { a.checked = false; stopPolling(); }
    loadTestPattern();
  });

  // (Rotation / flip are no longer needed — the virtual display recorder
  //  emits ops directly in u8g2's logical coordinate system, so the web
  //  canvas always matches the physical LCD orientation 1:1.)

  // Zoom slider: s = displayed pixels per logical LCD pixel. Canvas is
  // rendered internally at SCALE× for crisp text, so a zoom of SCALE means
  // 1:1 CSS:internal (no browser resampling). Lower zoom => browser scales
  // down, higher => image-rendering:pixelated keeps nearest-neighbour.
  const scale = document.getElementById('lcdScale');
  const lbl   = document.getElementById('lcdScaleVal');
  function applyScale() {
    if (document.body.classList.contains('display-fullscreen')) {
      applyFullscreenFit();
      return;
    }
    const s = Number(scale.value) || 4;
    const logicalW = canvas.width  / SCALE;   // e.g. 64
    const logicalH = canvas.height / SCALE;   // e.g. 128
    canvas.style.width  = (logicalW * s) + 'px';
    canvas.style.height = (logicalH * s) + 'px';
    if (lbl) lbl.textContent = s + '×';
  }
  if (scale) {
    scale.addEventListener('input', applyScale);
    applyScale();
  }

  // ---------- Smartphone fullscreen mode ----------------------------
  // On narrow viewports the Display tab becomes a fullscreen mirror:
  // - chrome (header, tabs, other cards) is hidden via body.display-fullscreen
  // - canvas is scaled to fit viewport while keeping aspect ratio
  // - tapping the bezel navigates back to the Status tab
  const MOBILE_MQ = window.matchMedia('(max-width: 768px)');

  function applyFullscreenFit() {
    const vw = window.innerWidth  * 0.92;
    const vh = window.innerHeight * 0.84;  // leave room for the hint pill
    const ar = canvas.width / canvas.height;
    let w, h;
    if (vw / vh > ar) { h = vh; w = h * ar; }
    else              { w = vw; h = w / ar; }
    canvas.style.width  = Math.floor(w) + 'px';
    canvas.style.height = Math.floor(h) + 'px';
  }

  function updateFullscreenState() {
    const shouldFull = isActive() && MOBILE_MQ.matches;
    const hasFull    = document.body.classList.contains('display-fullscreen');
    if (shouldFull && !hasFull) {
      document.body.classList.add('display-fullscreen');
      applyFullscreenFit();
    } else if (!shouldFull && hasFull) {
      document.body.classList.remove('display-fullscreen');
      applyScale();  // restore zoom-slider scale
    } else if (shouldFull) {
      applyFullscreenFit();
    }
  }

  // Re-evaluate whenever the tab opens/closes or the viewport changes.
  const mo2 = new MutationObserver(updateFullscreenState);
  if (tabEl) mo2.observe(tabEl, { attributes: true, attributeFilter: ['class'] });
  MOBILE_MQ.addEventListener('change', updateFullscreenState);
  window.addEventListener('resize', () => {
    if (document.body.classList.contains('display-fullscreen')) applyFullscreenFit();
  });
  // Initial check in case the page loads directly on #tab-display (unlikely
  // but harmless).
  updateFullscreenState();

  // Exit fullscreen back to the Status tab.
  // Strategy: a global `document`-level capture listener catches every tap
  // first (so nothing on top of the canvas can swallow it), and a visible
  // floating Close button gives the user an unambiguous target.
  const exitFs = () => {
    if (!document.body.classList.contains('display-fullscreen')) return;
    const statusBtn = document.querySelector('.tab[data-tab="status"]');
    if (statusBtn) statusBtn.click();
  };
  const guard = (e) => {
    if (!document.body.classList.contains('display-fullscreen')) return;
    // Don't hijack interactions with form controls inside the LCD card
    // (e.g. zoom slider, refresh button) - although in fullscreen mode CSS
    // hides the .lcd-controls block anyway.
    const t = e.target;
    if (t && t.closest && t.closest('.lcd-controls,button,input,select,a')) return;
    e.preventDefault();
    exitFs();
  };
  // Capture phase so we win against any other listener on the page.
  document.addEventListener('pointerdown', guard, true);
  document.addEventListener('touchstart',  guard, { capture: true, passive: false });
  document.addEventListener('click',       guard, true);

  if (canvas) canvas.style.cursor = 'pointer';
})();

// ======================= HISTORY CHART ==================================
// Fetches /api/history (ring-buffer snapshot from LittleFS /hist.bin) and
// renders pool temperature + ORP over the selected time window on a single
// Canvas. Kept dependency-free (no Chart.js) so the UI stays well under
// the LittleFS budget and loads instantly over WiFi.
(() => {
  const canvas = document.getElementById('histChart');
  if (!canvas) return;
  const ctx     = canvas.getContext('2d');
  const tooltip = document.getElementById('histTooltip');
  const windowSel = document.getElementById('histWindowMins');
  const heightSl  = document.getElementById('histHeight');
  const heightLbl = document.getElementById('histHeightVal');
  const statusEl  = document.getElementById('histStatus');
  const refreshBtn= document.getElementById('btnHistRefresh');

  let lastData = null;   // last /api/history response
  let pixelMap = [];     // [{x,y,p}] for tooltip hit-test
  let busy = false;

  // Manual chlorine + pH measurements (entered by the user in the form below)
  let manualData = { chlorine: [], ph: [] };

  function fmtClock(epoch) {
    if (!epoch) return '?';
    const d = new Date(epoch * 1000);
    return d.toLocaleString('de-DE', { hour12: false });
  }

  function filterByWindow(points, mins, nowEpoch) {
    if (!points) return [];
    // dt_sec is <=0 (seconds ago). Keep samples within the window.
    const cutoff = -mins * 60;
    return points.filter(p => p.dt_sec >= cutoff && p.t !== null);
  }

  function render() {
    if (!lastData) return;
    const mins = parseInt(windowSel.value, 10) || 1440;
    const pts  = filterByWindow(lastData.points, mins, lastData.now_epoch);

    const dpr  = window.devicePixelRatio || 1;
    const cssW = canvas.clientWidth;
    const cssH = parseInt(heightSl.value, 10) || 360;
    canvas.style.height = cssH + 'px';
    canvas.width  = cssW * dpr;
    canvas.height = cssH * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cssW, cssH);

    // Background gradient
    const bg = ctx.createLinearGradient(0, 0, 0, cssH);
    bg.addColorStop(0, '#0b1733');
    bg.addColorStop(1, '#020617');
    ctx.fillStyle = bg;
    ctx.fillRect(0, 0, cssW, cssH);

    if (!pts.length) {
      ctx.fillStyle = '#64748b';
      ctx.font = '13px -apple-system, system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Keine Messwerte im gewählten Zeitraum.', cssW/2, cssH/2);
      pixelMap = [];
      if (statusEl) statusEl.textContent =
        `0 Punkte · Ringpuffer ${lastData.capacity || 4320} Slots`;
      return;
    }

    const padL = 52, padR = 52, padT = 18, padB = 34;
    const iw = cssW - padL - padR;
    const ih = cssH - padT - padB;

    // Y-axis ranges
    let tMin = Infinity, tMax = -Infinity;
    let oMin = Infinity, oMax = -Infinity;
    for (const p of pts) {
      if (typeof p.t === 'number') { if (p.t < tMin) tMin = p.t; if (p.t > tMax) tMax = p.t; }
      if (typeof p.orp === 'number') { if (p.orp < oMin) oMin = p.orp; if (p.orp > oMax) oMax = p.orp; }
    }
    if (!isFinite(tMin)) { tMin = 0; tMax = 30; }
    if (tMax - tMin < 1) { tMax += 0.5; tMin -= 0.5; }
    const tPad = (tMax - tMin) * 0.1;
    tMin -= tPad; tMax += tPad;
    const hasOrp = isFinite(oMin);
    if (hasOrp) {
      if (oMax - oMin < 10) { oMax += 5; oMin -= 5; }
      const oPad = (oMax - oMin) * 0.1;
      oMin -= oPad; oMax += oPad;
    }

    // X range = selected window (not just min/max of points)
    const xMin = -mins * 60;
    const xMax = 0;

    const xFor = dt => padL + ((dt - xMin) / (xMax - xMin)) * iw;
    const yForT = t => padT + ih - ((t - tMin) / (tMax - tMin)) * ih;
    const yForO = o => padT + ih - ((o - oMin) / (oMax - oMin)) * ih;

    // Grid + Y labels
    ctx.strokeStyle = 'rgba(148,163,184,0.15)';
    ctx.lineWidth = 1;
    ctx.fillStyle = '#94a3b8';
    ctx.font = '11px -apple-system, system-ui, sans-serif';
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    const yTicks = 5;
    for (let i = 0; i <= yTicks; i++) {
      const y = padT + (ih * i) / yTicks;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(cssW - padR, y); ctx.stroke();
      const tv = tMax - ((tMax - tMin) * i) / yTicks;
      ctx.fillStyle = '#60a5fa';
      ctx.fillText(tv.toFixed(1) + '°', padL - 6, y);
      if (hasOrp) {
        const ov = oMax - ((oMax - oMin) * i) / yTicks;
        ctx.fillStyle = '#f59e0b';
        ctx.textAlign = 'left';
        ctx.fillText(Math.round(ov) + ' mV', cssW - padR + 6, y);
        ctx.textAlign = 'right';
      }
    }

    // X labels: relative time bands. Switch to days when the window is
    // long enough, otherwise show hours (or minutes for tight views).
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    ctx.fillStyle = '#64748b';
    const hours = mins / 60;
    const useDays = hours > 48;
    const stepH  = hours <= 2 ? 0.25 :
                   hours <= 12 ? 1 :
                   hours <= 48 ? 4 :
                   hours <= 72 ? 12 :
                   24;
    for (let h = 0; h <= hours; h += stepH) {
      const dt = -h * 3600;
      const x  = xFor(dt);
      ctx.beginPath();
      ctx.strokeStyle = 'rgba(148,163,184,0.08)';
      ctx.moveTo(x, padT); ctx.lineTo(x, padT + ih); ctx.stroke();
      let lbl;
      if (h < 1)          lbl = Math.round(h * 60) + ' m';
      else if (useDays)   lbl = '-' + (h / 24).toFixed(h % 24 === 0 ? 0 : 1) + ' d';
      else                lbl = '-' + h + ' h';
      ctx.fillText(lbl, x, padT + ih + 6);
    }

    // Polyline: ORP first (behind), then temperature on top
    if (hasOrp) {
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth   = 1.5;
      ctx.beginPath();
      let moved = false;
      for (const p of pts) {
        if (typeof p.orp !== 'number') { moved = false; continue; }
        const x = xFor(p.dt_sec), y = yForO(p.orp);
        if (!moved) { ctx.moveTo(x, y); moved = true; }
        else        ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    ctx.strokeStyle = '#60a5fa';
    ctx.lineWidth   = 2;
    ctx.beginPath();
    let moved = false;
    pixelMap = [];
    for (const p of pts) {
      if (typeof p.t !== 'number') { moved = false; continue; }
      const x = xFor(p.dt_sec), y = yForT(p.t);
      if (!moved) { ctx.moveTo(x, y); moved = true; }
      else        ctx.lineTo(x, y);
      pixelMap.push({ x, y, p });
    }
    ctx.stroke();

    // ----- Manual Chlor + pH overlays (linear interpolation) -------------
    // Each series gets its own normalised lane on the right side so they
    // never collide with temp/ORP units. We expose the value range in the
    // legend so the user always knows what scale they're reading.
    function drawManualSeries(entries, color, label, unit, fmt) {
      if (!entries || entries.length === 0) return null;
      // Filter to entries inside the visible time window (with one-element
      // overhang on each side so the line still extends to the chart edge).
      const winStart = (lastData.now_epoch || 0) + xMin;
      const winEnd   = (lastData.now_epoch || 0) + xMax;
      const inWin = [];
      let prev = null, next = null;
      for (const e of entries) {
        if (e.epoch < winStart) prev = e;
        else if (e.epoch > winEnd) { if (!next) next = e; }
        else inWin.push(e);
      }
      const series = [];
      if (prev) series.push(prev);
      series.push(...inWin);
      if (next) series.push(next);
      if (series.length === 0) return null;

      // Per-series y-range (so a slowly drifting pH still looks visible).
      let lo = Infinity, hi = -Infinity;
      for (const e of series) { if (e.value < lo) lo = e.value; if (e.value > hi) hi = e.value; }
      if (hi - lo < 0.1) { hi += 0.05; lo -= 0.05; }
      const pad = (hi - lo) * 0.2;
      lo -= pad; hi += pad;
      const yFor = v => padT + ih - ((v - lo) / (hi - lo)) * ih;

      ctx.strokeStyle = color;
      ctx.fillStyle   = color;
      ctx.lineWidth   = 2;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      let moved = false;
      for (const e of series) {
        const dt = e.epoch - lastData.now_epoch;
        const xv = Math.max(xMin, Math.min(xMax, dt));
        const x  = xFor(xv);
        const y  = yFor(e.value);
        if (!moved) { ctx.moveTo(x, y); moved = true; }
        else        ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
      // Markers on the *real* (unclipped) entries inside the window.
      for (const e of inWin) {
        const dt = e.epoch - lastData.now_epoch;
        const x  = xFor(dt);
        const y  = yFor(e.value);
        ctx.beginPath(); ctx.arc(x, y, 3.5, 0, Math.PI * 2); ctx.fill();
      }
      return { lo, hi, count: inWin.length, last: series[series.length - 1] };
    }

    const clInfo = drawManualSeries(manualData.chlorine, '#34d399',
                                    'Chlor', 'mg/L', v => v.toFixed(2));
    const phInfo = drawManualSeries(manualData.ph,       '#fbbf24',
                                    'pH',    '',     v => v.toFixed(2));

    // Legend
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.font = 'bold 12px -apple-system, system-ui, sans-serif';
    let legX = padL;
    ctx.fillStyle = '#60a5fa'; ctx.fillText('■ Pool-Temp', legX, padT - 4); legX += 110;
    if (hasOrp) { ctx.fillStyle = '#f59e0b'; ctx.fillText('■ ORP', legX, padT - 4); legX += 70; }
    if (clInfo) {
      ctx.fillStyle = '#34d399';
      ctx.fillText(`▦ Chlor (${clInfo.lo.toFixed(1)}-${clInfo.hi.toFixed(1)} mg/L)`, legX, padT - 4);
      legX += 200;
    }
    if (phInfo) {
      ctx.fillStyle = '#fbbf24';
      ctx.fillText(`▦ pH (${phInfo.lo.toFixed(1)}-${phInfo.hi.toFixed(1)})`, legX, padT - 4);
    }

    const lastPt = pts[pts.length - 1];
    const firstEpoch = lastData.now_epoch + (pts[0].dt_sec || 0);
    if (statusEl) {
      statusEl.textContent =
        `${pts.length} Punkte · letzter Wert ${lastPt.t.toFixed(2)} °C` +
        (lastData.now_epoch ? ` um ${fmtClock(lastData.now_epoch + lastPt.dt_sec)}` : '') +
        ` · Ringpuffer ${lastData.capacity || 4320} Slots`;
    }
  }

  // Tooltip on mousemove
  canvas.addEventListener('mousemove', (ev) => {
    if (!pixelMap.length || !tooltip) { if (tooltip) tooltip.style.display = 'none'; return; }
    const rect = canvas.getBoundingClientRect();
    const mx = ev.clientX - rect.left;
    const my = ev.clientY - rect.top;
    let best = null, bestD = 1e9;
    for (const m of pixelMap) {
      const d = Math.hypot(m.x - mx, m.y - my);
      if (d < bestD) { bestD = d; best = m; }
    }
    if (!best || bestD > 30) { tooltip.style.display = 'none'; return; }
    const p = best.p;
    const when = lastData.now_epoch
      ? fmtClock(lastData.now_epoch + p.dt_sec)
      : (p.dt_sec + ' s');
    tooltip.innerHTML =
      '<b>' + when + '</b><br>' +
      (typeof p.t === 'number' ? 'Temp: <b>' + p.t.toFixed(1) + ' °C</b><br>' : '') +
      (typeof p.orp === 'number' ? 'ORP: <b>' + p.orp + ' mV</b>' : '');
    tooltip.style.display = 'block';
    tooltip.style.left = Math.min(best.x + 14, canvas.clientWidth - 180) + 'px';
    tooltip.style.top  = Math.max(best.y - 40, 4) + 'px';
  });
  canvas.addEventListener('mouseleave', () => { if (tooltip) tooltip.style.display = 'none'; });

  function fmtDateTime(epoch) {
    const d = new Date(epoch * 1000);
    return d.toLocaleString('de-DE', { hour12: false });
  }

  function renderManualLists() {
    const elCl = document.getElementById('manualClList');
    const elPh = document.getElementById('manualPhList');
    function rowHtml(kind, e, unit) {
      return '<div style="display:flex;justify-content:space-between;gap:8px;padding:4px 6px;border-bottom:1px solid rgba(51,65,85,0.4);">' +
        '<span>' + fmtDateTime(e.epoch) + '</span>' +
        '<span><b>' + e.value.toFixed(2) + '</b> ' + unit + '</span>' +
        '<button data-kind="' + kind + '" data-epoch="' + e.epoch + '" type="button" ' +
        'class="manual-del" style="background:transparent;border:none;color:#ef4444;cursor:pointer;font-size:14px;padding:0 4px;" ' +
        'title="L\u00f6schen">\u2715</button>' +
        '</div>';
    }
    if (elCl) elCl.innerHTML = manualData.chlorine.length
      ? manualData.chlorine.slice().reverse().map(e => rowHtml('cl', e, 'mg/L')).join('')
      : '<div style="color:#64748b;padding:6px 0;">Noch keine Eintr\u00e4ge.</div>';
    if (elPh) elPh.innerHTML = manualData.ph.length
      ? manualData.ph.slice().reverse().map(e => rowHtml('ph', e, '')).join('')
      : '<div style="color:#64748b;padding:6px 0;">Noch keine Eintr\u00e4ge.</div>';
    document.querySelectorAll('.manual-del').forEach(btn => {
      btn.addEventListener('click', async () => {
        const k = btn.dataset.kind, ep = btn.dataset.epoch;
        if (!confirm('Eintrag wirklich l\u00f6schen?')) return;
        try {
          const r = await fetch('/api/manual?kind=' + k + '&epoch=' + ep, { method: 'DELETE' });
          if (!r.ok) throw new Error('HTTP ' + r.status);
          await refresh();
        } catch (e) { alert('L\u00f6schen fehlgeschlagen: ' + e.message); }
      });
    });
  }

  async function fetchManual() {
    try {
      const r = await fetch('/api/manual', { cache: 'no-store' });
      if (r.ok) manualData = await r.json();
    } catch (_) { /* keep old */ }
  }

  async function refresh() {
    if (busy) return;
    busy = true;
    if (statusEl) statusEl.textContent = 'Lade Verlauf …';
    try {
      const [rh] = await Promise.all([
        fetch('/api/history', { cache: 'no-store' }),
        fetchManual(),
      ]);
      if (!rh.ok) throw new Error('HTTP ' + rh.status);
      lastData = await rh.json();
      render();
      renderManualLists();
    } catch (e) {
      if (statusEl) { statusEl.style.color = '#ef4444'; statusEl.textContent = 'Fehler: ' + e.message; }
    } finally { busy = false; }
  }

  // ---- Manual entry: chlorine and pH are submitted independently --------
  function setNowDefault(el) {
    if (!el) return;
    const d = new Date();
    d.setMinutes(d.getMinutes() - d.getTimezoneOffset());
    el.value = d.toISOString().slice(0, 16);
  }
  setNowDefault(document.getElementById('manualClWhen'));
  setNowDefault(document.getElementById('manualPhWhen'));

  async function postEntry(kind, value, epoch) {
    const r = await fetch('/api/manual', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ kind, value, epoch }),
    });
    if (!r.ok) throw new Error('HTTP ' + r.status + ': ' + (await r.text()));
  }

  function wireManualSend(opts) {
    const btn  = document.getElementById(opts.btnId);
    const inp  = document.getElementById(opts.inpId);
    const when = document.getElementById(opts.whenId);
    const st   = document.getElementById(opts.stId);
    if (!btn) return;
    btn.addEventListener('click', async () => {
      const v = parseFloat(inp.value);
      if (isNaN(v)) {
        if (st) { st.style.color = '#ef4444'; st.textContent = 'Wert eingeben.'; }
        return;
      }
      let epoch = 0;
      if (when && when.value) epoch = Math.floor(new Date(when.value).getTime() / 1000);
      if (st) { st.style.color = '#94a3b8'; st.textContent = 'Sende …'; }
      try {
        await postEntry(opts.kind, v, epoch);
        if (st) { st.style.color = '#10b981'; st.textContent = 'Gesendet.'; }
        inp.value = '';
        setNowDefault(when);
        await refresh();
      } catch (e) {
        if (st) { st.style.color = '#ef4444'; st.textContent = 'Fehler: ' + e.message; }
      }
    });
  }
  wireManualSend({ kind:'cl', btnId:'btnManualSendCl', inpId:'manualCl', whenId:'manualClWhen', stId:'manualClStatus' });
  wireManualSend({ kind:'ph', btnId:'btnManualSendPh', inpId:'manualPh', whenId:'manualPhWhen', stId:'manualPhStatus' });

  windowSel.addEventListener('change', render);
  heightSl.addEventListener('input', () => {
    heightLbl.textContent = heightSl.value + ' px';
    render();
  });
  if (refreshBtn) refreshBtn.addEventListener('click', refresh);

  // Refresh when the Verlauf tab becomes active.
  const tabEl = document.getElementById('tab-verlauf');
  if (tabEl) {
    const mo = new MutationObserver(() => {
      if (tabEl.classList.contains('active')) refresh();
    });
    mo.observe(tabEl, { attributes: true, attributeFilter: ['class'] });
  }

  // Auto-refresh every 15 min while the tab is visible, matching the
  // backend's sample cadence.
  setInterval(() => {
    if (tabEl && tabEl.classList.contains('active')) refresh();
  }, 15 * 60 * 1000);

  window.addEventListener('resize', () => { if (lastData) render(); });
})();

// ---------- Kick off ----------
loadConfig();
refreshStatus();   // one-shot GET so the UI paints immediately on load

// ---------- SSE live stream ----------
// The ESP32 pushes named "status" events whenever state changes, plus a
// 2 s heartbeat. EventSource handles reconnect + backoff automatically.
// Fallback: if SSE fails (old cached bundle, proxy strips text/event-stream),
// we fall back to polling every 2 s like before.
let _sseFallbackTimer = null;
function startSse() {
  let es;
  try { es = new EventSource('/events'); }
  catch (e) { console.warn('SSE unavailable, polling', e); _sseFallbackTimer = setInterval(refreshStatus, 2000); return; }

  es.addEventListener('status', (e) => {
    try { applyStatus(JSON.parse(e.data)); }
    catch (err) { console.warn('bad SSE status', err); }
  });
  es.addEventListener('error', () => {
    // EventSource auto-reconnects; show a soft indicator while down.
    $('#metaMqttState').textContent = 'Verbindung verloren';
  });
  es.addEventListener('open', () => {
    // stop any fallback polling that may have started before
    if (_sseFallbackTimer) { clearInterval(_sseFallbackTimer); _sseFallbackTimer = null; }
  });
}
startSse();
