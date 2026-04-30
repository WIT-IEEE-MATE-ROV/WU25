// WebSocket connection
const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws/telemetry');
const connEl = document.getElementById('conn-status');

ws.onopen  = () => { connEl.textContent = '● Connected';    connEl.className = 'badge connected'; };
ws.onclose = () => { connEl.textContent = '● Disconnected'; connEl.className = 'badge disconnected'; };
ws.onerror = () => { connEl.textContent = '● Error';        connEl.className = 'badge disconnected'; };

// ── Thruster bars ──────────────────────────────────────────────────
const THRUSTER_NAMES = ['FLH', 'FRH', 'BLH', 'BRH', 'FLV', 'FRV', 'BLV', 'BRV'];
const barsContainer = document.getElementById('thruster-bars');

THRUSTER_NAMES.forEach((name, i) => {
  const div = document.createElement('div');
  div.className = 'tbar';
  div.id = `tbar-${i}`;
  div.innerHTML =
    `<span class="tbar-label">${name}</span>` +
    `<div class="tbar-track">` +
      `<div class="tbar-center-line"></div>` +
      `<div class="tbar-fill pos" id="tpos-${i}" style="width:0%"></div>` +
      `<div class="tbar-fill neg" id="tneg-${i}" style="width:0%"></div>` +
    `</div>` +
    `<span class="tbar-val" id="tval-${i}">0.00</span>`;
  barsContainer.appendChild(div);
});

function updateThrusterBar(i, value) {
  const v = Math.max(-1, Math.min(1, value));
  const pct = Math.abs(v) * 50;
  document.getElementById(`tpos-${i}`).style.width = (v >= 0 ? pct : 0) + '%';
  document.getElementById(`tneg-${i}`).style.width = (v < 0  ? pct : 0) + '%';
  document.getElementById(`tval-${i}`).textContent = v.toFixed(2);
}

// ── IMU quaternion → Euler ────────────────────────────────────────
function quatToEulerDeg(w, x, y, z) {
  const sinr = 2 * (w * x + y * z);
  const cosr = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinr, cosr) * 180 / Math.PI;

  const sinp = 2 * (w * y - z * x);
  const pitch = (Math.abs(sinp) >= 1)
    ? Math.sign(sinp) * 90
    : Math.asin(sinp) * 180 / Math.PI;

  const siny = 2 * (w * z + x * y);
  const cosy = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny, cosy) * 180 / Math.PI;

  return { roll, pitch, yaw };
}

// ── Control mode indicators ────────────────────────────────────────
function setLed(id, active) {
  const el = document.getElementById(id);
  if (el) el.className = 'led' + (active ? ' active' : '');
}

// ── WebSocket message handler ──────────────────────────────────────
ws.onmessage = (ev) => {
  let data;
  try { data = JSON.parse(ev.data); } catch { return; }

  if (data.topic === 'thrusters/output' && Array.isArray(data.thrusters)) {
    data.thrusters.forEach((v, i) => { if (i < 8) updateThrusterBar(i, v); });
  }

  if (data.topic === 'thrusters/control_mode') {
    setLed('led-rot',    !!data.hold_rotation);
    setLed('led-depth',  !!data.hold_depth);
    setLed('led-angvel', !!data.ang_vel_control);
    setLed('led-deplock',!!data.depth_lock);
  }

  if (data.topic === 'bno/quat' && data.quat) {
    const { w, x, y, z } = data.quat;
    const { roll, pitch, yaw } = quatToEulerDeg(w, x, y, z);
    document.getElementById('imu-roll').textContent  = roll.toFixed(1);
    document.getElementById('imu-pitch').textContent = pitch.toFixed(1);
    document.getElementById('imu-yaw').textContent   = yaw.toFixed(1);
  }
};

// ── PID sliders ────────────────────────────────────────────────────
function bindSlider(id, valId, decimals = 2) {
  const input = document.getElementById(id);
  const label = document.getElementById(valId);
  input.addEventListener('input', () => {
    label.textContent = parseFloat(input.value).toFixed(decimals);
  });
}

bindSlider('rot-kp',   'rot-kp-val');
bindSlider('rot-ki',   'rot-ki-val');
bindSlider('rot-kd',   'rot-kd-val');
bindSlider('rot-max',  'rot-max-val');
bindSlider('depth-kp', 'depth-kp-val');
bindSlider('depth-ki', 'depth-ki-val');
bindSlider('depth-kd', 'depth-kd-val');

async function post(path, body) {
  const opts = { method: 'POST' };
  if (body) {
    opts.headers = { 'Content-Type': 'application/json' };
    opts.body = JSON.stringify(body);
  }
  const r = await fetch(path, opts);
  return r.json();
}

function showPidStatus(msg, ok) {
  const el = document.getElementById('pid-status');
  el.textContent = msg;
  el.style.color = ok ? '#81c784' : '#e57373';
  setTimeout(() => { el.textContent = ''; }, 3000);
}

document.getElementById('apply-rot-pid').onclick = async () => {
  const params = {
    rot_kp:        parseFloat(document.getElementById('rot-kp').value),
    rot_ki:        parseFloat(document.getElementById('rot-ki').value),
    rot_kd:        parseFloat(document.getElementById('rot-kd').value),
    rot_max_output:parseFloat(document.getElementById('rot-max').value),
  };
  try {
    const r = await post('/api/pid', params);
    showPidStatus(r.ok ? '✓ Rotation PID applied' : ('Error: ' + r.error), r.ok);
  } catch { showPidStatus('Network error', false); }
};

document.getElementById('apply-depth-pid').onclick = async () => {
  const params = {
    depth_kp: parseFloat(document.getElementById('depth-kp').value),
    depth_ki: parseFloat(document.getElementById('depth-ki').value),
    depth_kd: parseFloat(document.getElementById('depth-kd').value),
  };
  try {
    const r = await post('/api/pid', params);
    showPidStatus(r.ok ? '✓ Depth PID applied' : ('Error: ' + r.error), r.ok);
  } catch { showPidStatus('Network error', false); }
};

// ── Node control buttons ───────────────────────────────────────────
document.getElementById('start-rovs').onclick = async () => {
  const r = await post('/api/orangepi/start_nodes');
  alert(r.ok ? 'OrangePi nodes started' : 'Error: ' + r.error);
};
document.getElementById('stop-rovs').onclick = async () => {
  const r = await post('/api/orangepi/stop_nodes');
  alert(r.ok ? 'OrangePi nodes stopped' : 'Error: ' + r.error);
};
document.getElementById('start-joy').onclick = async () => {
  const r = await post('/api/windows/start_joy');
  alert(r.ok ? 'Joy node started' : 'Error: ' + r.error);
};
document.getElementById('stop-joy').onclick = async () => {
  const r = await post('/api/windows/stop_joy');
  alert(r.ok ? 'Joy node stopped' : 'Error: ' + r.error);
};
