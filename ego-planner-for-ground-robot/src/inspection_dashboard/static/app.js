// Substation Inspection · Operator Console — frontend
const sock = io({ transports: ['websocket', 'polling'] });

const $ = (id) => document.getElementById(id);
const fmt = (n, d = 2) => (typeof n === 'number' && Number.isFinite(n) ? n.toFixed(d) : '—');
const tsStr = (t) => {
  const d = new Date(t * 1000);
  return d.toLocaleTimeString('zh-CN', { hour12: false });
};

/* ---------- battery cells ---------- */
const BATTERY_CELLS = 10;
const cellsEl = $('battery-cells');
const cells = [];
for (let i = 0; i < BATTERY_CELLS; i++) {
  const c = document.createElement('div');
  c.className = 'cell';
  cellsEl.appendChild(c);
  cells.push(c);
}
function setBattery(pct, status, charging) {
  const filled = Math.round((pct / 100) * BATTERY_CELLS);
  cells.forEach((c, i) => c.classList.toggle('on', i < filled));
  cellsEl.classList.toggle('low',  status === 'low');
  cellsEl.classList.toggle('crit', status === 'critical');
  $('battery-text').innerHTML =
    `${pct.toFixed(0)}%` + (charging ? ' <span class="charging">⚡</span>' : '');
}

/* ---------- connection ---------- */
sock.on('connect',    () => { $('conn-state').textContent = '已连接'; $('conn-state').className = 'badge ok'; });
sock.on('disconnect', () => { $('conn-state').textContent = '未连接'; $('conn-state').className = 'badge bad'; });

/* ---------- telemetry ---------- */
const STATUS_LABEL = { ok: '正常', low: '低电量', critical: '危急', charging: '充电中' };

sock.on('state_update', (s) => {
  const p = s.position || {};
  const v = s.velocity || {};
  const b = s.battery  || {};

  $('pos').innerHTML  = `${fmt(p.x)}<span class="u">,</span> ${fmt(p.y)}`;
  $('yaw').innerHTML  = `${fmt((p.yaw || 0) * 180 / Math.PI, 1)}<span class="u">°</span>`;
  $('vlin').innerHTML = `${fmt(v.v)}<span class="u"> m/s</span>`;
  $('vang').innerHTML = `${fmt(v.w)}<span class="u"> rad/s</span>`;

  const pct = Math.max(0, Math.min(100, b.percentage || 0));
  setBattery(pct, b.status, !!b.charging);

  const status = b.status || 'ok';
  $('bstatus').textContent = STATUS_LABEL[status] || status;
  $('bstatus').className   = 'v status-' + status;

  const dist = b.estimated_remaining_distance_m;
  $('bdist').innerHTML =
    `${(typeof dist === 'number' && dist > 0) ? dist.toFixed(0) : '—'}<span class="u"> m</span>`;

  $('header-meta').textContent =
    `(${fmt(p.x, 1)}, ${fmt(p.y, 1)}) · ${pct.toFixed(0)}%`;
});

/* ---------- camera ---------- */
let lastFrame = 0;
let frameCount = 0;
let fps = 0;
setInterval(() => {
  $('cam-fps').textContent = `${fps.toFixed(0)} FPS`;
  fps = frameCount;
  frameCount = 0;
}, 1000);

sock.on('camera_frame', (f) => {
  const img = $('cam-img');
  img.src = 'data:image/jpeg;base64,' + f.jpeg_b64;
  img.classList.add('live');
  $('cam-empty').style.display = 'none';
  frameCount++;
  if (f.width && f.height) {
    $('cam-meta').textContent = `${f.width}×${f.height}`;
  }
});

/* ---------- chat ---------- */
const chatLog = $('chat-log');
function appendChat(line) {
  const div = document.createElement('div');
  div.className = 'chat-line ' + (line.role || 'robot');
  div.innerHTML =
    `<span class="ts">${tsStr(line.time || Date.now() / 1000)}</span>` +
    `<div class="body"><b>${line.role === 'user' ? '我' : '机器人'}</b><span></span></div>`;
  div.querySelector('.body span').textContent = line.text;
  chatLog.appendChild(div);
  chatLog.scrollTop = chatLog.scrollHeight;
}
sock.on('chat_history', (history) => {
  chatLog.innerHTML = '';
  (history || []).forEach(appendChat);
});
sock.on('chat_msg', appendChat);

$('chat-form').addEventListener('submit', (e) => {
  e.preventDefault();
  const inp = $('chat-input');
  const text = inp.value.trim();
  if (!text) return;
  sock.emit('user_cmd', { text });
  inp.value = '';
});

/* ---------- photos (filmstrip) ---------- */
const photoGrid = $('photo-grid');
const photoEmpty = $('photo-empty');
const photoCount = $('photo-count');
let totalPhotos = 0;
function refreshPhotoUI() {
  photoEmpty.style.display = totalPhotos === 0 ? 'flex' : 'none';
  photoCount.textContent = totalPhotos + ' 张';
}
refreshPhotoUI();
function addPhoto(ph) {
  const card = document.createElement('div');
  card.className = 'photo-card';
  const src = ph.thumb ? ('data:image/jpeg;base64,' + ph.thumb) : '';
  card.innerHTML =
    `<img src="${src}" alt="${ph.label || ''}" />` +
    `<div class="meta"><span class="label"></span><span class="ts">${tsStr(ph.time || Date.now() / 1000)}</span></div>`;
  card.querySelector('.label').textContent = ph.label || '(未命名)';
  if (ph.filepath) card.title = ph.filepath;
  photoGrid.insertBefore(card, photoGrid.firstChild);
  totalPhotos++;
  while (photoGrid.children.length > 60) {
    photoGrid.removeChild(photoGrid.lastChild);
  }
  refreshPhotoUI();
}
sock.on('photo', addPhoto);
sock.on('photo_history', (list) => {
  // server sends oldest→newest; replay in order so newest ends up first
  photoGrid.innerHTML = '';
  totalPhotos = 0;
  (list || []).forEach(addPhoto);
});
