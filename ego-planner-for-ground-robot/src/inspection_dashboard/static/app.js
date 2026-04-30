// inspection_dashboard frontend
const sock = io({ transports: ['websocket', 'polling'] });

const $ = (id) => document.getElementById(id);
const fmt = (n, d = 2) => (typeof n === 'number' ? n.toFixed(d) : '—');
const tsStr = (t) => {
  const d = new Date(t * 1000);
  return d.toLocaleTimeString();
};

sock.on('connect', () => {
  $('conn-state').textContent = '已连接';
  $('conn-state').className = 'badge ok';
});
sock.on('disconnect', () => {
  $('conn-state').textContent = '未连接';
  $('conn-state').className = 'badge bad';
});

sock.on('state_update', (s) => {
  const p = s.position || {};
  const v = s.velocity || {};
  const b = s.battery || {};
  $('pos').textContent  = `(${fmt(p.x)}, ${fmt(p.y)})`;
  $('yaw').textContent  = `${fmt((p.yaw * 180 / Math.PI), 1)}°`;
  $('vlin').textContent = `${fmt(v.v)} m/s`;
  $('vang').textContent = `${fmt(v.w)} rad/s`;
  const pct = Math.max(0, Math.min(100, b.percentage || 0));
  $('battery-fill').style.width = pct.toFixed(1) + '%';
  $('battery-fill').style.background =
    pct < 20 ? 'linear-gradient(90deg, #da3633, #f85149)'
             : pct < 40 ? 'linear-gradient(90deg, #d29922, #f0b429)'
                        : 'linear-gradient(90deg, #2ea043, #56d364)';
  $('battery-text').textContent = pct.toFixed(1) + '%' + (b.charging ? ' ⚡' : '');
  $('bstatus').textContent = b.status || '—';
  $('bdist').textContent  = fmt(b.estimated_remaining_distance_m, 1) + ' m';
});

sock.on('camera_frame', (f) => {
  const img = $('cam-img');
  img.src = 'data:image/jpeg;base64,' + f.jpeg_b64;
  img.style.display = 'block';
  $('cam-empty').style.display = 'none';
});

const chatLog = $('chat-log');
function appendChat(line) {
  const div = document.createElement('div');
  div.className = 'chat-line ' + (line.role || 'robot');
  div.innerHTML =
    `<span class="ts">${tsStr(line.time || Date.now() / 1000)}</span>` +
    `<b>${line.role === 'user' ? '我' : '机器人'}：</b>` +
    `<span></span>`;
  div.querySelector('span:last-child').textContent = line.text;
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

const photoGrid = $('photo-grid');
sock.on('photo', (ph) => {
  const card = document.createElement('div');
  card.className = 'photo-card';
  const src = ph.thumb ? ('data:image/jpeg;base64,' + ph.thumb) : '';
  card.innerHTML =
    `<img src="${src}" alt="${ph.label || ''}" />` +
    `<div class="label"></div><div class="ts">${tsStr(ph.time || Date.now() / 1000)}</div>`;
  card.querySelector('.label').textContent = ph.label || '(未命名)';
  photoGrid.insertBefore(card, photoGrid.firstChild);
  // 限制数量
  while (photoGrid.children.length > 24) {
    photoGrid.removeChild(photoGrid.lastChild);
  }
});
