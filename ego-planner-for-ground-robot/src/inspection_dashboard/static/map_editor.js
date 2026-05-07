// OSM map graph editor.
// Coordinate convention (与项目一致): node.x = lon, node.y = lat
// Screen y is flipped so larger y goes up.

const SVG_NS = 'http://www.w3.org/2000/svg';
const svg = document.getElementById('map-svg');
const gridLayer = document.getElementById('grid-layer');
const edgeLayer = document.getElementById('edge-layer');
const nodeLayer = document.getElementById('node-layer');
const robotLayer = document.getElementById('robot-layer');
const statusMsg = document.getElementById('status-msg');
const osmPathBadge = document.getElementById('osm-path');

let DATA = { osm_path: '', nodes: [], edges: [] };
const nodeById = new Map();
const edgeById = new Map();

// view state: world-to-screen with rotation applied first, then translate/scale.
// rot is clockwise degrees ∈ {0, 90, 180, 270}. Default 90 (matches physical site layout).
const view = { cx: 0, cy: 0, scale: 8, rot: 90 };
let dirty = false;

let selectedNode = null;   // node object
let selectedEdge = null;
let armedAddEdge = false;
let edgePickFirst = null;

let nextNodeId = 1000;
let nextEdgeId = 2000;

// ---------- helpers ----------
const $ = (id) => document.getElementById(id);
function setStatus(text, kind = '') {
  statusMsg.textContent = text;
  statusMsg.className = kind;
}
function markDirty() {
  dirty = true;
  setStatus('● 有未保存修改', 'err');
}

function bbox() {
  const r = svg.getBoundingClientRect();
  return { w: r.width, h: r.height };
}
// rotate a world point clockwise by view.rot (cx,cy live in this rotated space)
function _rot(x, y) {
  switch (view.rot % 360) {
    case 0:   return [x, y];
    case 90:  return [y, -x];
    case 180: return [-x, -y];
    case 270: return [-y, x];
    default:  return [x, y];
  }
}
function _unrot(rx, ry) {
  switch (view.rot % 360) {
    case 0:   return [rx, ry];
    case 90:  return [-ry, rx];
    case 180: return [-rx, -ry];
    case 270: return [ry, -rx];
    default:  return [rx, ry];
  }
}
// world (x, y) -> screen (sx, sy)
function w2s(x, y) {
  const { w, h } = bbox();
  const [rx, ry] = _rot(x, y);
  return [(rx - view.cx) * view.scale + w / 2,
          -(ry - view.cy) * view.scale + h / 2];
}
// screen -> world
function s2w(sx, sy) {
  const { w, h } = bbox();
  const rx = (sx - w / 2) / view.scale + view.cx;
  const ry = -(sy - h / 2) / view.scale + view.cy;
  return _unrot(rx, ry);
}
// shorthand for callers that want one axis
function w2sX(x, y) { return w2s(x, y)[0]; }
function w2sY(x, y) { return w2s(x, y)[1]; }

// ---------- data load/save ----------
async function loadMap() {
  setStatus('加载中...', '');
  const r = await fetch('/api/map');
  if (!r.ok) {
    const e = await r.json().catch(() => ({}));
    setStatus('加载失败: ' + (e.error || r.status), 'err');
    return;
  }
  DATA = await r.json();
  osmPathBadge.textContent = DATA.osm_path || '(未知路径)';
  // 计算后续 id 起点 (避开已有数字 id)
  let mn = 0, me = 0;
  DATA.nodes.forEach((n) => { mn = Math.max(mn, parseInt(n.id) || 0); });
  DATA.edges.forEach((e) => { me = Math.max(me, parseInt(e.id) || 0); });
  nextNodeId = mn + 1;
  nextEdgeId = Math.max(me + 1, 2000);

  rebuildIndex();
  // 等 SVG 真正有尺寸再 fit (首次加载时 layout 可能还没完成)
  await new Promise((res) => requestAnimationFrame(() => requestAnimationFrame(res)));
  fitView();
  render();
  setStatus(`已加载: ${DATA.nodes.length} 节点 / ${DATA.edges.length} 边`, 'ok');
  dirty = false;
}

function rebuildIndex() {
  nodeById.clear(); edgeById.clear();
  DATA.nodes.forEach((n) => nodeById.set(String(n.id), n));
  DATA.edges.forEach((e) => edgeById.set(String(e.id), e));
}

async function saveMap() {
  setStatus('保存中...', '');
  const body = {
    osm_path: DATA.osm_path,
    nodes: DATA.nodes,
    edges: DATA.edges,
  };
  const r = await fetch('/api/map', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const j = await r.json().catch(() => ({}));
  if (r.ok) {
    const parts = [`✅ 已保存: ${j.path}`];
    if (j.simplified) parts.push(j.simplified.ok ? '简化版✓' : '简化版✗');
    if (j.reload)     parts.push(j.reload.ok ? 'NLP热加载✓' : 'NLP热加载✗(' + (j.reload.msg||'') + ')');
    setStatus(parts.join(' · '), 'ok');
    dirty = false;
  } else {
    setStatus('保存失败: ' + (j.error || r.status), 'err');
  }
}

// ---------- view ----------
function fitView() {
  if (!DATA.nodes.length) {
    view.cx = 0; view.cy = 0; view.scale = 8;
    return;
  }
  // bbox in ROTATED space (since cx/cy live there)
  let minx = Infinity, maxx = -Infinity, miny = Infinity, maxy = -Infinity;
  DATA.nodes.forEach((n) => {
    const [rx, ry] = _rot(n.x, n.y);
    minx = Math.min(minx, rx); maxx = Math.max(maxx, rx);
    miny = Math.min(miny, ry); maxy = Math.max(maxy, ry);
  });
  const wx = Math.max(1, maxx - minx);
  const wy = Math.max(1, maxy - miny);
  const { w, h } = bbox();
  const padding = 60;
  view.scale = Math.min((w - padding * 2) / wx, (h - padding * 2) / wy);
  view.cx = (minx + maxx) / 2;
  view.cy = (miny + maxy) / 2;
}

// ---------- render ----------
function render() {
  const { w, h } = bbox();

  // grid in screen space (decorative; spacing scales with zoom)
  gridLayer.innerHTML = '';
  const stepWorld = 5;
  const stepPx = stepWorld * view.scale;
  if (stepPx > 6) {
    // align to view center
    const cxPx = w / 2 - (view.cx % stepWorld) * view.scale;
    const cyPx = h / 2 + (view.cy % stepWorld) * view.scale;
    for (let sx = cxPx % stepPx; sx < w; sx += stepPx) {
      const ln = document.createElementNS(SVG_NS, 'line');
      ln.setAttribute('x1', sx); ln.setAttribute('x2', sx);
      ln.setAttribute('y1', 0);  ln.setAttribute('y2', h);
      ln.setAttribute('class', 'grid-line');
      gridLayer.appendChild(ln);
    }
    for (let sy = cyPx % stepPx; sy < h; sy += stepPx) {
      const ln = document.createElementNS(SVG_NS, 'line');
      ln.setAttribute('y1', sy); ln.setAttribute('y2', sy);
      ln.setAttribute('x1', 0);  ln.setAttribute('x2', w);
      ln.setAttribute('class', 'grid-line');
      gridLayer.appendChild(ln);
    }
  }

  // edges
  edgeLayer.innerHTML = '';
  DATA.edges.forEach((e) => {
    const a = nodeById.get(String(e.a));
    const b = nodeById.get(String(e.b));
    if (!a || !b) return;
    const [x1, y1] = w2s(a.x, a.y);
    const [x2, y2] = w2s(b.x, b.y);
    const hit = document.createElementNS(SVG_NS, 'line');
    hit.setAttribute('x1', x1); hit.setAttribute('y1', y1);
    hit.setAttribute('x2', x2); hit.setAttribute('y2', y2);
    hit.setAttribute('class', 'edge-hit');
    hit.addEventListener('click', (ev) => { ev.stopPropagation(); selectEdge(e); });
    edgeLayer.appendChild(hit);
    const ln = document.createElementNS(SVG_NS, 'line');
    ln.setAttribute('x1', x1); ln.setAttribute('y1', y1);
    ln.setAttribute('x2', x2); ln.setAttribute('y2', y2);
    ln.setAttribute('class', 'edge' + (selectedEdge && selectedEdge.id === e.id ? ' selected' : ''));
    ln.style.pointerEvents = 'none';
    edgeLayer.appendChild(ln);
  });

  // nodes
  nodeLayer.innerHTML = '';
  DATA.nodes.forEach((n) => {
    const g = document.createElementNS(SVG_NS, 'g');
    g.setAttribute('class', 'node-group');
    g.setAttribute('data-id', n.id);
    const [cx, cy] = w2s(n.x, n.y);
    g.setAttribute('transform', `translate(${cx},${cy})`);
    const c = document.createElementNS(SVG_NS, 'circle');
    let cls = 'node-circle';
    if (n.tags && n.tags.area) cls += ' area-' + n.tags.area;
    if (n.tags && n.tags.device_type) cls += ' dev-' + n.tags.device_type;
    if (selectedNode && selectedNode.id === n.id) cls += ' selected';
    if (armedAddEdge && edgePickFirst && edgePickFirst.id === n.id) cls += ' armed';
    c.setAttribute('class', cls);
    c.setAttribute('r', 9);
    c.setAttribute('cx', 0); c.setAttribute('cy', 0);
    g.appendChild(c);
    const t = document.createElementNS(SVG_NS, 'text');
    t.setAttribute('class', 'node-label');
    t.setAttribute('y', -14);
    t.textContent = n.name;
    g.appendChild(t);

    g.addEventListener('mousedown', (ev) => onNodeMouseDown(ev, n, g));
    g.addEventListener('click', (ev) => {
      ev.stopPropagation();
      if (armedAddEdge) {
        handleEdgePick(n);
      } else {
        selectNode(n);
      }
    });
    nodeLayer.appendChild(g);
  });
  renderRobot();
}

// ---------- selection ----------
function selectNode(n) {
  selectedNode = n; selectedEdge = null;
  showNodeForm(n);
  render();
}
function selectEdge(e) {
  selectedEdge = e; selectedNode = null;
  showEdgeForm(e);
  render();
}
function clearSelection() {
  selectedNode = null; selectedEdge = null;
  $('node-form').style.display = 'none';
  $('edge-form').style.display = 'none';
  $('empty-msg').style.display = 'block';
}

// ---------- forms ----------
function showNodeForm(n) {
  $('empty-msg').style.display = 'none';
  $('edge-form').style.display = 'none';
  const f = $('node-form');
  f.style.display = 'flex';
  $('nf-id').value = n.id;
  $('nf-name').value = n.name;
  $('nf-x').value = n.x.toFixed(2);
  $('nf-y').value = n.y.toFixed(2);
  const tagsBox = $('nf-tags');
  tagsBox.innerHTML = '';
  Object.entries(n.tags || {}).forEach(([k, v]) => addTagRow(k, v));
}
function addTagRow(k = '', v = '') {
  const row = document.createElement('div');
  row.className = 'tag-row';
  row.innerHTML = `<input placeholder="key" value="${k}" />
                   <input placeholder="value" value="${v}" />
                   <button type="button">×</button>`;
  row.querySelector('button').addEventListener('click', () => row.remove());
  $('nf-tags').appendChild(row);
}
function showEdgeForm(e) {
  $('empty-msg').style.display = 'none';
  $('node-form').style.display = 'none';
  const f = $('edge-form');
  f.style.display = 'flex';
  $('ef-id').value = e.id;
  const a = nodeById.get(String(e.a));
  const b = nodeById.get(String(e.b));
  $('ef-a').value = a ? `${e.a} (${a.name})` : e.a;
  $('ef-b').value = b ? `${e.b} (${b.name})` : b;
  $('ef-name').value = e.name || '';
}

$('nf-add-tag').addEventListener('click', () => addTagRow());
$('node-form').addEventListener('submit', (ev) => {
  ev.preventDefault();
  if (!selectedNode) return;
  selectedNode.name = $('nf-name').value.trim();
  selectedNode.x = parseFloat($('nf-x').value) || 0;
  selectedNode.y = parseFloat($('nf-y').value) || 0;
  const tags = {};
  $('nf-tags').querySelectorAll('.tag-row').forEach((row) => {
    const ins = row.querySelectorAll('input');
    const k = ins[0].value.trim();
    const v = ins[1].value.trim();
    if (k) tags[k] = v;
  });
  selectedNode.tags = tags;
  markDirty();
  render();
});
$('edge-form').addEventListener('submit', (ev) => {
  ev.preventDefault();
  if (!selectedEdge) return;
  selectedEdge.name = $('ef-name').value.trim();
  markDirty();
  render();
});

// ---------- node drag & pan/zoom ----------
let dragging = null;     // {node, startMouse, startWorld}
let panning = null;

function onNodeMouseDown(ev, n, g) {
  if (ev.button !== 0) return;
  if (armedAddEdge) return;          // 不在选边模式时才允许拖
  ev.stopPropagation();
  ev.preventDefault();
  dragging = { node: n, g };
  g.classList.add('dragging');
}

svg.addEventListener('mousemove', (ev) => {
  if (dragging) {
    const r = svg.getBoundingClientRect();
    const mx = ev.clientX - r.left, my = ev.clientY - r.top;
    const [wx, wy] = s2w(mx, my);
    dragging.node.x = wx;
    dragging.node.y = wy;
    dragging.g.setAttribute('transform', `translate(${mx},${my})`);
    redrawEdgesOnly();
    if (selectedNode === dragging.node) {
      $('nf-x').value = wx.toFixed(2);
      $('nf-y').value = wy.toFixed(2);
    }
    return;
  }
  if (panning) {
    const dx = ev.clientX - panning.startX;
    const dy = ev.clientY - panning.startY;
    view.cx = panning.cx0 - dx / view.scale;
    view.cy = panning.cy0 + dy / view.scale;
    render();
  }
});

svg.addEventListener('mouseup', (ev) => {
  if (dragging) {
    dragging.g.classList.remove('dragging');
    markDirty();
    dragging = null;
    render();
  }
  if (panning) {
    svg.classList.remove('panning');
    panning = null;
  }
});

svg.addEventListener('mousedown', (ev) => {
  if (ev.button === 2 || ev.button === 1 ||
      (ev.button === 0 && ev.target === svg)) {
    panning = {
      startX: ev.clientX, startY: ev.clientY,
      cx0: view.cx, cy0: view.cy,
    };
    svg.classList.add('panning');
    if (ev.button === 0) clearSelection();
  }
});

svg.addEventListener('contextmenu', (ev) => ev.preventDefault());

svg.addEventListener('wheel', (ev) => {
  ev.preventDefault();
  const factor = ev.deltaY < 0 ? 1.15 : 1 / 1.15;
  // zoom around mouse: do math in rotated space directly
  const r = svg.getBoundingClientRect();
  const { w, h } = bbox();
  const mx = ev.clientX - r.left, my = ev.clientY - r.top;
  const rxBefore = (mx - w / 2) / view.scale + view.cx;
  const ryBefore = -(my - h / 2) / view.scale + view.cy;
  view.scale *= factor;
  view.scale = Math.max(0.5, Math.min(view.scale, 200));
  const rxAfter = (mx - w / 2) / view.scale + view.cx;
  const ryAfter = -(my - h / 2) / view.scale + view.cy;
  view.cx += rxBefore - rxAfter;
  view.cy += ryBefore - ryAfter;
  render();
}, { passive: false });

function redrawEdgesOnly() {
  const lines = edgeLayer.querySelectorAll('line');
  let i = 0;
  DATA.edges.forEach((e) => {
    const a = nodeById.get(String(e.a));
    const b = nodeById.get(String(e.b));
    if (!a || !b) return;
    const hit = lines[i++], real = lines[i++];
    if (!hit || !real) return;
    const [x1, y1] = w2s(a.x, a.y);
    const [x2, y2] = w2s(b.x, b.y);
    hit.setAttribute('x1', x1); hit.setAttribute('y1', y1);
    hit.setAttribute('x2', x2); hit.setAttribute('y2', y2);
    real.setAttribute('x1', x1); real.setAttribute('y1', y1);
    real.setAttribute('x2', x2); real.setAttribute('y2', y2);
  });
}

window.addEventListener('resize', render);

// ---------- toolbar ----------
function newId(taken) {
  let i = nextNodeId;
  while (taken.has(String(i))) i++;
  nextNodeId = i + 1;
  return String(i);
}
function newEdgeId(taken) {
  let i = nextEdgeId;
  while (taken.has(String(i))) i++;
  nextEdgeId = i + 1;
  return String(i);
}

function addNodeAt(x, y) {
  const id = newId(nodeById);
  const n = {
    id, name: '新节点_' + id, x, y,
    tags: { area: 'center', device_type: 'waypoint' },
  };
  DATA.nodes.push(n);
  nodeById.set(id, n);
  selectNode(n);
  markDirty();
  render();
}

function handleEdgePick(n) {
  if (!edgePickFirst) {
    edgePickFirst = n;
    setStatus(`已选起点 [${n.name}]，再点一个节点作为终点 (Esc 取消)`, '');
    render();
    return;
  }
  if (edgePickFirst.id === n.id) {
    setStatus('起点与终点相同，已取消', 'err');
    cancelArmedEdge();
    return;
  }
  // 检查重复
  const exist = DATA.edges.some(
    (e) => (e.a === edgePickFirst.id && e.b === n.id) ||
           (e.a === n.id && e.b === edgePickFirst.id));
  if (exist) {
    setStatus('该边已存在', 'err');
    cancelArmedEdge();
    return;
  }
  const id = newEdgeId(edgeById);
  const e = {
    id, a: edgePickFirst.id, b: n.id,
    name: `${edgePickFirst.name}-${n.name}`,
  };
  DATA.edges.push(e); edgeById.set(id, e);
  setStatus(`已添加边: ${e.name}`, 'ok');
  cancelArmedEdge();
  selectEdge(e);
  markDirty();
  render();
}
function cancelArmedEdge() {
  armedAddEdge = false;
  edgePickFirst = null;
  $('btn-add-edge').classList.remove('armed');
  render();
}

document.addEventListener('keydown', (ev) => {
  if (ev.key === 'Escape') cancelArmedEdge();
  if (ev.key === 'Delete' && (selectedNode || selectedEdge)) deleteSelected();
});

function deleteSelected() {
  if (selectedEdge) {
    DATA.edges = DATA.edges.filter((e) => e.id !== selectedEdge.id);
    edgeById.delete(selectedEdge.id);
    setStatus('已删除边', 'ok');
    selectedEdge = null;
  } else if (selectedNode) {
    const nid = selectedNode.id;
    DATA.edges = DATA.edges.filter((e) => e.a !== nid && e.b !== nid);
    DATA.nodes = DATA.nodes.filter((n) => n.id !== nid);
    rebuildIndex();
    setStatus('已删除节点及其相连边', 'ok');
    selectedNode = null;
  } else { return; }
  clearSelection();
  markDirty();
  render();
}

document.querySelectorAll('#map-toolbar button').forEach((btn) => {
  btn.addEventListener('click', async () => {
    const act = btn.dataset.act;
    if (act === 'reload') {
      if (dirty && !confirm('有未保存修改，确定重新加载?')) return;
      loadMap();
    } else if (act === 'add-node') {
      addNodeAt(view.cx, view.cy);
    } else if (act === 'add-here') {
      try {
        const r = await fetch('/api/robot_pose');
        const j = await r.json();
        addNodeAt(j.x, j.y);
        setStatus(`已用机器人位置 (${j.x.toFixed(2)}, ${j.y.toFixed(2)}) 添加节点`, 'ok');
      } catch (e) {
        setStatus('获取机器人位置失败: ' + e, 'err');
      }
    } else if (act === 'add-edge') {
      armedAddEdge = !armedAddEdge;
      edgePickFirst = null;
      btn.classList.toggle('armed', armedAddEdge);
      setStatus(armedAddEdge ? '依次点击两个节点以连边 (Esc 取消)' : '已取消加边模式', '');
      render();
    } else if (act === 'delete') {
      deleteSelected();
    } else if (act === 'fit') {
      fitView(); render();
    } else if (act === 'rotate') {
      view.rot = (view.rot + 90) % 360;
      fitView(); render();
      setStatus(`视图旋转: ${view.rot}°`, '');
    } else if (act === 'save') {
      saveMap();
    }
  });
});

window.addEventListener('beforeunload', (ev) => {
  if (dirty) { ev.preventDefault(); ev.returnValue = ''; }
});

// ---------- live robot pose marker ----------
let robotPose = null;  // {x, y, yaw}

function renderRobot() {
  robotLayer.innerHTML = '';
  if (!robotPose) return;
  const [sx, sy] = w2s(robotPose.x, robotPose.y);
  // 朝向: world yaw 是世界坐标下绕 z 的弧度。
  // 屏幕需要考虑视图旋转 (clockwise rot°) 和 y 轴翻转。
  // 等价做法: 把 (cos, sin) 当成单位向量做 w2s，然后算 atan2 屏幕角。
  const [hx, hy] = w2s(robotPose.x + Math.cos(robotPose.yaw || 0),
                       robotPose.y + Math.sin(robotPose.yaw || 0));
  const screenAngle = Math.atan2(hy - sy, hx - sx) * 180 / Math.PI;

  const g = document.createElementNS(SVG_NS, 'g');
  g.setAttribute('transform', `translate(${sx},${sy}) rotate(${screenAngle})`);
  // 三角形 (机头朝向 +x)
  const tri = document.createElementNS(SVG_NS, 'polygon');
  tri.setAttribute('points', '12,0 -8,-7 -4,0 -8,7');
  tri.setAttribute('class', 'robot-marker');
  g.appendChild(tri);
  // 中心圆
  const c = document.createElementNS(SVG_NS, 'circle');
  c.setAttribute('r', 4);
  c.setAttribute('class', 'robot-marker');
  g.appendChild(c);
  robotLayer.appendChild(g);
}

if (typeof io === 'function') {
  try {
    const sock = io({ transports: ['websocket', 'polling'] });
    sock.on('state_update', (s) => {
      if (s && s.position) {
        robotPose = {
          x: s.position.x || 0,
          y: s.position.y || 0,
          yaw: s.position.yaw || 0,
        };
        renderRobot();
      }
    });
  } catch (e) {
    console.warn('socket.io init failed', e);
  }
}

loadMap();
