const state = {
  mapVersion: -1,
  pathVersion: -1,
  status: null,
  map: null,
  path: [],
  pendingGoal: null,
  goalPickArmed: false,
  mapView: {
    scale: 1,
    minScale: 1,
    maxScale: 1,
    offsetX: 0,
    offsetY: 0,
    initialized: false,
  },
};

const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");
const statusLine = document.getElementById("status-line");
const poseText = document.getElementById("pose-text");
const velocityText = document.getElementById("velocity-text");
const mapText = document.getElementById("map-text");
const goalText = document.getElementById("goal-text");
const manualPoseText = document.getElementById("manual-pose-text");
const manualVelocityText = document.getElementById("manual-velocity-text");
const pageTitle = document.getElementById("page-title");
const navPage = document.getElementById("nav-page");
const manualPage = document.getElementById("manual-page");

const joystickBase = document.getElementById("joystick-base");
const joystickKnob = document.getElementById("joystick-knob");
const stopButton = document.getElementById("stop-button");
const refreshMapButton = document.getElementById("refresh-map-button");
const manualModeButton = document.getElementById("manual-mode-button");
const navModeButton = document.getElementById("nav-mode-button");
const zoomInButton = document.getElementById("zoom-in-button");
const zoomOutButton = document.getElementById("zoom-out-button");
const fitMapButton = document.getElementById("fit-map-button");
const pickGoalButton = document.getElementById("pick-goal-button");
const clearGoalButton = document.getElementById("clear-goal-button");
const confirmGoalButton = document.getElementById("confirm-goal-button");
const cameraFrame = document.getElementById("camera-frame");
const cameraStatus = document.getElementById("camera-status");
const modeText = document.getElementById("mode-text");
const modeDot = document.getElementById("mode-dot");
const radarDot = document.getElementById("radar-dot");
const baseDot = document.getElementById("base-dot");
const cameraDot = document.getElementById("camera-dot");

let joystickPointerId = null;
let joystickCenter = null;
let joystickRadius = 0;
let cmdVelTimer = null;
let lastCmd = { linear: 0, angular: 0 };
let mapGesture = null;
let mapImage = null;
let cameraTimer = null;
let statusOverrideUntil = 0;

function clamp(value, low, high) {
  return Math.max(low, Math.min(high, value));
}

function formatNumber(value) {
  return Number.isFinite(value) ? value.toFixed(2) : "--";
}

function setStatusMessage(message, durationMs = 0) {
  statusLine.textContent = message;
  statusOverrideUntil = durationMs > 0 ? Date.now() + durationMs : 0;
}

function setDefaultStatus(message) {
  if (Date.now() < statusOverrideUntil) {
    return;
  }
  statusLine.textContent = message;
}

async function apiGet(url) {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`${url} -> ${response.status}`);
  }
  return response.json();
}

async function apiPost(url, payload) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload || {}),
  });
  if (!response.ok) {
    throw new Error(`${url} -> ${response.status}`);
  }
  return response.json();
}

function resizeCanvas() {
  const centerWorld = state.map && state.mapView.initialized
    ? canvasToWorld(canvas.width / 2, canvas.height / 2)
    : null;
  const size = Math.max(320, Math.floor(canvas.clientWidth));
  canvas.width = size;
  canvas.height = size;
  if (state.map) {
    syncMapView(centerWorld);
  } else {
    renderScene();
  }
}

function setModeButtons(mode) {
  manualModeButton.classList.toggle("active", mode === "manual");
  navModeButton.classList.toggle("active", mode === "nav");
  joystickBase.classList.toggle("disabled", mode !== "manual");
  navPage.classList.toggle("active", mode === "nav");
  manualPage.classList.toggle("active", mode === "manual");
  pageTitle.textContent = mode === "manual" ? "手动页面" : "导航页面";
  if (mode === "nav") {
    window.requestAnimationFrame(() => {
      resizeCanvas();
    });
  }
}

function setStatusDot(element, isOnline, onlineClass = "online") {
  element.classList.remove("online", "offline", "manual", "nav");
  element.classList.add(isOnline ? onlineClass : "offline");
}

function getMapPixelFromWorld(x, y) {
  if (!state.map) {
    return { x: 0, y: 0 };
  }
  const { width, height, resolution, origin } = state.map;
  const px = (x - origin.x) / resolution;
  const py = (y - origin.y) / resolution;
  return {
    x: px,
    y: height - py,
  };
}

function getWorldFromMapPixel(x, y) {
  if (!state.map) {
    return { x: 0, y: 0 };
  }
  const { height, resolution, origin } = state.map;
  return {
    x: origin.x + x * resolution,
    y: origin.y + (height - y) * resolution,
  };
}

function mapPixelToCanvas(x, y) {
  return {
    x: state.mapView.offsetX + x * state.mapView.scale,
    y: state.mapView.offsetY + y * state.mapView.scale,
  };
}

function canvasToMapPixel(x, y) {
  return {
    x: (x - state.mapView.offsetX) / state.mapView.scale,
    y: (y - state.mapView.offsetY) / state.mapView.scale,
  };
}

function worldToCanvas(x, y) {
  if (!state.map) {
    return { x: 0, y: 0 };
  }
  const mapPixel = getMapPixelFromWorld(x, y);
  return mapPixelToCanvas(mapPixel.x, mapPixel.y);
}

function canvasToWorld(x, y) {
  if (!state.map) {
    return { x: 0, y: 0 };
  }
  const mapPixel = canvasToMapPixel(x, y);
  return getWorldFromMapPixel(mapPixel.x, mapPixel.y);
}

function getFitScale() {
  if (!state.map) {
    return 1;
  }
  const margin = 28;
  return Math.min(
    (canvas.width - margin * 2) / state.map.width,
    (canvas.height - margin * 2) / state.map.height,
  );
}

function centerMapOn(mapPixelX, mapPixelY) {
  state.mapView.offsetX = canvas.width / 2 - mapPixelX * state.mapView.scale;
  state.mapView.offsetY = canvas.height / 2 - mapPixelY * state.mapView.scale;
}

function resetMapView(centerOnRobot = true) {
  if (!state.map) {
    return;
  }
  const fitScale = getFitScale();
  state.mapView.scale = fitScale;
  state.mapView.minScale = fitScale * 0.85;
  state.mapView.maxScale = fitScale * 8.0;
  state.mapView.initialized = true;

  const robot = centerOnRobot ? state.status?.odom : null;
  if (robot) {
    const mapPixel = getMapPixelFromWorld(robot.x, robot.y);
    centerMapOn(mapPixel.x, mapPixel.y);
  } else {
    state.mapView.offsetX = (canvas.width - state.map.width * state.mapView.scale) / 2;
    state.mapView.offsetY = (canvas.height - state.map.height * state.mapView.scale) / 2;
  }
  renderScene();
}

function syncMapView(centerWorld = null) {
  if (!state.map) {
    return;
  }

  const fitScale = getFitScale();

  if (!state.mapView.initialized) {
    resetMapView(Boolean(centerWorld));
    return;
  }

  state.mapView.minScale = fitScale * 0.85;
  state.mapView.maxScale = fitScale * 8.0;
  state.mapView.scale = clamp(
    state.mapView.scale,
    state.mapView.minScale,
    state.mapView.maxScale,
  );

  if (centerWorld) {
    const centerPixel = getMapPixelFromWorld(centerWorld.x, centerWorld.y);
    centerMapOn(centerPixel.x, centerPixel.y);
  }

  renderScene();
}

function zoomAtCanvasPoint(canvasX, canvasY, factor) {
  if (!state.map) {
    return;
  }
  const focus = canvasToMapPixel(canvasX, canvasY);
  const nextScale = clamp(
    state.mapView.scale * factor,
    state.mapView.minScale,
    state.mapView.maxScale,
  );
  state.mapView.scale = nextScale;
  state.mapView.offsetX = canvasX - focus.x * nextScale;
  state.mapView.offsetY = canvasY - focus.y * nextScale;
  renderScene();
}

function zoomMap(factor) {
  zoomAtCanvasPoint(canvas.width / 2, canvas.height / 2, factor);
}

function deriveGoalYaw(goal) {
  const robot = state.status?.odom;
  if (!robot) {
    return 0.0;
  }
  return Math.atan2(goal.y - robot.y, goal.x - robot.x);
}

function rebuildMapImage() {
  if (!state.map) {
    mapImage = null;
    state.mapView.initialized = false;
    return;
  }

  const { width, height, data } = state.map;
  const imageData = new ImageData(width, height);

  for (let i = 0; i < data.length; i += 1) {
    const occupancy = data[i];
    const srcX = i % width;
    const srcY = Math.floor(i / width);
    const imageY = height - 1 - srcY;
    const index = (imageY * width + srcX) * 4;

    let r = 246;
    let g = 250;
    let b = 255;
    if (occupancy < 0) {
      r = 223;
      g = 231;
      b = 242;
    } else if (occupancy >= 100) {
      r = 31;
      g = 48;
      b = 71;
    } else if (occupancy > 0) {
      const shade = 248 - Math.round((occupancy / 100.0) * 185);
      r = shade;
      g = shade + 2;
      b = shade + 8;
    }

    imageData.data[index] = clamp(r, 0, 255);
    imageData.data[index + 1] = clamp(g, 0, 255);
    imageData.data[index + 2] = clamp(b, 0, 255);
    imageData.data[index + 3] = 255;
  }

  const offscreen = document.createElement("canvas");
  offscreen.width = width;
  offscreen.height = height;
  offscreen.getContext("2d").putImageData(imageData, 0, 0);
  mapImage = offscreen;
}

function drawMapBackdrop() {
  const gradient = ctx.createLinearGradient(0, 0, 0, canvas.height);
  gradient.addColorStop(0, "#f9fcff");
  gradient.addColorStop(1, "#e9f0f7");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = "rgba(17, 51, 85, 0.08)";
  ctx.lineWidth = 1;
  for (let x = 24; x < canvas.width; x += 24) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
  }
  for (let y = 24; y < canvas.height; y += 24) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }
}

function drawMapGrid() {
  if (!state.map) {
    return;
  }

  const meterStepInPixels = 1.0 / state.map.resolution;
  const spacing = meterStepInPixels * state.mapView.scale;
  if (spacing < 28) {
    return;
  }

  const startMap = canvasToMapPixel(0, 0);
  const endMap = canvasToMapPixel(canvas.width, canvas.height);
  const xStart = Math.floor(Math.min(startMap.x, endMap.x) / meterStepInPixels) * meterStepInPixels;
  const xEnd = Math.ceil(Math.max(startMap.x, endMap.x) / meterStepInPixels) * meterStepInPixels;
  const yStart = Math.floor(Math.min(startMap.y, endMap.y) / meterStepInPixels) * meterStepInPixels;
  const yEnd = Math.ceil(Math.max(startMap.y, endMap.y) / meterStepInPixels) * meterStepInPixels;

  ctx.save();
  ctx.strokeStyle = "rgba(15, 118, 214, 0.12)";
  ctx.lineWidth = 1;

  for (let x = xStart; x <= xEnd; x += meterStepInPixels) {
    const sx = state.mapView.offsetX + x * state.mapView.scale;
    ctx.beginPath();
    ctx.moveTo(sx, 0);
    ctx.lineTo(sx, canvas.height);
    ctx.stroke();
  }

  for (let y = yStart; y <= yEnd; y += meterStepInPixels) {
    const sy = state.mapView.offsetY + y * state.mapView.scale;
    ctx.beginPath();
    ctx.moveTo(0, sy);
    ctx.lineTo(canvas.width, sy);
    ctx.stroke();
  }
  ctx.restore();
}

function drawRobot() {
  if (!state.status?.odom || !state.map) {
    return;
  }

  const pose = state.status.odom;
  const p = worldToCanvas(pose.x, pose.y);
  const radiusPx = Math.max(
    10,
    (state.status.robot_radius / state.map.resolution) * state.mapView.scale,
  );

  ctx.save();
  ctx.translate(p.x, p.y);
  ctx.rotate(-pose.yaw);
  ctx.fillStyle = "#0f76d6";
  ctx.beginPath();
  ctx.arc(0, 0, radiusPx, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = "#ffffff";
  ctx.beginPath();
  ctx.moveTo(radiusPx * 0.95, 0);
  ctx.lineTo(-radiusPx * 0.22, radiusPx * 0.58);
  ctx.lineTo(-radiusPx * 0.22, -radiusPx * 0.58);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function drawPath() {
  if (!state.path || state.path.length < 2) {
    return;
  }

  ctx.save();
  ctx.strokeStyle = "#15a56f";
  ctx.lineWidth = 3;
  ctx.beginPath();
  state.path.forEach((point, index) => {
    const p = worldToCanvas(point.x, point.y);
    if (index === 0) {
      ctx.moveTo(p.x, p.y);
    } else {
      ctx.lineTo(p.x, p.y);
    }
  });
  ctx.stroke();
  ctx.restore();
}

function drawGoal() {
  const goal = state.pendingGoal || state.status?.goal;
  if (!goal) {
    return;
  }

  const p = worldToCanvas(goal.x, goal.y);
  const accent = state.pendingGoal ? "#ff8a00" : "#d64b28";

  ctx.save();
  ctx.strokeStyle = accent;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(p.x, p.y, 12, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(p.x - 16, p.y);
  ctx.lineTo(p.x + 16, p.y);
  ctx.moveTo(p.x, p.y - 16);
  ctx.lineTo(p.x, p.y + 16);
  ctx.stroke();

  if (Number.isFinite(goal.yaw)) {
    ctx.translate(p.x, p.y);
    ctx.rotate(-goal.yaw);
    ctx.fillStyle = accent;
    ctx.beginPath();
    ctx.moveTo(20, 0);
    ctx.lineTo(5, 7);
    ctx.lineTo(5, -7);
    ctx.closePath();
    ctx.fill();
  }

  if (state.pendingGoal) {
    ctx.fillStyle = "#ff8a00";
    ctx.font = "700 13px 'Noto Sans SC', sans-serif";
    ctx.fillText("待确认", p.x + 14, p.y - 14);
  }
  ctx.restore();
}

function renderScene() {
  drawMapBackdrop();

  if (mapImage && state.map) {
    ctx.save();
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(
      mapImage,
      state.mapView.offsetX,
      state.mapView.offsetY,
      state.map.width * state.mapView.scale,
      state.map.height * state.mapView.scale,
    );
    ctx.restore();
    drawMapGrid();
  }

  drawPath();
  drawGoal();
  drawRobot();
}

async function fetchStatus() {
  try {
    const payload = await apiGet("/api/status");
    state.status = payload;
    setDefaultStatus(
      payload.has_odom
        ? "已连接机器人，可缩放查看地图；点“选点导航”后再确认发起导航。"
        : "已连网页服务，等待机器人位姿。",
    );

    if (payload.map_version !== state.mapVersion) {
      state.mapVersion = payload.map_version;
      fetchMap();
    }

    if (payload.path_version !== state.pathVersion) {
      state.pathVersion = payload.path_version;
      fetchPath();
    }

    updateStatusCards();
    setModeButtons(payload.control_mode || "nav");
    renderScene();
  } catch (error) {
    setStatusMessage(`连接失败：${error.message}`, 3000);
  }
}

async function fetchMap() {
  const centerWorld = state.map && state.mapView.initialized
    ? canvasToWorld(canvas.width / 2, canvas.height / 2)
    : null;
  const payload = await apiGet("/api/map");
  if (!payload.ok) {
    return;
  }
  state.map = payload.map;
  rebuildMapImage();
  if (centerWorld) {
    syncMapView(centerWorld);
  } else {
    resetMapView();
  }
  updateStatusCards();
}

async function fetchPath() {
  const payload = await apiGet("/api/path");
  if (!payload.ok) {
    return;
  }
  state.path = payload.path;
  renderScene();
}

function updateGoalControls() {
  pickGoalButton.classList.toggle("active", state.goalPickArmed);
  pickGoalButton.textContent = state.goalPickArmed ? "选点中..." : "选点导航";
  clearGoalButton.disabled = !state.pendingGoal;
  confirmGoalButton.disabled = !state.pendingGoal;
}

function updateStatusCards() {
  const odom = state.status?.odom;
  const goal = state.pendingGoal || state.status?.goal;
  const mode = state.status?.control_mode || "nav";
  const devices = state.status?.devices || {};

  poseText.textContent = odom
    ? `x ${formatNumber(odom.x)}  y ${formatNumber(odom.y)}  yaw ${formatNumber(odom.yaw)}`
    : "--";
  velocityText.textContent = odom
    ? `v ${formatNumber(odom.linear_x)}  w ${formatNumber(odom.angular_z)}`
    : "--";
  manualPoseText.textContent = poseText.textContent;
  manualVelocityText.textContent = velocityText.textContent;
  mapText.textContent = state.map
    ? `${state.map.width}×${state.map.height} @ ${formatNumber(state.map.resolution)}m`
    : "等待地图";
  goalText.textContent = goal
    ? `${state.pendingGoal ? "候选" : "目标"} x ${formatNumber(goal.x)}  y ${formatNumber(goal.y)}`
    : "未设置";

  if (state.status?.has_camera && state.status?.camera) {
    cameraStatus.textContent = `${state.status.camera.width}×${state.status.camera.height}`;
  } else {
    cameraStatus.textContent = "等待相机";
  }

  modeText.textContent = mode === "manual" ? "手动" : "导航";
  setStatusDot(modeDot, true, mode === "manual" ? "manual" : "nav");
  setStatusDot(radarDot, Boolean(devices.radar));
  setStatusDot(baseDot, Boolean(devices.base));
  setStatusDot(cameraDot, Boolean(devices.camera));
  updateGoalControls();
}

function getJoystickMetrics() {
  const rect = joystickBase.getBoundingClientRect();
  joystickCenter = {
    x: rect.left + rect.width / 2,
    y: rect.top + rect.height / 2,
  };
  joystickRadius = rect.width * 0.32;
}

function setJoystickPosition(nx, ny) {
  const x = nx * joystickRadius;
  const y = ny * joystickRadius;
  joystickKnob.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
}

function queueCmdVel(linear, angular) {
  if (state.status?.control_mode !== "manual") {
    return;
  }
  lastCmd = { linear, angular };
  if (cmdVelTimer) {
    return;
  }
  cmdVelTimer = setTimeout(async () => {
    cmdVelTimer = null;
    try {
      await apiPost("/api/cmd_vel", lastCmd);
    } catch (error) {
      setStatusMessage(`速度指令发送失败：${error.message}`, 3000);
    }
  }, 90);
}

function resetJoystick() {
  setJoystickPosition(0, 0);
  lastCmd = { linear: 0, angular: 0 };
  apiPost("/api/stop", {}).catch(() => {});
}

function onJoystickMove(clientX, clientY) {
  const dx = clientX - joystickCenter.x;
  const dy = clientY - joystickCenter.y;
  const distance = Math.hypot(dx, dy) || 1;
  const scale = Math.min(1, joystickRadius / distance);
  const nx = (dx * scale) / joystickRadius;
  const ny = (dy * scale) / joystickRadius;

  setJoystickPosition(nx, ny);

  const linearLimit = state.status?.limits?.linear ?? 0.25;
  const angularLimit = state.status?.limits?.angular ?? 1.2;
  queueCmdVel(-ny * linearLimit, -nx * angularLimit);
}

joystickBase.addEventListener("pointerdown", (event) => {
  if (state.status?.control_mode !== "manual") {
    setStatusMessage("导航模式下摇杆已禁用，请先切到手动模式。", 2200);
    return;
  }
  joystickPointerId = event.pointerId;
  getJoystickMetrics();
  joystickBase.setPointerCapture(event.pointerId);
  onJoystickMove(event.clientX, event.clientY);
});

joystickBase.addEventListener("pointermove", (event) => {
  if (event.pointerId !== joystickPointerId) {
    return;
  }
  onJoystickMove(event.clientX, event.clientY);
});

function releaseJoystick(event) {
  if (event.pointerId !== joystickPointerId) {
    return;
  }
  joystickPointerId = null;
  resetJoystick();
}

joystickBase.addEventListener("pointerup", releaseJoystick);
joystickBase.addEventListener("pointercancel", releaseJoystick);

canvas.addEventListener("wheel", (event) => {
  if (!state.map) {
    return;
  }
  event.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  const factor = event.deltaY < 0 ? 1.12 : 1 / 1.12;
  zoomAtCanvasPoint(x, y, factor);
}, { passive: false });

canvas.addEventListener("pointerdown", (event) => {
  if (!state.map) {
    return;
  }
  const rect = canvas.getBoundingClientRect();
  mapGesture = {
    pointerId: event.pointerId,
    startX: event.clientX - rect.left,
    startY: event.clientY - rect.top,
    startOffsetX: state.mapView.offsetX,
    startOffsetY: state.mapView.offsetY,
    moved: false,
  };
  canvas.setPointerCapture(event.pointerId);
});

canvas.addEventListener("pointermove", (event) => {
  if (!mapGesture || event.pointerId !== mapGesture.pointerId) {
    return;
  }
  const rect = canvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  const dx = x - mapGesture.startX;
  const dy = y - mapGesture.startY;

  if (Math.hypot(dx, dy) > 6) {
    mapGesture.moved = true;
  }

  if (mapGesture.moved) {
    state.mapView.offsetX = mapGesture.startOffsetX + dx;
    state.mapView.offsetY = mapGesture.startOffsetY + dy;
    renderScene();
  }
});

function finishMapGesture(event) {
  if (!mapGesture || event.pointerId !== mapGesture.pointerId) {
    return;
  }

  const rect = canvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  const wasTap = !mapGesture.moved;

  if (wasTap && state.goalPickArmed) {
    if (state.status?.control_mode === "manual") {
      setStatusMessage("当前是手动模式，先切到导航模式再选目标。", 2500);
    } else {
      const goal = canvasToWorld(x, y);
      state.pendingGoal = {
        x: goal.x,
        y: goal.y,
        yaw: deriveGoalYaw(goal),
      };
      state.goalPickArmed = false;
      updateStatusCards();
      renderScene();
      setStatusMessage("候选目标已选中，请点击“确认导航”后发送。", 2500);
    }
  }

  mapGesture = null;
}

canvas.addEventListener("pointerup", finishMapGesture);
canvas.addEventListener("pointercancel", finishMapGesture);

stopButton.addEventListener("click", () => {
  apiPost("/api/stop", {}).catch(() => {});
});

manualModeButton.addEventListener("click", async () => {
  try {
    await apiPost("/api/mode", { mode: "manual" });
    await apiPost("/api/stop", {});
    if (state.status) {
      state.status.control_mode = "manual";
    }
    setModeButtons("manual");
    setStatusMessage("已切到手动模式。", 1800);
    updateStatusCards();
  } catch (error) {
    setStatusMessage(`切换手动模式失败：${error.message}`, 3000);
  }
});

navModeButton.addEventListener("click", async () => {
  try {
    await apiPost("/api/mode", { mode: "nav" });
    resetJoystick();
    if (state.status) {
      state.status.control_mode = "nav";
    }
    setModeButtons("nav");
    setStatusMessage("已切到导航模式。", 1800);
    updateStatusCards();
  } catch (error) {
    setStatusMessage(`切换导航模式失败：${error.message}`, 3000);
  }
});

zoomInButton.addEventListener("click", () => {
  zoomMap(1.2);
});

zoomOutButton.addEventListener("click", () => {
  zoomMap(1 / 1.2);
});

fitMapButton.addEventListener("click", () => {
  resetMapView();
  setStatusMessage("地图视图已重置。", 1200);
});

pickGoalButton.addEventListener("click", () => {
  state.goalPickArmed = !state.goalPickArmed;
  if (state.goalPickArmed) {
    setStatusMessage("请轻点地图选择目标点，确认后才会开始导航。", 2500);
  } else {
    setStatusMessage("已退出选点。", 1200);
  }
  updateGoalControls();
});

clearGoalButton.addEventListener("click", () => {
  state.pendingGoal = null;
  state.goalPickArmed = false;
  updateStatusCards();
  renderScene();
  setStatusMessage("已清除候选目标。", 1200);
});

confirmGoalButton.addEventListener("click", async () => {
  if (!state.pendingGoal) {
    return;
  }
  if (state.status?.control_mode === "manual") {
    setStatusMessage("当前是手动模式，先切到导航模式再确认。", 2500);
    return;
  }

  const payload = { ...state.pendingGoal };
  try {
    await apiPost("/api/goal", payload);
    state.pendingGoal = null;
    updateStatusCards();
    renderScene();
    fetchStatus();
    setStatusMessage("导航目标已确认并发送。", 1800);
  } catch (error) {
    setStatusMessage(`目标发送失败：${error.message}`, 3000);
  }
});

refreshMapButton.addEventListener("click", () => {
  fetchMap().catch((error) => {
    setStatusMessage(`刷新地图失败：${error.message}`, 3000);
  });
});

function refreshCameraFrame() {
  if (!state.status?.devices?.camera) {
    cameraFrame.removeAttribute("src");
    cameraStatus.textContent = "等待相机";
    return;
  }
  cameraFrame.src = `/api/camera/frame.bmp?ts=${Date.now()}`;
}

function startCameraLoop() {
  if (cameraTimer) {
    clearInterval(cameraTimer);
  }
  cameraTimer = setInterval(refreshCameraFrame, 700);
}

window.addEventListener("resize", resizeCanvas);

resizeCanvas();
setModeButtons("nav");
fetchStatus();
fetchMap().catch(() => {});
fetchPath().catch(() => {});
startCameraLoop();
setInterval(fetchStatus, 600);
