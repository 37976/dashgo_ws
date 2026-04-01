const state = {
  mapVersion: -1,
  pathVersion: -1,
  status: null,
  map: null,
  path: [],
  pendingGoal: null,
};

const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");
const statusLine = document.getElementById("status-line");
const poseText = document.getElementById("pose-text");
const velocityText = document.getElementById("velocity-text");
const mapText = document.getElementById("map-text");
const goalText = document.getElementById("goal-text");

const joystickBase = document.getElementById("joystick-base");
const joystickKnob = document.getElementById("joystick-knob");
const stopButton = document.getElementById("stop-button");
const refreshMapButton = document.getElementById("refresh-map-button");
const manualModeButton = document.getElementById("manual-mode-button");
const navModeButton = document.getElementById("nav-mode-button");
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
let mapPointerState = null;
let mapImage = null;
let cameraTimer = null;

function clamp(value, low, high) {
  return Math.max(low, Math.min(high, value));
}

function formatNumber(value) {
  return Number.isFinite(value) ? value.toFixed(2) : "--";
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
  const size = Math.max(320, Math.floor(canvas.clientWidth));
  canvas.width = size;
  canvas.height = size;
  renderScene();
}

function setModeButtons(mode) {
  manualModeButton.classList.toggle("active", mode === "manual");
  navModeButton.classList.toggle("active", mode === "nav");
}

function setStatusDot(element, isOnline, onlineClass = "online") {
  element.classList.remove("online", "offline", "manual", "nav");
  element.classList.add(isOnline ? onlineClass : "offline");
}

function worldToCanvas(x, y) {
  if (!state.map) {
    return { x: 0, y: 0 };
  }
  const { width, height, resolution, origin } = state.map;
  const px = (x - origin.x) / resolution;
  const py = (y - origin.y) / resolution;
  return {
    x: (px / width) * canvas.width,
    y: canvas.height - (py / height) * canvas.height,
  };
}

function canvasToWorld(x, y) {
  if (!state.map) {
    return { x: 0, y: 0 };
  }
  const { width, height, resolution, origin } = state.map;
  return {
    x: origin.x + (x / canvas.width) * width * resolution,
    y: origin.y + ((canvas.height - y) / canvas.height) * height * resolution,
  };
}

function rebuildMapImage() {
  if (!state.map) {
    mapImage = null;
    return;
  }

  const { width, height, data } = state.map;
  const imageData = new ImageData(width, height);
  for (let i = 0; i < data.length; i += 1) {
    const occupancy = data[i];
    let color = 255;
    if (occupancy < 0) {
      color = 225;
    } else if (occupancy >= 100) {
      color = 20;
    } else if (occupancy > 0) {
      color = 255 - Math.round((occupancy / 100.0) * 170);
    }
    const index = i * 4;
    imageData.data[index] = color;
    imageData.data[index + 1] = color;
    imageData.data[index + 2] = color;
    imageData.data[index + 3] = 255;
  }

  const offscreen = document.createElement("canvas");
  offscreen.width = width;
  offscreen.height = height;
  offscreen.getContext("2d").putImageData(imageData, 0, 0);
  mapImage = offscreen;
}

function drawRobot() {
  if (!state.status?.odom || !state.map) {
    return;
  }

  const pose = state.status.odom;
  const p = worldToCanvas(pose.x, pose.y);
  const radiusPx = Math.max(
    8,
    (state.status.robot_radius / state.map.resolution) * (canvas.width / state.map.width),
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
  ctx.moveTo(radiusPx * 0.85, 0);
  ctx.lineTo(-radiusPx * 0.2, radiusPx * 0.55);
  ctx.lineTo(-radiusPx * 0.2, -radiusPx * 0.55);
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
  ctx.save();
  ctx.strokeStyle = "#d64b28";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(p.x, p.y, 10, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(p.x - 14, p.y);
  ctx.lineTo(p.x + 14, p.y);
  ctx.moveTo(p.x, p.y - 14);
  ctx.lineTo(p.x, p.y + 14);
  ctx.stroke();

  if (Number.isFinite(goal.yaw)) {
    ctx.translate(p.x, p.y);
    ctx.rotate(-goal.yaw);
    ctx.fillStyle = "#d64b28";
    ctx.beginPath();
    ctx.moveTo(18, 0);
    ctx.lineTo(4, 6);
    ctx.lineTo(4, -6);
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();
}

function drawPendingGoalGuide() {
  if (!mapPointerState?.current) {
    return;
  }

  const start = worldToCanvas(mapPointerState.start.x, mapPointerState.start.y);
  const current = worldToCanvas(mapPointerState.current.x, mapPointerState.current.y);
  ctx.save();
  ctx.strokeStyle = "#ff8a00";
  ctx.lineWidth = 2;
  ctx.setLineDash([8, 6]);
  ctx.beginPath();
  ctx.moveTo(start.x, start.y);
  ctx.lineTo(current.x, current.y);
  ctx.stroke();
  ctx.restore();
}

function renderScene() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  if (mapImage) {
    ctx.save();
    ctx.translate(0, canvas.height);
    ctx.scale(1, -1);
    ctx.drawImage(mapImage, 0, 0, canvas.width, canvas.height);
    ctx.restore();
  } else {
    ctx.fillStyle = "#edf3f9";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
  }

  drawPath();
  drawGoal();
  drawPendingGoalGuide();
  drawRobot();
}

async function fetchStatus() {
  try {
    const payload = await apiGet("/api/status");
    state.status = payload;
    statusLine.textContent = payload.has_odom
      ? "已连接机器人，手机可直接控制与点目标导航"
      : "已连网页服务，等待机器人位姿";

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
    statusLine.textContent = `连接失败：${error.message}`;
  }
}

async function fetchMap() {
  const payload = await apiGet("/api/map");
  if (!payload.ok) {
    return;
  }
  state.map = payload.map;
  rebuildMapImage();
  updateStatusCards();
  renderScene();
}

async function fetchPath() {
  const payload = await apiGet("/api/path");
  if (!payload.ok) {
    return;
  }
  state.path = payload.path;
  renderScene();
}

function updateStatusCards() {
  const odom = state.status?.odom;
  const goal = state.status?.goal;
  const mode = state.status?.control_mode || "nav";
  const devices = state.status?.devices || {};
  poseText.textContent = odom
    ? `x ${formatNumber(odom.x)}  y ${formatNumber(odom.y)}  yaw ${formatNumber(odom.yaw)}`
    : "--";
  velocityText.textContent = odom
    ? `v ${formatNumber(odom.linear_x)}  w ${formatNumber(odom.angular_z)}`
    : "--";
  mapText.textContent = state.map
    ? `${state.map.width}×${state.map.height} @ ${formatNumber(state.map.resolution)}m`
    : "等待地图";
  goalText.textContent = goal
    ? `x ${formatNumber(goal.x)}  y ${formatNumber(goal.y)}`
    : "未设置";
  if (state.status?.has_camera && state.status?.camera) {
    cameraStatus.textContent = `${state.status.camera.width}×${state.status.camera.height}`;
  } else {
    cameraStatus.textContent = "等待相机";
  }

  modeText.textContent = mode === "manual" ? "模式 手动" : "模式 导航";
  setStatusDot(modeDot, true, mode === "manual" ? "manual" : "nav");
  setStatusDot(radarDot, Boolean(devices.radar));
  setStatusDot(baseDot, Boolean(devices.base));
  setStatusDot(cameraDot, Boolean(devices.camera));
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
  lastCmd = { linear, angular };
  if (cmdVelTimer) {
    return;
  }
  cmdVelTimer = setTimeout(async () => {
    cmdVelTimer = null;
    try {
      await apiPost("/api/cmd_vel", lastCmd);
    } catch (error) {
      statusLine.textContent = `速度指令发送失败：${error.message}`;
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

canvas.addEventListener("pointerdown", (event) => {
  if (!state.map) {
    return;
  }
  const rect = canvas.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  mapPointerState = {
    start: canvasToWorld(x, y),
    current: canvasToWorld(x, y),
  };
  canvas.setPointerCapture(event.pointerId);
  renderScene();
});

canvas.addEventListener("pointermove", (event) => {
  if (!mapPointerState || !state.map) {
    return;
  }
  const rect = canvas.getBoundingClientRect();
  mapPointerState.current = canvasToWorld(event.clientX - rect.left, event.clientY - rect.top);
  renderScene();
});

async function releaseMapGoal(event) {
  if (!mapPointerState) {
    return;
  }

  if (state.status?.control_mode === "manual") {
    statusLine.textContent = "当前是手动模式，先切到导航模式再点目标。";
    mapPointerState = null;
    renderScene();
    return;
  }

  const start = mapPointerState.start;
  const current = mapPointerState.current || start;
  const yaw = Math.atan2(current.y - start.y, current.x - start.x);
  state.pendingGoal = { x: start.x, y: start.y, yaw };
  renderScene();

  try {
    await apiPost("/api/goal", state.pendingGoal);
    state.pendingGoal = null;
    fetchStatus();
  } catch (error) {
    statusLine.textContent = `目标发送失败：${error.message}`;
  } finally {
    mapPointerState = null;
    renderScene();
  }
}

canvas.addEventListener("pointerup", releaseMapGoal);
canvas.addEventListener("pointercancel", releaseMapGoal);

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
    statusLine.textContent = "已切到手动模式";
  } catch (error) {
    statusLine.textContent = `切换手动模式失败：${error.message}`;
  }
});

navModeButton.addEventListener("click", async () => {
  try {
    await apiPost("/api/mode", { mode: "nav" });
    if (state.status) {
      state.status.control_mode = "nav";
    }
    setModeButtons("nav");
    statusLine.textContent = "已切到导航模式";
  } catch (error) {
    statusLine.textContent = `切换导航模式失败：${error.message}`;
  }
});

refreshMapButton.addEventListener("click", () => {
  fetchMap().catch((error) => {
    statusLine.textContent = `刷新地图失败：${error.message}`;
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
fetchStatus();
fetchMap().catch(() => {});
fetchPath().catch(() => {});
startCameraLoop();
setInterval(fetchStatus, 600);
