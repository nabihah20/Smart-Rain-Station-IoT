/**
 * Firebase Cloud & AI Rain Monitoring System - Client Logic
 * Real-Time Telemetry, Remote Station Control, Audio Synthesis, and AI Predictor
 */

// Application State
const state = {
  drops: 0,
  tips: 0,
  remainingDrops: 0,
  depthMm: 0.0,
  rainRateMmH: 0.0,
  weatherStatus: 'Clear / Sunny',
  runState: 'RUNNING', // 'RUNNING' or 'STOPPED'
  audioEnabled: true,
  isFirebaseActive: false,
  isDemoMode: true,
  history: [],
  lastDropTime: Date.now(),
  lastTipTimestamp: null,
  dropsInLastMinute: 0,
  dropsPerTip: 126,
  mmPerTip: 0.20,
  activeChartTab: 'rate' // 'rate' or 'depth'
};

// Web Audio Context for realistic sound emulation
let audioCtx = null;

function initAudioContext() {
  if (!audioCtx) {
    const AudioContext = window.AudioContext || window.webkitAudioContext;
    if (AudioContext) {
      audioCtx = new AudioContext();
    }
  }
  if (audioCtx && audioCtx.state === 'suspended') {
    audioCtx.resume();
  }
}

// Synthesize 1200Hz raindrop splash
function playDropSound() {
  if (!state.audioEnabled || !audioCtx) return;
  try {
    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    const now = audioCtx.currentTime;

    osc.type = 'sine';
    osc.frequency.setValueAtTime(1100 + Math.random() * 250, now);
    osc.frequency.exponentialRampToValueAtTime(300, now + 0.08);

    gain.gain.setValueAtTime(0.15, now);
    gain.gain.exponentialRampToValueAtTime(0.001, now + 0.08);

    osc.connect(gain);
    gain.connect(audioCtx.destination);

    osc.start(now);
    osc.stop(now + 0.08);
  } catch (e) {
    console.warn("Audio drop error:", e);
  }
}

// Synthesize 2093Hz + 2793Hz metallic mechanical tip chime
function playTipChime() {
  if (!state.audioEnabled || !audioCtx) return;
  try {
    const now = audioCtx.currentTime;

    // Harmonic bell chime 1 (2093 Hz)
    const osc1 = audioCtx.createOscillator();
    const gain1 = audioCtx.createGain();
    osc1.type = 'sine';
    osc1.frequency.setValueAtTime(2093, now);
    gain1.gain.setValueAtTime(0.28, now);
    gain1.gain.exponentialRampToValueAtTime(0.001, now + 0.45);
    osc1.connect(gain1);
    gain1.connect(audioCtx.destination);
    osc1.start(now);
    osc1.stop(now + 0.45);

    // Harmonic overtone 2 (2793 Hz)
    const osc2 = audioCtx.createOscillator();
    const gain2 = audioCtx.createGain();
    osc2.type = 'sine';
    osc2.frequency.setValueAtTime(2793, now);
    gain2.gain.setValueAtTime(0.18, now);
    gain2.gain.exponentialRampToValueAtTime(0.001, now + 0.35);
    osc2.connect(gain2);
    gain2.connect(audioCtx.destination);
    osc2.start(now);
    osc2.stop(now + 0.35);
  } catch (e) {
    console.warn("Audio chime error:", e);
  }
}

// ==========================================
// METEOROLOGICAL CLASSIFIER & UI UPDATER
// ==========================================
function classifyWeather(rateMmH) {
  if (rateMmH <= 0.05) {
    return {
      status: 'Clear / Sunny',
      icon: '☀️',
      desc: 'No rainfall detected at station. Atmosphere dry and clear.',
      theme: 'clear',
      rainDensity: 0
    };
  } else if (rateMmH < 2.5) {
    return {
      status: 'Light Rain / Drizzle',
      icon: '🌦️',
      desc: 'Gentle mist and fine precipitation droplets accumulating slowly.',
      theme: 'drizzle',
      rainDensity: 25
    };
  } else if (rateMmH < 10.0) {
    return {
      status: 'Moderate Rain',
      icon: '🌧️',
      desc: 'Consistent steady rainfall. Standard tipping bucket activity.',
      theme: 'moderate',
      rainDensity: 65
    };
  } else if (rateMmH < 50.0) {
    return {
      status: 'Heavy Rain',
      icon: '⛈️',
      desc: 'Intense precipitation downpour. Rapid accumulation.',
      theme: 'heavy',
      rainDensity: 140
    };
  } else {
    return {
      status: 'Thunderstorm / Torrential',
      icon: '⚡',
      desc: 'Extreme rainfall intensity & convective storm alert. High runoff risk!',
      theme: 'storm',
      rainDensity: 240
    };
  }
}

function updateUI() {
  // Update Primary Numbers
  document.getElementById('tipsVal').textContent = state.tips;
  document.getElementById('dropsVal').textContent = state.remainingDrops;
  document.getElementById('depthVal').textContent = state.depthMm.toFixed(2);
  document.getElementById('rateVal').textContent = state.rainRateMmH.toFixed(1);

  // Update Progress Bars
  const dropPercent = ((state.remainingDrops / state.dropsPerTip) * 100).toFixed(1);
  document.getElementById('dropsProgress').style.width = `${dropPercent}%`;
  document.getElementById('dropsPercent').textContent = `${dropPercent}%`;

  const ratePercent = Math.min(100, (state.rainRateMmH / 60) * 100).toFixed(1);
  document.getElementById('rateProgress').style.width = `${ratePercent}%`;

  // Hero Weather Banner
  const weather = classifyWeather(state.rainRateMmH);
  state.weatherStatus = weather.status;
  document.getElementById('weatherIcon').textContent = weather.icon;
  document.getElementById('weatherTitle').textContent = weather.status;
  document.getElementById('weatherDesc').textContent = weather.desc;
  const heroRateEl = document.getElementById('heroRateVal');
  if (heroRateEl) heroRateEl.textContent = `${state.rainRateMmH.toFixed(1)} mm/h`;
  const heroTipRateEl = document.getElementById('heroTipRateVal');
  if (heroTipRateEl) {
    const tipsPerMin = (state.rainRateMmH / (state.mmPerTip * 60)).toFixed(1);
    heroTipRateEl.textContent = `${tipsPerMin} tips/min`;
  }
  currentRainDensity = weather.rainDensity;

  // Run/Stop Badges
  const stateBadge = document.getElementById('stateBadge');
  const btnRun = document.getElementById('btnRun');
  const btnStop = document.getElementById('btnStop');

  const isRunning = state.runState && state.runState.toUpperCase().startsWith('RUN');

  if (isRunning) {
    stateBadge.className = 'state-badge running';
    stateBadge.textContent = 'RUNNING';
    btnRun.disabled = true;
    btnRun.style.opacity = '0.5';
    btnStop.disabled = false;
    btnStop.style.opacity = '1';
  } else {
    stateBadge.className = 'state-badge stopped';
    stateBadge.textContent = 'STOPPED';
    btnRun.disabled = false;
    btnRun.style.opacity = '1';
    btnStop.disabled = true;
    btnStop.style.opacity = '0.5';
  }

  // AI Predictive Insights
  updateAIPredictions();
}

// ==========================================
// AI PREDICTIVE METEOROLOGICAL MODULE
// ==========================================
function updateAIPredictions() {
  const rate = state.rainRateMmH;
  let riskScore = 0;
  let riskTag = 'Minimal (0%)';
  let riskColor = 'var(--color-emerald)';
  let insight = 'Atmospheric conditions stable. No immediate runoff hazard.';

  if (rate <= 0.05) {
    riskScore = 5;
    riskTag = 'Low (5%)';
    riskColor = 'var(--color-emerald)';
    insight = 'Clear sky baseline. Dry ground absorption capacity is at 100%.';
  } else if (rate < 2.5) {
    riskScore = 18;
    riskTag = 'Low (18%)';
    riskColor = 'var(--color-emerald)';
    insight = 'Light precipitation will absorb naturally without pooling.';
  } else if (rate < 10.0) {
    riskScore = 48;
    riskTag = 'Moderate (48%)';
    riskColor = 'var(--color-amber)';
    insight = 'Sustained rain may saturate topsoil within 45 minutes.';
  } else if (rate < 50.0) {
    riskScore = 78;
    riskTag = 'High Alert (78%)';
    riskColor = 'var(--color-crimson)';
    insight = 'AI Trend: Heavy downpour. Localized surface runoff probable in 15 mins.';
  } else {
    riskScore = 95;
    riskTag = 'CRITICAL (95%)';
    riskColor = '#dc2626';
    insight = 'Severe storm intensity! Flash flood risk elevated. Immediate drainage alert.';
  }

  document.getElementById('aiRiskLevel').textContent = riskTag;
  document.getElementById('aiRiskLevel').style.color = riskColor;
  document.getElementById('aiRiskProgress').style.width = `${riskScore}%`;
  document.getElementById('aiRiskProgress').style.backgroundColor = riskColor;
  document.getElementById('aiInsightText').textContent = insight;

  // Pattern Anomaly
  const isSteady = Math.abs(state.rainRateMmH - 5.0) < 3.0;
  document.getElementById('aiAnomalyVal').textContent = isSteady ? 'Natural Poisson Curve' : 'Convective Burst';
}

// ==========================================
// REMOTE COMMAND DISPATCHER
// ==========================================
function dispatchRemoteCommand(commandType) {
  const timestamp = new Date().toLocaleTimeString();
  const logEl = document.getElementById('commandLogText');

  logEl.textContent = `[${timestamp}] Sent ${commandType} to station...`;

  if (commandType === 'RUN') {
    state.runState = 'RUNNING';
  } else if (commandType === 'STOP') {
    state.runState = 'STOPPED';
  } else if (commandType === 'RESET') {
    state.drops = 0;
    state.tips = 0;
    state.remainingDrops = 0;
    state.depthMm = 0.0;
    state.rainRateMmH = 0.0;
  } else if (commandType === 'TIP_NOW') {
    // Fast forward to next tip
    const dropsToTip = state.dropsPerTip - state.remainingDrops;
    for (let i = 0; i < dropsToTip; i++) {
      simulateSingleDrop(false);
    }
    playTipChime();
    updateUI();
  }

  updateUI();

  // If Firebase Realtime Database is connected, write to /rain_station/control
  if (state.isFirebaseActive && window.firebase && window.firebase.database) {
    try {
      const db = window.firebase.database();
      db.ref(window.DB_PATHS.CONTROL).set({
        command: commandType,
        handled: false,
        timestamp: Date.now()
      }).then(() => {
        logEl.textContent = `[${timestamp}] Station ACK: ${commandType} verified.`;
      }).catch(err => {
        logEl.textContent = `[${timestamp}] Firebase Error: ${err.message}`;
      });
    } catch (e) {
      console.error("Firebase write error:", e);
    }
  } else {
    // Local simulated station confirmation
    setTimeout(() => {
      logEl.textContent = `[${timestamp}] Local Gateway ACK: ${commandType} applied.`;
    }, 400);
  }
}

// ==========================================
// LOCAL RAIN SIMULATOR (Fallback / Demo Mode)
// ==========================================
let simInterval = null;

function simulateSingleDrop(withSound = true) {
  if (state.runState === 'STOPPED') return;

  state.drops++;
  const oldTips = state.tips;
  state.tips = Math.floor(state.drops / state.dropsPerTip);
  state.remainingDrops = state.drops % state.dropsPerTip;
  state.depthMm = state.tips * state.mmPerTip;

  // Rate calculation (drops in current minute)
  state.dropsInLastMinute++;

  const isTipEvent = (state.tips > oldTips);

  if (isTipEvent) {
    if (withSound) playTipChime();
  } else {
    if (withSound) playDropSound();
  }

  updateUI();
}

function startDemoSimulator() {
  if (simInterval) clearInterval(simInterval);

  // Dynamic natural rain distribution
  const triggerNext = () => {
    if (state.runState === 'RUNNING') {
      simulateSingleDrop(true);
    }
    let nextDelay = Math.random() * 2200 + 800; // 0.8s to 3.0s
    simInterval = setTimeout(triggerNext, nextDelay);
  };
  triggerNext();

  // Recalculate rain rate every 5 seconds
  setInterval(() => {
    // mm/hr = (drops * 0.20 / 126) * (3600 / 5)
    const mmIn5Sec = state.dropsInLastMinute * (state.mmPerTip / state.dropsPerTip);
    state.rainRateMmH = (state.runState === 'RUNNING') ? (mmIn5Sec * 720) : 0.0;
    state.dropsInLastMinute = 0;

    // Add to chart
    pushChartData(state.rainRateMmH, state.depthMm);
    updateUI();
  }, 5000);
}

// ==========================================
// CHART.JS INTEGRATION
// ==========================================
let rainChart = null;

function initChart() {
  const ctx = document.getElementById('rainHistoryChart').getContext('2d');

  const gradientRate = ctx.createLinearGradient(0, 0, 0, 300);
  gradientRate.addColorStop(0, 'rgba(139, 92, 246, 0.45)');
  gradientRate.addColorStop(1, 'rgba(139, 92, 246, 0.0)');

  const gradientDepth = ctx.createLinearGradient(0, 0, 0, 300);
  gradientDepth.addColorStop(0, 'rgba(16, 185, 129, 0.45)');
  gradientDepth.addColorStop(1, 'rgba(16, 185, 129, 0.0)');

  const initialLabels = [];
  const initialData = [];
  const now = new Date();

  for (let i = 12; i >= 0; i--) {
    const t = new Date(now.getTime() - i * 15000);
    initialLabels.push(t.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' }));
    initialData.push(0);
  }

  rainChart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: initialLabels,
      datasets: [{
        label: 'Rainfall Rate (mm/h)',
        data: initialData,
        borderColor: '#8b5cf6',
        borderWidth: 2.5,
        backgroundColor: gradientRate,
        fill: true,
        tension: 0.35,
        pointRadius: 3,
        pointBackgroundColor: '#8b5cf6'
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      plugins: {
        legend: { display: false }
      },
      scales: {
        x: {
          grid: { color: 'rgba(255, 255, 255, 0.05)' },
          ticks: { color: '#64748b', font: { family: 'JetBrains Mono', size: 10 } }
        },
        y: {
          grid: { color: 'rgba(255, 255, 255, 0.05)' },
          ticks: { color: '#64748b', font: { family: 'JetBrains Mono', size: 10 } },
          beginAtZero: true
        }
      }
    }
  });
}

function pushChartData(rate, depth) {
  if (!rainChart) return;

  const now = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
  rainChart.data.labels.push(now);

  const val = (state.activeChartTab === 'rate') ? rate : depth;
  rainChart.data.datasets[0].data.push(val);

  if (rainChart.data.labels.length > 20) {
    rainChart.data.labels.shift();
    rainChart.data.datasets[0].data.shift();
  }

  rainChart.update();
}

function switchChartTab(tab) {
  state.activeChartTab = tab;
  document.getElementById('tabRate').className = (tab === 'rate') ? 'chart-tab active' : 'chart-tab';
  document.getElementById('tabDepth').className = (tab === 'depth') ? 'chart-tab active' : 'chart-tab';

  if (!rainChart) return;

  if (tab === 'rate') {
    rainChart.data.datasets[0].label = 'Rainfall Rate (mm/h)';
    rainChart.data.datasets[0].borderColor = '#8b5cf6';
    rainChart.data.datasets[0].pointBackgroundColor = '#8b5cf6';
  } else {
    rainChart.data.datasets[0].label = 'Precipitation Depth (mm)';
    rainChart.data.datasets[0].borderColor = '#10b981';
    rainChart.data.datasets[0].pointBackgroundColor = '#10b981';
  }
  rainChart.update();
}

// ==========================================
// BACKGROUND RAIN CANVAS PARTICLE ENGINE
// ==========================================
const canvas = document.getElementById('rainCanvas');
const ctx = canvas.getContext('2d');
let drops = [];
let currentRainDensity = 25;

function resizeCanvas() {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;
}

function initRainParticles() {
  drops = [];
  for (let i = 0; i < 180; i++) {
    drops.push({
      x: Math.random() * canvas.width,
      y: Math.random() * canvas.height,
      length: Math.random() * 20 + 10,
      speed: Math.random() * 8 + 8,
      opacity: Math.random() * 0.4 + 0.1
    });
  }
}

function animateRain() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const activeCount = Math.floor((currentRainDensity / 250) * drops.length);

  for (let i = 0; i < activeCount; i++) {
    const d = drops[i];
    ctx.beginPath();
    ctx.strokeStyle = `rgba(56, 189, 248, ${d.opacity})`;
    ctx.lineWidth = 1.2;
    ctx.moveTo(d.x, d.y);
    ctx.lineTo(d.x + 1.5, d.y + d.length);
    ctx.stroke();

    d.y += d.speed;
    d.x += 1.5;

    if (d.y > canvas.height) {
      d.y = -d.length;
      d.x = Math.random() * canvas.width;
    }
  }

  requestAnimationFrame(animateRain);
}

// ==========================================
// FIREBASE REALTIME DB INITIALIZATION
// ==========================================
function initFirebase() {
  if (window.FIREBASE_CONFIG && window.FIREBASE_CONFIG.databaseURL && !window.FIREBASE_CONFIG.databaseURL.includes("YOUR_PROJECT_ID") && window.firebase) {
    try {
      firebase.initializeApp(window.FIREBASE_CONFIG);
      const db = firebase.database();
      state.isFirebaseActive = true;
      state.isDemoMode = false;

      document.getElementById('cloudStatusPill').innerHTML = '<span class="pill-dot online"></span> Firebase Cloud Connected';

      // Track previous values to detect incoming drop/tip increments
      let isFirstTelemetry = true;

      // Listen for incoming telemetry from ESP32-S3
      db.ref(window.DB_PATHS.TELEMETRY).on('value', snapshot => {
        const val = snapshot.val();
        if (val) {
          const oldDrops = state.drops;
          const oldTips = state.tips;

          state.drops = val.drops || 0;
          state.tips = val.tips || 0;
          state.remainingDrops = val.remaining_drops || 0;
          state.depthMm = val.depth_mm || 0.0;
          state.rainRateMmH = val.rain_rate_mmh || 0.0;
          state.runState = val.run_state || 'RUNNING';
          state.weatherStatus = val.weather_status || 'Clear';

          // Play real-time synthesized acoustic drops and tip chimes from cloud
          if (!isFirstTelemetry && state.audioEnabled) {
            if (state.tips > oldTips) {
              playTipChime();
            } else if (state.drops > oldDrops) {
              playDropSound();
            }
          }
          isFirstTelemetry = false;

          updateUI();
          pushChartData(state.rainRateMmH, state.depthMm);
        }
      });

      console.log("[FIREBASE] Connected to Realtime Database successfully!");
    } catch (e) {
      console.warn("[FIREBASE] Initialization skipped, running in Demo Mode:", e.message);
    }
  } else {
    console.log("[STATUS] Running in High-Fidelity Demo/Emulation Mode. Configure firebase-config.js for live cloud.");
  }
}

// ==========================================
// EVENT LISTENERS & SETUP
// ==========================================
window.addEventListener('DOMContentLoaded', () => {
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas);
  initRainParticles();
  animateRain();

  initChart();
  updateUI();
  initFirebase();

  // If not live on Firebase, start interactive local simulator
  if (!state.isFirebaseActive) {
    startDemoSimulator();
  }

  // Live Station Time ticker
  setInterval(() => {
    document.getElementById('stationTime').textContent = new Date().toLocaleTimeString();
  }, 1000);

  // Audio Toggle Functionality (ON / OFF)
  const audioToggleBtn = document.getElementById('audioToggle');

  function updateAudioButtonUI() {
    if (state.audioEnabled) {
      audioToggleBtn.innerHTML = '🔊 Sound: ON';
      audioToggleBtn.className = 'splash-indicator active';
      audioToggleBtn.title = 'Sound is ON. Click to turn OFF';
    } else {
      audioToggleBtn.innerHTML = '🔇 Sound: OFF';
      audioToggleBtn.className = 'splash-indicator muted';
      audioToggleBtn.title = 'Sound is OFF. Click to turn ON';
    }
  }

  // Initialize button appearance
  updateAudioButtonUI();

  audioToggleBtn.addEventListener('click', (e) => {
    e.stopPropagation();
    initAudioContext();

    // Toggle sound state
    state.audioEnabled = !state.audioEnabled;
    updateAudioButtonUI();

    // If turned ON, play confirmation droplet sound so user hears it working!
    if (state.audioEnabled) {
      playDropSound();
    }
  });

  // Remote Control Buttons
  document.getElementById('btnRun').addEventListener('click', () => {
    initAudioContext();
    dispatchRemoteCommand('RUN');
  });

  document.getElementById('btnStop').addEventListener('click', () => {
    initAudioContext();
    dispatchRemoteCommand('STOP');
  });

  document.getElementById('btnTipNow').addEventListener('click', () => {
    initAudioContext();
    dispatchRemoteCommand('TIP_NOW');
  });

  document.getElementById('btnReset').addEventListener('click', () => {
    if (confirm("Reset station rain and tips counters to 0?")) {
      dispatchRemoteCommand('RESET');
    }
  });

  // Chart Switchers
  document.getElementById('tabRate').addEventListener('click', () => switchChartTab('rate'));
  document.getElementById('tabDepth').addEventListener('click', () => switchChartTab('depth'));

  // Settings Modal Handlers
  document.getElementById('btnSettings').addEventListener('click', () => {
    document.getElementById('settingsModal').classList.add('active');
  });

  document.getElementById('btnCancelSettings').addEventListener('click', () => {
    document.getElementById('settingsModal').classList.remove('active');
  });

  document.getElementById('btnSaveSettings').addEventListener('click', () => {
    const url = document.getElementById('cfgDbUrl').value.trim();
    if (url) {
      window.FIREBASE_CONFIG.databaseURL = url;
      alert("Firebase configuration saved! Please reload page to connect.");
    }
    document.getElementById('settingsModal').classList.remove('active');
  });

  // First user interaction initializes Web Audio
  document.body.addEventListener('click', () => initAudioContext(), { once: true });
});
