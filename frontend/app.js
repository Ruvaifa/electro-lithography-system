/* frontend/app.js */

const API_BASE = ""; // Relative path to local server

// State tracking
let isConnected = false;
let isPatterning = false;
let selectedStep = 0.2;
let statusInterval = null;


// DOM Elements
const systemStatusBadge = document.getElementById("system-status");
const systemStatusText = systemStatusBadge.querySelector(".status-text");

const btnConnect = document.getElementById("btn-connect");
const btnHome = document.getElementById("btn-home");
const btnEmergencyStop = document.getElementById("btn-emergency-stop");
const btnStartPattern = document.getElementById("btn-start-pattern");

const portXSelect = document.getElementById("port-x");
const portYSelect = document.getElementById("port-y");
const portZSelect = document.getElementById("port-z");

const posXText = document.getElementById("pos-x");
const posYText = document.getElementById("pos-y");
const posZText = document.getElementById("pos-z");

const telVoltage = document.getElementById("tel-voltage");
const telCurrent = document.getElementById("tel-current");
const telFeedbackDir = document.getElementById("tel-feedback-direction");
const telMovesDone = document.getElementById("tel-moves-done");
const telMovesTotal = document.getElementById("tel-moves-total");
const telMovesLeft = document.getElementById("tel-moves-left");
const progressBar = document.getElementById("run-progress-bar");

const paramMode = document.getElementById("param-mode");
const paramFile = document.getElementById("param-file");
const paramDisableRamps = document.getElementById("param-disable_ramps");

// Parameter inputs
const paramContactVolt = document.getElementById("param-contact_voltage");
const paramContactComp = document.getElementById("param-contact_compliance");
const paramThresholdCurr = document.getElementById("param-threshold_current");
const paramVoltSource = document.getElementById("param-volt_source");
const paramCurrComp = document.getElementById("param-curr_comp");
const paramVoltageThr1 = document.getElementById("param-voltage_threshold_1");
const paramVoltageThr2 = document.getElementById("param-voltage_threshold_2");
const paramDeltaZ = document.getElementById("param-delta_z");
const paramZContactStep = document.getElementById("param-z_contact_step");
const paramSpeed = document.getElementById("param-speed");
const paramZFeedbackSpeed = document.getElementById("param-z_feedback_speed");
const paramLiftoffHeight = document.getElementById("param-liftoff_height");
const paramMaxSafeZMargin = document.getElementById("param-max_safe_z_margin");
const paramSampleInterval = document.getElementById("param-sample_interval");

const btnThemeToggle = document.getElementById("btn-theme-toggle");
const themeIcon = document.getElementById("theme-icon");

// Real-time canvas chart state
const canvas = document.getElementById("telemetry-canvas");
const ctx = canvas ? canvas.getContext("2d") : null;
let chartData = [];
let patterningStartTime = null;

// Theme toggling logic
const SUN_SVG = `<circle cx="12" cy="12" r="5"></circle><line x1="12" y1="1" x2="12" y2="3"></line><line x1="12" y1="21" x2="12" y2="23"></line><line x1="4.22" y1="4.22" x2="5.64" y2="5.64"></line><line x1="18.36" y1="18.36" x2="19.78" y2="19.78"></line><line x1="1" y1="12" x2="3" y2="12"></line><line x1="21" y1="12" x2="23" y2="12"></line><line x1="4.22" y1="19.78" x2="5.64" y2="18.36"></line><line x1="18.36" y1="5.64" x2="19.78" y2="4.22"></line>`;
const MOON_SVG = `<path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"></path>`;

function toggleTheme() {
    const currentTheme = document.documentElement.getAttribute("data-theme");
    if (currentTheme === "dark") {
        document.documentElement.setAttribute("data-theme", "light");
        themeIcon.innerHTML = MOON_SVG;
        localStorage.setItem("theme", "light");
        logToConsole("[SYSTEM] Switched to Light Theme");
    } else {
        document.documentElement.setAttribute("data-theme", "dark");
        themeIcon.innerHTML = SUN_SVG;
        localStorage.setItem("theme", "dark");
        logToConsole("[SYSTEM] Switched to Dark Theme");
    }
}

function loadThemePreference() {
    const savedTheme = localStorage.getItem("theme") || "light";
    document.documentElement.setAttribute("data-theme", savedTheme);
    if (savedTheme === "dark") {
        themeIcon.innerHTML = SUN_SVG;
    } else {
        themeIcon.innerHTML = MOON_SVG;
    }
}

btnThemeToggle.addEventListener("click", toggleTheme);

// Setup accordion behavior
document.querySelectorAll(".accordion-header").forEach(header => {
    header.addEventListener("click", () => {
        const item = header.parentElement;
        item.classList.toggle("active");
    });
});

// Setup jog step selection
document.querySelectorAll(".step-btn").forEach(btn => {
    btn.addEventListener("click", () => {
        document.querySelectorAll(".step-btn").forEach(b => b.classList.remove("active"));
        btn.classList.add("active");
        selectedStep = parseFloat(btn.dataset.step);
        logToConsole(`[SYSTEM] Jog step size changed to ${selectedStep} µm`);
    });
});

// INITIALIZE APP
window.addEventListener("DOMContentLoaded", () => {
    loadThemePreference();
    scanPorts();
    updateUIState();
    
    // Start regular state check (once every 1 second when idle, faster when running)
    setInterval(checkSystemStatus, 1500);
});

// SCAN SERIAL PORTS
async function scanPorts() {
    try {
        const res = await fetch(`${API_BASE}/api/ports`);
        const ports = await res.json();
        
        const selectors = [portXSelect, portYSelect, portZSelect];
        selectors.forEach(select => {
            select.innerHTML = "";
            if (ports.length === 0) {
                select.innerHTML = '<option value="">No ports found</option>';
                return;
            }
            ports.forEach((p, idx) => {
                const opt = document.createElement("option");
                opt.value = p.device;
                opt.textContent = `${p.device} (${p.description})`;
                select.appendChild(opt);
            });
        });
        
        // Auto-select defaults if there are at least 3 ports (fallback)
        if (ports.length >= 1) portXSelect.value = ports[0].device;
        if (ports.length >= 2) portYSelect.value = ports[1].device;
        if (ports.length >= 3) portZSelect.value = ports[2].device;

        // Custom defaults (COM3 for X, COM10 for Y, COM8 for Z) override fallback if available
        ports.forEach(p => {
            const dev = p.device.toUpperCase();
            if (dev === "COM3" || dev === "COM3" || dev === "3") portXSelect.value = p.device;
            if (dev === "COM10" || dev === "COM10" || dev === "10") portYSelect.value = p.device;
            if (dev === "COM8" || dev === "COM8" || dev === "8") portZSelect.value = p.device;
        });
        
    } catch (err) {
        logToConsole(`[ERROR] Failed to scan COM ports: ${err.message}`, true);
    }
}



// TOGGLE ACCORDION CONTROLS BASED ON SELECTED MODE
paramMode.addEventListener("change", () => {
    const mode = parseInt(paramMode.value);
    const zSpeedGroup = document.getElementById("group-z-feedback-speed");
    const intervalGroup = document.getElementById("group-sample-interval");
    
    if (mode === 6) {
        zSpeedGroup.style.opacity = "0.3";
        zSpeedGroup.querySelector("input").disabled = true;
        intervalGroup.style.opacity = "0.3";
        intervalGroup.querySelector("input").disabled = true;
    } else {
        zSpeedGroup.style.opacity = "1";
        zSpeedGroup.querySelector("input").disabled = false;
        intervalGroup.style.opacity = "1";
        intervalGroup.querySelector("input").disabled = false;
    }
});

// CONNECT BUTTON
btnConnect.addEventListener("click", async () => {
    if (isConnected) {
        // Disconnect
        btnConnect.disabled = true;
        logToConsole("[SYSTEM] Disconnecting serial ports...");
        try {
            const res = await fetch(`${API_BASE}/api/disconnect`, { method: "POST" });
            const data = await res.json();
            if (data.success) {
                isConnected = false;
                logToConsole("[SYSTEM] Disconnected.");
            }
        } catch (err) {
            logToConsole(`[ERROR] Disconnect failed: ${err.message}`, true);
        }
        btnConnect.disabled = false;
        updateUIState();
    } else {
        // Connect
        btnConnect.disabled = true;
        const x_port = portXSelect.value;
        const y_port = portYSelect.value;
        const z_port = portZSelect.value;
        
        if (!x_port || !y_port || !z_port) {
            logToConsole("[ERROR] Please select COM ports for X, Y, and Z axes.", true);
            btnConnect.disabled = false;
            return;
        }
        
        logToConsole(`[SYSTEM] Connecting to X=${x_port}, Y=${y_port}, Z=${z_port}...`);
        try {
            const res = await fetch(`${API_BASE}/api/connect`, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ x_port, y_port, z_port })
            });
            const data = await res.json();
            if (data.success) {
                isConnected = true;
                logToConsole("[SYSTEM] Successfully connected to controller axes!");
            } else {
                logToConsole(`[ERROR] Connection failed: ${data.error}`, true);
            }
        } catch (err) {
            logToConsole(`[ERROR] Connection error: ${err.message}`, true);
        }
        btnConnect.disabled = false;
        updateUIState();
    }
});

// HOME BUTTON
btnHome.addEventListener("click", async () => {
    btnHome.disabled = true;
    logToConsole("[SYSTEM] Initiating homing sequence. Please wait...");
    try {
        const res = await fetch(`${API_BASE}/api/home`, { method: "POST" });
        const data = await res.json();
        if (data.success) {
            logToConsole("[SYSTEM] Homing completed successfully.");
            updatePositions(data.position);
        } else {
            logToConsole(`[ERROR] Homing sequence aborted: ${data.error}`, true);
        }
    } catch (err) {
        logToConsole(`[ERROR] Homing error: ${err.message}`, true);
    }
    btnHome.disabled = false;
});

// EMERGENCY STOP BUTTON
btnEmergencyStop.addEventListener("click", async () => {
    logToConsole("[ABORT] Emergency stop triggered! Stopping all stage drives...", true);
    try {
        const res = await fetch(`${API_BASE}/api/stop`, { method: "POST" });
        const data = await res.json();
        if (data.success) {
            logToConsole("[SYSTEM] Emergency stop active.");
        }
    } catch (err) {
        logToConsole(`[ERROR] Stop command error: ${err.message}`, true);
    }
});

// JOG BUTTON EVENT LISTENERS
const JOG_MAPPINGS = {
    "jog-x-plus": { x: 1, y: 0, z: 0 },
    "jog-x-minus": { x: -1, y: 0, z: 0 },
    "jog-y-plus": { x: 0, y: 1, z: 0 },
    "jog-y-minus": { x: 0, y: -1, z: 0 },
    "jog-z-plus": { x: 0, y: 0, z: 1 },
    "jog-z-minus": { x: 0, y: 0, z: -1 }
};

Object.keys(JOG_MAPPINGS).forEach(id => {
    const btn = document.getElementById(id);
    btn.addEventListener("click", () => jogAxis(JOG_MAPPINGS[id]));
});

async function jogAxis(deltas) {
    if (!isConnected || isPatterning) return;
    
    // Scale deltas by the selected step size
    const moveX = deltas.x * selectedStep;
    const moveY = deltas.y * selectedStep;
    const moveZ = deltas.z * selectedStep;
    
    logToConsole(`[SYSTEM] Jogging: dX=${moveX} µm, dY=${moveY} µm, dZ=${moveZ} µm`);
    try {
        const res = await fetch(`${API_BASE}/api/move`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ x: moveX, y: moveY, z: moveZ })
        });
        const data = await res.json();
        if (data.success) {
            updatePositions(data.position);
        } else {
            logToConsole(`[ERROR] Jog failed: ${data.error}`, true);
        }
    } catch (err) {
        logToConsole(`[ERROR] Jog error: ${err.message}`, true);
    }
}

// START PATTERNING RUN
// START PATTERNING RUN
btnStartPattern.addEventListener("click", async () => {
    if (isPatterning) {
        // Stop current patterning
        btnStartPattern.disabled = true;
        logToConsole("[SYSTEM] Stopping active patterning run...", true);
        try {
            await fetch(`${API_BASE}/api/stop`, { method: "POST" });
        } catch (err) {}
        btnStartPattern.disabled = false;
        return;
    }
    
    // Check if file is selected
    if (paramFile.files.length === 0) {
        logToConsole("[ERROR] Please select a coordinate pattern file (.txt) first.", true);
        return;
    }
    const file = paramFile.files[0];
    
    // Disable run button while loading
    btnStartPattern.disabled = true;
    logToConsole(`[SYSTEM] Reading pattern file: ${file.name}...`);
    
    const reader = new FileReader();
    reader.onload = function(e) {
        const fileContent = e.target.result;
        
        // Gather parameters
        const params = {
            mode: parseInt(paramMode.value),
            filename: file.name,
            file_content: fileContent,
            disable_ramps: paramDisableRamps.checked,
            contact_voltage: parseFloat(paramContactVolt.value),
            contact_compliance_current_ua: parseFloat(paramContactComp.value),
            threshold_current_ua: parseFloat(paramThresholdCurr.value),
            volt_source: parseFloat(paramVoltSource.value),
            curr_comp: parseFloat(paramCurrComp.value),
            voltage_threshold_1: parseFloat(paramVoltageThr1.value),
            voltage_threshold_2: parseFloat(paramVoltageThr2.value),
            delta_z: parseFloat(paramDeltaZ.value),
            z_contact_step: parseFloat(paramZContactStep.value),
            speed: parseFloat(paramSpeed.value),
            z_feedback_speed: parseFloat(paramZFeedbackSpeed.value),
            liftoff_height: parseFloat(paramLiftoffHeight.value),
            max_safe_z_margin: parseFloat(paramMaxSafeZMargin.value),
            sample_interval_ms: parseFloat(paramSampleInterval.value)
        };
        
        executePatternRun(params);
    };
    reader.onerror = function() {
        logToConsole("[ERROR] Failed to read coordinate file.", true);
        btnStartPattern.disabled = false;
    };
    reader.readAsText(file);
});

async function executePatternRun(params) {
    logToConsole(`[SYSTEM] Executing pattern run: ${params.filename} (Mode ${params.mode})...`);
    // Reset chart data for a clean run
    chartData = [];
    patterningStartTime = Date.now();
    drawTelemetryChart();

    try {
        const res = await fetch(`${API_BASE}/api/run_pattern`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(params)
        });
        const data = await res.json();
        if (data.success) {
            isPatterning = true;
            logToConsole("[SYSTEM] Patterning run started successfully.");
            startFastTelemetryPoll();
        } else {
            logToConsole(`[ERROR] Pattern failed to start: ${data.error}`, true);
        }
    } catch (err) {
        logToConsole(`[ERROR] Run error: ${err.message}`, true);
    }
    btnStartPattern.disabled = false;
    updateUIState();
}

// STATUS MONITORING AND TELEMETRY
async function checkSystemStatus() {
    if (!isConnected) return;
    
    try {
        const res = await fetch(`${API_BASE}/api/status`);
        const status = await res.json();
        
        // Update positions
        updatePositions(status.position);
        
        // Check if patterning is active
        const patterningActive = status.patterning.active;
        if (patterningActive !== isPatterning) {
            isPatterning = patterningActive;
            updateUIState();
            if (isPatterning) {
                startFastTelemetryPoll();
            } else {
                stopFastTelemetryPoll();
            }
        }
        
        if (isPatterning) {
            updateTelemetry(status.patterning);
        }
    } catch (err) {
        // Silent fail on background checks
    }
}

function startFastTelemetryPoll() {
    if (statusInterval) clearInterval(statusInterval);
    
    // Poll quickly (every 300ms) during a run
    statusInterval = setInterval(async () => {
        try {
            const statusRes = await fetch(`${API_BASE}/api/status`);
            const status = await statusRes.json();
            updatePositions(status.position);
            updateTelemetry(status.patterning);
            
            // Check if completed
            if (!status.patterning.active) {
                isPatterning = false;
                logToConsole("[SYSTEM] Patterning run finished.");
                updateUIState();
                stopFastTelemetryPoll();
            }
        } catch (err) {}
    }, 300);
}

function stopFastTelemetryPoll() {
    if (statusInterval) {
        clearInterval(statusInterval);
        statusInterval = null;
    }
}

// HELPER UPDATES
function updatePositions(pos) {
    if (!pos) return;
    posXText.textContent = parseFloat(pos.x).toFixed(2);
    posYText.textContent = parseFloat(pos.y).toFixed(2);
    posZText.textContent = parseFloat(pos.z).toFixed(2);
}

function updateTelemetry(tel) {
    if (!tel) return;
    telVoltage.textContent = parseFloat(tel.smu_voltage).toFixed(4);
    telCurrent.textContent = (parseFloat(tel.smu_current) * 1e6).toFixed(4); // Display in µA
    
    // Update feedback direction badge
    const feedbackDir = tel.z_feedback_direction || "inactive";
    telFeedbackDir.className = `tel-value-badge ${feedbackDir}`;
    telFeedbackDir.textContent = feedbackDir.toUpperCase().replace("_", " ");
    
    // Update progress numbers and bar
    const done = tel.moves_done || 0;
    const total = tel.total_moves || 0;
    const left = tel.moves_left || 0;
    
    telMovesDone.textContent = done;
    telMovesTotal.textContent = total;
    telMovesLeft.textContent = left;
    
    if (total > 0) {
        const pct = (done / total) * 100;
        progressBar.style.width = `${pct}%`;
    } else {
        progressBar.style.width = "0%";
    }

    // Record real-time chart data if patterning is active
    if (tel.active) {
        if (!patterningStartTime) {
            patterningStartTime = Date.now();
        }
        const elapsed = (Date.now() - patterningStartTime) / 1000;
        const v = parseFloat(tel.smu_voltage);
        const i = parseFloat(tel.smu_current) * 1e6; // Convert to µA
        
        // Add point to chart if time advanced or first point
        if (chartData.length === 0 || elapsed - chartData[chartData.length - 1].time >= 0.05) {
            chartData.push({ time: elapsed, voltage: v, current: i });
            if (chartData.length > 300) chartData.shift();
            drawTelemetryChart();
        }
    } else {
        patterningStartTime = null;
    }
}

function updateUIState() {
    // Badges
    if (isPatterning) {
        systemStatusBadge.className = "status-badge patterning";
        systemStatusText.textContent = "Patterning Running";
    } else if (isConnected) {
        systemStatusBadge.className = "status-badge connected";
        systemStatusText.textContent = "Connected (Axes Idle)";
    } else {
        systemStatusBadge.className = "status-badge";
        systemStatusText.textContent = "Disconnected";
    }
    
    // Ports selector disabled state
    portXSelect.disabled = isConnected;
    portYSelect.disabled = isConnected;
    portZSelect.disabled = isConnected;
    
    // Connect button label
    btnConnect.textContent = isConnected ? "Disconnect Ports" : "Connect Ports";
    btnConnect.className = isConnected ? "btn btn-secondary" : "btn btn-primary";
    
    // Jog control status
    const jogButtons = document.querySelectorAll(".jog-btn");
    jogButtons.forEach(btn => {
        btn.disabled = !isConnected || isPatterning;
    });
    
    // Homing button status
    btnHome.disabled = !isConnected || isPatterning;
    
    // Run button status
    btnStartPattern.disabled = !isConnected;
    if (isPatterning) {
        btnStartPattern.textContent = "Abort Patterning Run";
        btnStartPattern.className = "btn btn-danger btn-action";
    } else {
        btnStartPattern.textContent = "Execute Pattern";
        btnStartPattern.className = "btn btn-action";
    }
}

// REAL-TIME TELEMETRY CHART DRAWING
function drawTelemetryChart() {
    if (!canvas || !ctx) return;
    
    const rect = canvas.getBoundingClientRect();
    if (canvas.width !== rect.width || canvas.height !== rect.height) {
        canvas.width = rect.width;
        canvas.height = rect.height;
    }
    
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    
    if (chartData.length === 0) {
        ctx.fillStyle = "rgba(255, 255, 255, 0.2)";
        ctx.font = "12px sans-serif";
        ctx.textAlign = "center";
        ctx.fillText("Real-time telemetry chart will plot here during patterning...", w / 2, h / 2);
        return;
    }
    
    const padLeft = 45;
    const padRight = 45;
    const padTop = 15;
    const padBottom = 20;
    
    const plotW = w - padLeft - padRight;
    const plotH = h - padTop - padBottom;
    
    // Find min/max values
    let minTime = 0;
    let maxTime = Math.max(10, ...chartData.map(d => d.time));
    
    let minVolt = Math.min(...chartData.map(d => d.voltage));
    let maxVolt = Math.max(...chartData.map(d => d.voltage));
    if (maxVolt - minVolt < 1.0) {
        const avg = (maxVolt + minVolt) / 2;
        minVolt = avg - 0.5;
        maxVolt = avg + 0.5;
    }
    
    let minCurr = Math.min(...chartData.map(d => d.current));
    let maxCurr = Math.max(...chartData.map(d => d.current));
    if (maxCurr - minCurr < 0.1) {
        const avg = (maxCurr + minCurr) / 2;
        minCurr = avg - 0.05;
        maxCurr = avg + 0.05;
    }
    
    // Draw background grid lines
    ctx.strokeStyle = "rgba(255, 255, 255, 0.05)";
    ctx.lineWidth = 1;
    const gridCols = 6;
    for (let c = 0; c <= gridCols; c++) {
        const gx = padLeft + (c / gridCols) * plotW;
        ctx.beginPath();
        ctx.moveTo(gx, padTop);
        ctx.lineTo(gx, padTop + plotH);
        ctx.stroke();
    }
    const gridRows = 4;
    for (let r = 0; r <= gridRows; r++) {
        const gy = padTop + (r / gridRows) * plotH;
        ctx.beginPath();
        ctx.moveTo(padLeft, gy);
        ctx.lineTo(padLeft + plotW, gy);
        ctx.stroke();
    }
    
    // Draw Y axis labels (Voltage - Left side in Cyan)
    ctx.fillStyle = "#00f2fe";
    ctx.font = "9px monospace";
    ctx.textAlign = "right";
    ctx.fillText(maxVolt.toFixed(2) + "V", padLeft - 6, padTop + 4);
    ctx.fillText(((maxVolt + minVolt) / 2).toFixed(2) + "V", padLeft - 6, padTop + plotH/2 + 3);
    ctx.fillText(minVolt.toFixed(2) + "V", padLeft - 6, padTop + plotH + 2);
    
    // Draw Y axis labels (Current - Right side in Pink)
    ctx.fillStyle = "#ff007f";
    ctx.textAlign = "left";
    ctx.fillText(maxCurr.toFixed(3) + "µA", padLeft + plotW + 6, padTop + 4);
    ctx.fillText(((maxCurr + minCurr) / 2).toFixed(3) + "µA", padLeft + plotW + 6, padTop + plotH/2 + 3);
    ctx.fillText(minCurr.toFixed(3) + "µA", padLeft + plotW + 6, padTop + plotH + 2);
    
    // Draw X axis label (Time - bottom center)
    ctx.fillStyle = "rgba(255, 255, 255, 0.4)";
    ctx.textAlign = "center";
    ctx.fillText(maxTime.toFixed(1) + "s", padLeft + plotW, padTop + plotH + 12);
    ctx.fillText("Time (s)", padLeft + plotW / 2, padTop + plotH + 12);
    
    // Helper to map coordinates
    function getX(t) {
        return padLeft + ((t - minTime) / (maxTime - minTime)) * plotW;
    }
    function getYVolt(v) {
        return padTop + plotH - ((v - minVolt) / (maxVolt - minVolt)) * plotH;
    }
    function getYCurr(c) {
        return padTop + plotH - ((c - minCurr) / (maxCurr - minCurr)) * plotH;
    }
    
    // Draw Voltage line
    ctx.beginPath();
    ctx.strokeStyle = "#00f2fe";
    ctx.lineWidth = 1.8;
    ctx.shadowColor = "rgba(0, 242, 254, 0.4)";
    ctx.shadowBlur = 4;
    chartData.forEach((pt, idx) => {
        const x = getX(pt.time);
        const y = getYVolt(pt.voltage);
        if (idx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    });
    ctx.stroke();
    
    // Draw Current line
    ctx.beginPath();
    ctx.strokeStyle = "#ff007f";
    ctx.lineWidth = 1.8;
    ctx.shadowColor = "rgba(255, 0, 127, 0.4)";
    ctx.shadowBlur = 4;
    chartData.forEach((pt, idx) => {
        const x = getX(pt.time);
        const y = getYCurr(pt.current);
        if (idx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    });
    ctx.stroke();
    
    // Reset shadow
    ctx.shadowBlur = 0;
}

window.addEventListener("resize", drawTelemetryChart);

// CONSOLE UTILITIES
function logToConsole(message, isError = false) {
    if (isError) {
        console.error(message);
    } else {
        console.log(message);
    }
}
