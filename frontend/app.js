/* frontend/app.js */

const API_BASE = ""; // Relative path to local server

// State tracking
let isConnected = false;
let isPatterning = false;
let selectedStep = 0.2;
let statusInterval = null;
let logOffset = 0;
let lastLogsLength = 0;

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
const paramFilename = document.getElementById("param-filename");
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

const consoleOutput = document.getElementById("console-output");
const btnClearConsole = document.getElementById("btn-clear-console");
const chkAutoscroll = document.getElementById("chk-autoscroll");

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
    scanPorts();
    scanFiles();
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
        
        // Auto-select defaults if there are at least 3 ports
        if (ports.length >= 1) portXSelect.value = ports[0].device;
        if (ports.length >= 2) portYSelect.value = ports[1].device;
        if (ports.length >= 3) portZSelect.value = ports[2].device;
        
    } catch (err) {
        logToConsole(`[ERROR] Failed to scan COM ports: ${err.message}`, true);
    }
}

// SCAN COORD FILES
async function scanFiles() {
    try {
        const res = await fetch(`${API_BASE}/api/files`);
        const files = await res.json();
        
        paramFilename.innerHTML = "";
        if (files.length === 0) {
            paramFilename.innerHTML = '<option value="circles">None found (fallback to circles)</option>';
            return;
        }
        
        files.forEach(file => {
            const opt = document.createElement("option");
            opt.value = file;
            opt.textContent = `${file}.txt`;
            paramFilename.appendChild(opt);
        });
    } catch (err) {
        logToConsole(`[ERROR] Failed to load pattern files: ${err.message}`, true);
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
    
    // Gather and validate parameters
    const params = {
        mode: parseInt(paramMode.value),
        filename: paramFilename.value,
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
    
    logToConsole(`[SYSTEM] Starting patterning: ${params.filename}.txt (Mode ${params.mode})...`);
    btnStartPattern.disabled = true;
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
});

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
        
        pollLogs();
    }, 300);
}

function stopFastTelemetryPoll() {
    if (statusInterval) {
        clearInterval(statusInterval);
        statusInterval = null;
    }
}

// POLL CONSOLE OUTPUTS
async function pollLogs() {
    try {
        const res = await fetch(`${API_BASE}/api/log`);
        const data = await res.json();
        const logs = data.logs;
        
        if (logs.length > lastLogsLength) {
            const newLines = logs.slice(lastLogsLength);
            newLines.forEach(line => appendConsoleLine(line));
            lastLogsLength = logs.length;
        } else if (logs.length < lastLogsLength) {
            // Buffer was reset
            consoleOutput.innerHTML = "";
            logs.forEach(line => appendConsoleLine(line));
            lastLogsLength = logs.length;
        }
    } catch (err) {}
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

// CONSOLE UTILITIES
btnClearConsole.addEventListener("click", () => {
    consoleOutput.innerHTML = "";
    lastLogsLength = 0;
    logToConsole("[SYSTEM] Console cleared.");
});

function logToConsole(message, isError = false) {
    appendConsoleLine(message, isError);
}

function appendConsoleLine(line, isError = false) {
    const div = document.createElement("div");
    div.className = "console-line";
    
    // Color coding matching pattern tags
    if (isError || line.includes("[ERROR]") || line.includes("[EXCEPTION]") || line.includes("TimeoutError")) {
        div.classList.add("error-line");
    } else if (line.includes("[SYSTEM]") || line.includes("[INFO]") || line.includes("[SAFETY]")) {
        div.classList.add("system-line");
    } else if (line.includes("[FEEDBACK]")) {
        div.classList.add("feedback-line");
    } else if (line.includes("[VOLTAGE]") || line.includes("[CURRENT]") || line.includes("[SMU]")) {
        div.classList.add("smu-line");
    } else if (line.includes("[MOVE]")) {
        div.classList.add("move-line");
    }
    
    div.textContent = line;
    consoleOutput.appendChild(div);
    
    // Autoscroll if selected
    if (chkAutoscroll.checked) {
        consoleOutput.scrollTop = consoleOutput.scrollHeight;
    }
}
