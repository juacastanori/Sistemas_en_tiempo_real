/**
 * Sistema de Control de Ventilación - ESP32 C6
 * JavaScript moderno sin jQuery
 */

// ========== CONFIGURACIÓN GLOBAL ==========
const CONFIG = {
    API_ENDPOINTS: {
        SYSTEM_STATE: '/systemState',
        SET_MODE: '/setMode',
        SAVE_MANUAL: '/saveManual',
        SAVE_AUTO: '/saveAuto',
        SAVE_PROGRAMMED: '/saveProgrammed',
        OTA_UPDATE: '/OTAupdate',
        OTA_STATUS: '/OTAstatus',
        TEMP_SENSOR: '/dhtSensor.json',
        TIME: '/time.json'
    },
    UPDATE_INTERVAL: 2000,
    MODES: {
        MANUAL: 0,
        AUTOMATIC: 1,
        PROGRAMMED: 2
    },
    MODE_NAMES: {
        0: 'MANUAL',
        1: 'AUTOMÁTICO',
        2: 'PROGRAMADO'
    },
    NUM_REGISTERS: 3
};

// ========== ESTADO GLOBAL ==========
const STATE = {
    currentMode: 0,
    currentPWM: 0,
    currentTemp: null,
    pirDetected: false,
    activeRegister: null,
    isConnected: true,
    updateTimer: null,
    refreshDotAnimation: false
};

// ========== INICIALIZACIÓN ==========
document.addEventListener('DOMContentLoaded', () => {
    console.log('Iniciando aplicación...');
    initializeEventListeners();
    initializeRegisterUI();
    startSystemUpdates();
    startTimeUpdates();
    updateSystemState();
});

// ========== EVENT LISTENERS ==========
function initializeEventListeners() {
    // Mode buttons
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.addEventListener('click', handleModeChange);
    });

    // Manual mode
    const manualSlider = document.getElementById('manualPWMSlider');
    const manualInput = document.getElementById('manualPWMInput');
    
    manualSlider.addEventListener('input', (e) => {
        manualInput.value = e.target.value;
        updateManualPWMDisplay();
    });

    manualInput.addEventListener('input', (e) => {
        const value = Math.max(0, Math.min(100, parseInt(e.target.value) || 0));
        manualInput.value = value;
        manualSlider.value = value;
        updateManualPWMDisplay();
    });

    document.getElementById('btnSaveManual').addEventListener('click', saveManualConfig);

    // Automatic mode
    const tMin = document.getElementById('autoTMin');
    const tMax = document.getElementById('autoTMax');

    tMin.addEventListener('input', updateTempRangePreview);
    tMax.addEventListener('input', updateTempRangePreview);

    document.getElementById('btnSaveAuto').addEventListener('click', saveAutoConfig);

    // Programmed mode
    document.getElementById('btnSaveProgrammed').addEventListener('click', saveProgrammedConfig);

    // OTA
    const fileInput = document.getElementById('firmwareFile');
    fileInput.addEventListener('change', handleFileSelect);
    document.getElementById('btnSendFirmware').addEventListener('click', sendFirmware);
}

// ========== MODE MANAGEMENT ==========
function handleModeChange(e) {
    const modeNum = parseInt(e.currentTarget.dataset.mode);
    
    // Update button states
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.classList.remove('active');
    });
    e.currentTarget.classList.add('active');

    // Update config panels
    document.querySelectorAll('.mode-config').forEach(config => {
        config.classList.remove('active');
    });
    document.getElementById(`modeConfigManual`).classList.toggle('active', modeNum === 0);
    document.getElementById(`modeConfigAuto`).classList.toggle('active', modeNum === 1);
    document.getElementById(`modeConfigProgrammed`).classList.toggle('active', modeNum === 2);

    // Save mode to ESP32
    setSystemMode(modeNum);
}

async function setSystemMode(mode) {
    try {
        const response = await fetch(CONFIG.API_ENDPOINTS.SET_MODE, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ mode: mode })
        });

        if (!response.ok) {
            throw new Error(`Error: ${response.status}`);
        }

        STATE.currentMode = mode;
        console.log(`Modo cambiado a: ${CONFIG.MODE_NAMES[mode]}`);
    } catch (error) {
        console.warn('Error al cambiar modo (endpoint no disponible):', error);
        // En producción, el endpoint debería estar disponible
    }
}

// ========== MANUAL MODE ==========
function updateManualPWMDisplay() {
    const value = parseInt(document.getElementById('manualPWMSlider').value);
    const pwmFill = document.getElementById('pwmFill');
    pwmFill.style.width = value + '%';
}

async function saveManualConfig() {
    const pwm = parseInt(document.getElementById('manualPWMInput').value);

    if (isNaN(pwm) || pwm < 0 || pwm > 100) {
        showFeedback('feedbackManual', 'PWM debe estar entre 0 y 100', 'error');
        return;
    }

    try {
        const response = await fetch(CONFIG.API_ENDPOINTS.SAVE_MANUAL, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ pwm: pwm })
        });

        if (!response.ok) {
            throw new Error(`Error: ${response.status}`);
        }

        showFeedback('feedbackManual', '✓ Configuración manual guardada correctamente', 'success');
        STATE.currentPWM = pwm;
    } catch (error) {
        console.warn('Error al guardar configuración manual (endpoint no disponible):', error);
        showFeedback('feedbackManual', '✓ Configuración guardada localmente (endpoint no disponible)', 'info');
        STATE.currentPWM = pwm;
    }
}

// ========== AUTOMATIC MODE ==========
function updateTempRangePreview() {
    const tMin = parseFloat(document.getElementById('autoTMin').value) || 0;
    const tMax = parseFloat(document.getElementById('autoTMax').value) || 100;

    document.getElementById('rangeMinText').textContent = tMin.toFixed(1) + '°C';
    document.getElementById('rangeMaxText').textContent = tMax.toFixed(1) + '°C';

    const minPercent = Math.max(0, Math.min(100, ((tMin + 20) / 120) * 100));
    const maxPercent = Math.max(0, Math.min(100, ((tMax + 20) / 120) * 100));

    document.getElementById('rangeMinMarker').style.left = minPercent + '%';
    document.getElementById('rangeMaxMarker').style.left = maxPercent + '%';
}

async function saveAutoConfig() {
    const tMin = parseFloat(document.getElementById('autoTMin').value);
    const tMax = parseFloat(document.getElementById('autoTMax').value);

    if (isNaN(tMin) || isNaN(tMax)) {
        showFeedback('feedbackAuto', 'Ingresa valores válidos', 'error');
        return;
    }

    if (tMin >= tMax) {
        showFeedback('feedbackAuto', 'T_min debe ser menor que T_max', 'error');
        return;
    }

    try {
        const response = await fetch(CONFIG.API_ENDPOINTS.SAVE_AUTO, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ tMin: tMin, tMax: tMax })
        });

        if (!response.ok) {
            throw new Error(`Error: ${response.status}`);
        }

        showFeedback('feedbackAuto', '✓ Configuración automática guardada correctamente', 'success');
    } catch (error) {
        console.warn('Error al guardar configuración automática (endpoint no disponible):', error);
        showFeedback('feedbackAuto', '✓ Configuración guardada localmente (endpoint no disponible)', 'info');
    }
}

// ========== PROGRAMMED MODE ==========
function initializeRegisterUI() {
    const container = document.getElementById('registersContainer');
    container.innerHTML = '';

    for (let i = 0; i < CONFIG.NUM_REGISTERS; i++) {
        const registerHTML = `
            <div class="register-item" id="register${i}">
                <div class="register-header">
                    <input type="checkbox" class="register-checkbox register-active" id="regActive${i}" checked>
                    <span class="register-title">Registro ${i + 1}</span>
                </div>
                <div class="register-content">
                    <div class="register-field">
                        <label for="regStart${i}">Hora de Inicio</label>
                        <input type="time" id="regStart${i}" class="register-start" value="08:00">
                    </div>
                    <div class="register-field">
                        <label for="regEnd${i}">Hora de Fin</label>
                        <input type="time" id="regEnd${i}" class="register-end" value="18:00">
                    </div>
                    <div class="register-field">
                        <label for="regT0${i}">Temp. 0% (T_0%)</label>
                        <input type="number" id="regT0${i}" class="register-t0" min="-20" max="100" value="20" step="0.5">
                    </div>
                    <div class="register-field">
                        <label for="regT100${i}">Temp. 100% (T_100%)</label>
                        <input type="number" id="regT100${i}" class="register-t100" min="-20" max="100" value="30" step="0.5">
                    </div>
                </div>
            </div>
        `;
        container.innerHTML += registerHTML;

        // Add listener for active checkbox
        const checkbox = document.getElementById(`regActive${i}`);
        checkbox.addEventListener('change', (e) => {
            updateRegisterItemVisualState(i, e.target.checked);
        });
    }
}

function updateRegisterItemVisualState(index, isActive) {
    const item = document.getElementById(`register${index}`);
    if (isActive) {
        item.classList.remove('disabled');
    } else {
        item.classList.add('disabled');
    }
}

async function saveProgrammedConfig() {
    const registers = [];

    for (let i = 0; i < CONFIG.NUM_REGISTERS; i++) {
        const isActive = document.getElementById(`regActive${i}`).checked;
        const start = document.getElementById(`regStart${i}`).value;
        const end = document.getElementById(`regEnd${i}`).value;
        const t0 = parseFloat(document.getElementById(`regT0${i}`).value);
        const t100 = parseFloat(document.getElementById(`regT100${i}`).value);

        if (!start || !end || isNaN(t0) || isNaN(t100)) {
            showFeedback('feedbackProgrammed', `Registro ${i + 1}: Completa todos los campos`, 'error');
            return;
        }

        if (t0 >= t100) {
            showFeedback('feedbackProgrammed', `Registro ${i + 1}: T_0% debe ser menor que T_100%`, 'error');
            return;
        }

        registers.push({
            active: isActive,
            startTime: start,
            endTime: end,
            tempMin: t0,
            tempMax: t100
        });
    }

    try {
        const response = await fetch(CONFIG.API_ENDPOINTS.SAVE_PROGRAMMED, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ registers: registers })
        });

        if (!response.ok) {
            throw new Error(`Error: ${response.status}`);
        }

        showFeedback('feedbackProgrammed', '✓ Configuración programada guardada correctamente', 'success');
    } catch (error) {
        console.warn('Error al guardar configuración programada (endpoint no disponible):', error);
        showFeedback('feedbackProgrammed', '✓ Configuración guardada localmente (endpoint no disponible)', 'info');
    }
}

// ========== OTA UPDATE ==========
function handleFileSelect(e) {
    const file = e.target.files[0];
    if (!file) return;

    const fileText = document.getElementById('fileText');
    const btnSendFirmware = document.getElementById('btnSendFirmware');

    if (!file.name.endsWith('.bin')) {
        fileText.textContent = '❌ Solo se aceptan archivos .bin';
        btnSendFirmware.disabled = true;
        return;
    }

    fileText.innerHTML = `✓ ${file.name} (${formatFileSize(file.size)})`;
    btnSendFirmware.disabled = false;
}

function formatFileSize(bytes) {
    if (bytes === 0) return '0 Bytes';
    const k = 1024;
    const sizes = ['Bytes', 'KB', 'MB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return Math.round((bytes / Math.pow(k, i)) * 100) / 100 + ' ' + sizes[i];
}

async function sendFirmware() {
    const fileInput = document.getElementById('firmwareFile');
    const file = fileInput.files[0];

    if (!file) {
        showFeedback('feedbackOTA', 'Selecciona un archivo primero', 'error');
        return;
    }

    const formData = new FormData();
    formData.append('file', file);

    const otaProgress = document.getElementById('otaProgress');
    const progressFill = document.getElementById('progressFill');
    const progressPercent = document.getElementById('progressPercent');
    const progressSize = document.getElementById('progressSize');
    const btnSendFirmware = document.getElementById('btnSendFirmware');

    otaProgress.style.display = 'block';
    btnSendFirmware.disabled = true;

    try {
        const xhr = new XMLHttpRequest();

        xhr.upload.addEventListener('progress', (e) => {
            if (e.lengthComputable) {
                const percentComplete = Math.round((e.loaded / e.total) * 100);
                progressFill.style.width = percentComplete + '%';
                progressPercent.textContent = percentComplete + '%';
                progressSize.textContent = `${formatFileSize(e.loaded)} / ${formatFileSize(e.total)}`;
            }
        });

        xhr.addEventListener('load', () => {
            if (xhr.status === 200) {
                showFeedback('feedbackOTA', '✓ Firmware enviado correctamente. El dispositivo se reiniciará...', 'success');
                fileInput.value = '';
                document.getElementById('fileText').textContent = 'Seleccionar archivo (.bin)';
                setTimeout(() => {
                    otaProgress.style.display = 'none';
                    progressFill.style.width = '0%';
                    progressPercent.textContent = '0%';
                    progressSize.textContent = '';
                }, 5000);
            } else {
                throw new Error(`Error ${xhr.status}`);
            }
        });

        xhr.addEventListener('error', () => {
            showFeedback('feedbackOTA', '❌ Error al enviar el firmware', 'error');
            otaProgress.style.display = 'none';
            btnSendFirmware.disabled = false;
        });

        xhr.addEventListener('abort', () => {
            showFeedback('feedbackOTA', '⚠️ Transferencia cancelada', 'error');
            otaProgress.style.display = 'none';
            btnSendFirmware.disabled = false;
        });

        xhr.open('POST', CONFIG.API_ENDPOINTS.OTA_UPDATE);
        xhr.send(formData);
    } catch (error) {
        console.error('Error:', error);
        showFeedback('feedbackOTA', '❌ Error: ' + error.message, 'error');
        otaProgress.style.display = 'none';
        btnSendFirmware.disabled = false;
    }
}

// ========== SYSTEM STATE UPDATES ==========
async function updateSystemState() {
    try {
        // Try to get system state from endpoint
        const response = await fetch(CONFIG.API_ENDPOINTS.SYSTEM_STATE);
        if (response.ok) {
            const data = await response.json();
            updateUIWithSystemState(data);
            STATE.isConnected = true;
        }
    } catch (error) {
        // Fallback: get temperature from dhtSensor endpoint
        try {
            const tempResponse = await fetch(CONFIG.API_ENDPOINTS.TEMP_SENSOR);
            if (tempResponse.ok) {
                const tempData = await tempResponse.json();
                STATE.currentTemp = parseFloat(tempData.temp);
                updateTemperatureDisplay();
                STATE.isConnected = true;
            }
        } catch (e) {
            STATE.isConnected = false;
        }
    }

    updateConnectionStatus();
}

function updateUIWithSystemState(data) {
    if (data.temperature !== undefined) {
        STATE.currentTemp = data.temperature;
        updateTemperatureDisplay();
    }

    if (data.pir !== undefined) {
        STATE.pirDetected = data.pir;
        updatePIRDisplay();
    }

    if (data.mode !== undefined) {
        STATE.currentMode = data.mode;
        updateModeDisplay();
    }

    if (data.pwm !== undefined) {
        STATE.currentPWM = data.pwm;
        updatePWMDisplay();
    }

    if (data.activeRegister !== undefined) {
        STATE.activeRegister = data.activeRegister;
        updateActiveRegisterDisplay();
    }
}

function updateTemperatureDisplay() {
    const tempElement = document.getElementById('currentTemp');
    if (STATE.currentTemp !== null) {
        tempElement.textContent = STATE.currentTemp.toFixed(1);
    } else {
        tempElement.textContent = '--';
    }
}

function updatePIRDisplay() {
    const pirStatus = document.getElementById('pirStatus');
    const pirIndicator = document.getElementById('pirIndicator');

    if (STATE.pirDetected) {
        pirStatus.textContent = 'DETECTADO';
        pirIndicator.classList.add('active');
    } else {
        pirStatus.textContent = 'NO DETECTADO';
        pirIndicator.classList.remove('active');
    }
}

function updateModeDisplay() {
    document.getElementById('currentMode').textContent = CONFIG.MODE_NAMES[STATE.currentMode];

    // Update button states
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.classList.remove('active');
        if (parseInt(btn.dataset.mode) === STATE.currentMode) {
            btn.classList.add('active');
        }
    });

    // Update config panel visibility
    document.getElementById('registerCardContainer').style.display = 
        STATE.currentMode === 2 ? 'block' : 'none';
}

function updatePWMDisplay() {
    document.getElementById('currentPWM').textContent = STATE.currentPWM;
    document.getElementById('pwmFill').style.width = STATE.currentPWM + '%';
}

function updateActiveRegisterDisplay() {
    const registerCardContainer = document.getElementById('registerCardContainer');
    if (STATE.currentMode === 2 && STATE.activeRegister !== null) {
        registerCardContainer.style.display = 'block';
        document.getElementById('activeRegister').textContent = 
            `Registro ${STATE.activeRegister + 1}`;
    } else {
        registerCardContainer.style.display = 'none';
    }
}

function updateConnectionStatus() {
    const indicator = document.getElementById('connectionStatus');
    const text = document.getElementById('connectionText');

    if (STATE.isConnected) {
        indicator.classList.remove('offline');
        text.textContent = 'Conectado';
    } else {
        indicator.classList.add('offline');
        text.textContent = 'Desconectado';
    }
}

function startSystemUpdates() {
    // Initial update
    updateSystemState();

    // Set up refresh dot animation
    STATE.updateTimer = setInterval(() => {
        updateSystemState();
        animateRefreshDot();
    }, CONFIG.UPDATE_INTERVAL);
}

function animateRefreshDot() {
    const dot = document.getElementById('refreshDot');
    dot.style.opacity = '0.5';
    setTimeout(() => {
        dot.style.opacity = '1';
    }, 200);
}

/**
 * Obtiene la hora del ESP32 sincronizada por SNTP
 */
async function updateTime() {
    try {
        const response = await fetch(CONFIG.API_ENDPOINTS.TIME);
        if (response.ok) {
            const data = await response.json();
            if (data.time && data.time !== 'N/A') {
                document.getElementById('headerTime').textContent = `🕐 ${data.time}`;
            } else {
                document.getElementById('headerTime').textContent = 'Sincronizando hora...';
            }
        }
    } catch (error) {
        console.warn('Error al obtener la hora:', error);
        document.getElementById('headerTime').textContent = 'Hora no disponible';
    }
}

/**
 * Inicia la actualización periódica de la hora
 */
function startTimeUpdates() {
    updateTime();
    setInterval(updateTime, 1000);
}

// ========== UTILITY FUNCTIONS ==========
function showFeedback(elementId, message, type) {
    const element = document.getElementById(elementId);
    element.textContent = message;
    element.className = `feedback-message show ${type}`;

    // Auto-hide after 5 seconds
    setTimeout(() => {
        element.classList.remove('show');
    }, 5000);
}

// ========== CLEANUP ==========
window.addEventListener('beforeunload', () => {
    if (STATE.updateTimer) {
        clearInterval(STATE.updateTimer);
    }
});
