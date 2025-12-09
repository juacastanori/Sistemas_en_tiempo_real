/**
 * Sistema de Control de Ventilación - ESP32
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
        GET_PROGRAMMED: '/getProgrammed',
        OTA_UPDATE: '/OTAupdate',
        OTA_STATUS: '/OTAstatus',
        TIME: '/time.json',
        SAVE_WIFI: '/saveWiFi'
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
    isConnected: false,
    updateTimer: null,
    refreshDotAnimation: false
};

// ========== INICIALIZACIÓN ==========
document.addEventListener('DOMContentLoaded', () => {
    console.log('Iniciando aplicación...');
    initializeEventListeners();
    initializeRegisterUI();
    loadSavedRegisters();  // Cargar registros guardados
    startSystemUpdates();
    startTimeUpdates();
    // Primera actualización inmediata
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
    
    if(manualSlider && manualInput) {
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
    }

    const btnSaveManual = document.getElementById('btnSaveManual');
    if(btnSaveManual) btnSaveManual.addEventListener('click', saveManualConfig);

    // Automatic mode
    const tMin = document.getElementById('autoTMin');
    const tMax = document.getElementById('autoTMax');

    if(tMin) tMin.addEventListener('input', updateTempRangePreview);
    if(tMax) tMax.addEventListener('input', updateTempRangePreview);

    const btnSaveAuto = document.getElementById('btnSaveAuto');
    if(btnSaveAuto) btnSaveAuto.addEventListener('click', saveAutoConfig);

    // Programmed mode
    const btnSaveProgrammed = document.getElementById('btnSaveProgrammed');
    if(btnSaveProgrammed) btnSaveProgrammed.addEventListener('click', saveProgrammedConfig);

    // OTA
    const fileInput = document.getElementById('firmwareFile');
    if(fileInput) fileInput.addEventListener('change', handleFileSelect);
    
    const btnSendFirmware = document.getElementById('btnSendFirmware');
    if(btnSendFirmware) btnSendFirmware.addEventListener('click', sendFirmware);

    // WiFi Configuration
    const btnSaveWiFi = document.getElementById('btnSaveWiFi');
    if(btnSaveWiFi) btnSaveWiFi.addEventListener('click', saveWiFiConfig);
}

// ========== MODE MANAGEMENT ==========
function handleModeChange(e) {
    const modeNum = parseInt(e.currentTarget.dataset.mode);
    
    // Update button states visualmente
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.classList.remove('active');
    });
    e.currentTarget.classList.add('active');

    // Update config panels
    document.querySelectorAll('.mode-config').forEach(config => {
        config.classList.remove('active');
    });
    
    const manualConfig = document.getElementById(`modeConfigManual`);
    const autoConfig = document.getElementById(`modeConfigAuto`);
    const progConfig = document.getElementById(`modeConfigProgrammed`);

    if(manualConfig) manualConfig.classList.toggle('active', modeNum === 0);
    if(autoConfig) autoConfig.classList.toggle('active', modeNum === 1);
    if(progConfig) progConfig.classList.toggle('active', modeNum === 2);

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
        // Actualizar UI para reflejar el cambio confirmado
        updateModeDisplay();
    } catch (error) {
        console.warn('Error al cambiar modo:', error);
    }
}

// ========== MANUAL MODE ==========
function updateManualPWMDisplay() {
    const slider = document.getElementById('manualPWMSlider');
    const pwmFill = document.getElementById('pwmFill');
    if(slider && pwmFill) {
        const value = parseInt(slider.value);
        pwmFill.style.width = value + '%';
    }
}

async function saveManualConfig() {
    const input = document.getElementById('manualPWMInput');
    if(!input) return;
    
    const pwm = parseInt(input.value);

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
        console.warn('Error al guardar configuración manual:', error);
        showFeedback('feedbackManual', '⚠ Error de conexión', 'error');
    }
}

// ========== AUTOMATIC MODE ==========
function updateTempRangePreview() {
    const tMinEl = document.getElementById('autoTMin');
    const tMaxEl = document.getElementById('autoTMax');
    
    if(!tMinEl || !tMaxEl) return;

    const tMin = parseFloat(tMinEl.value) || 0;
    const tMax = parseFloat(tMaxEl.value) || 100;

    const rangeMinText = document.getElementById('rangeMinText');
    const rangeMaxText = document.getElementById('rangeMaxText');
    
    if(rangeMinText) rangeMinText.textContent = tMin.toFixed(1) + '°C';
    if(rangeMaxText) rangeMaxText.textContent = tMax.toFixed(1) + '°C';

    // Asumiendo un rango visual de -20 a 100 grados para la barra
    const minPercent = Math.max(0, Math.min(100, ((tMin + 20) / 120) * 100));
    const maxPercent = Math.max(0, Math.min(100, ((tMax + 20) / 120) * 100));

    const markerMin = document.getElementById('rangeMinMarker');
    const markerMax = document.getElementById('rangeMaxMarker');

    if(markerMin) markerMin.style.left = minPercent + '%';
    if(markerMax) markerMax.style.left = maxPercent + '%';
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
        console.warn('Error al guardar configuración automática:', error);
        showFeedback('feedbackAuto', '⚠ Error de conexión', 'error');
    }
}

// ========== PROGRAMMED MODE ==========
function initializeRegisterUI() {
    const container = document.getElementById('registersContainer');
    if(!container) return;
    
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
    }

    // Add listeners after creation
    for (let i = 0; i < CONFIG.NUM_REGISTERS; i++) {
        const checkbox = document.getElementById(`regActive${i}`);
        if(checkbox) {
            checkbox.addEventListener('change', (e) => {
                updateRegisterItemVisualState(i, e.target.checked);
            });
            // Init state
            updateRegisterItemVisualState(i, checkbox.checked);
        }
    }
}

function updateRegisterItemVisualState(index, isActive) {
    const item = document.getElementById(`register${index}`);
    if(item) {
        if (isActive) {
            item.classList.remove('disabled');
        } else {
            item.classList.add('disabled');
        }
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
            active: isActive ? 1 : 0, // Convert boolean to int for C code
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
        console.warn('Error al guardar configuración programada:', error);
        showFeedback('feedbackProgrammed', '⚠ Error de conexión', 'error');
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
                showFeedback('feedbackOTA', `❌ Error ${xhr.status}`, 'error');
                btnSendFirmware.disabled = false;
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

// ========== WiFi CONFIGURATION ==========
/**
 * Guarda las credenciales WiFi STA
 */
async function saveWiFiConfig() {
    const ssidInput = document.getElementById('wifiSSID');
    const passwordInput = document.getElementById('wifiPassword');
    const feedbackEl = document.getElementById('feedbackWiFi');
    const btnSaveWiFi = document.getElementById('btnSaveWiFi');

    const ssid = ssidInput.value.trim();
    const password = passwordInput.value.trim();

    // Validación básica
    if (!ssid) {
        showFeedback('feedbackWiFi', '❌ Ingresa el nombre de la red WiFi (SSID)', 'error');
        return;
    }

    if (!password) {
        showFeedback('feedbackWiFi', '❌ Ingresa la contraseña WiFi', 'error');
        return;
    }

    if (ssid.length > 32) {
        showFeedback('feedbackWiFi', '❌ El SSID no puede exceder 32 caracteres', 'error');
        return;
    }

    if (password.length > 64) {
        showFeedback('feedbackWiFi', '❌ La contraseña no puede exceder 64 caracteres', 'error');
        return;
    }

    btnSaveWiFi.disabled = true;
    showFeedback('feedbackWiFi', '⏳ Guardando credenciales WiFi...', 'info');

    try {
        const response = await fetch('/saveWiFi', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ 
                ssid: ssid,
                password: password
            })
        });

        const data = await response.json();

        if (response.ok && data.status === 'ok') {
            showFeedback('feedbackWiFi', '✓ Credenciales guardadas correctamente. El dispositivo intentará conectarse a la nueva red...', 'success');
            // Limpiar campos
            ssidInput.value = '';
            passwordInput.value = '';
            // Auto-reset feedback después de 5 segundos
            setTimeout(() => {
                feedbackEl.textContent = '';
                feedbackEl.className = '';
            }, 5000);
        } else {
            showFeedback('feedbackWiFi', `❌ Error: ${data.message || 'No se pudieron guardar las credenciales'}`, 'error');
        }
    } catch (error) {
        console.error('Error:', error);
        showFeedback('feedbackWiFi', '❌ Error de conexión: ' + error.message, 'error');
    } finally {
        btnSaveWiFi.disabled = false;
    }
}

// ========== SYSTEM STATE UPDATES ========== 
async function updateSystemState() {
    try {
        const response = await fetch(CONFIG.API_ENDPOINTS.SYSTEM_STATE);
        if (response.ok) {
            const data = await response.json();
            updateUIWithSystemState(data);
            STATE.isConnected = true;
        } else {
            STATE.isConnected = false;
        }
    } catch (error) {
        // Error de conexión
        STATE.isConnected = false;
    }

    updateConnectionStatus();
}

function updateUIWithSystemState(data) {
    if (data.temperature !== undefined) {
        STATE.currentTemp = data.temperature;
        updateTemperatureDisplay();
    }

    if (data.pir !== undefined) {
        STATE.pirDetected = (data.pir === 1); // Ensure boolean
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

    // Nota: systemState en C no estaba enviando activeRegister en tu código anterior
    // Si decidimos agregarlo al JSON de C, esto funcionará.
    if (data.activeRegister !== undefined) {
        STATE.activeRegister = data.activeRegister;
        updateActiveRegisterDisplay();
    }
}

function updateTemperatureDisplay() {
    const tempElement = document.getElementById('currentTemp');
    if(tempElement) {
        if (STATE.currentTemp !== null && STATE.currentTemp > -99) {
            tempElement.textContent = STATE.currentTemp.toFixed(1);
        } else {
            tempElement.textContent = '--';
        }
    }
}

function updatePIRDisplay() {
    const pirStatus = document.getElementById('pirStatus');
    const pirIndicator = document.getElementById('pirIndicator');

    if(pirStatus && pirIndicator) {
        if (STATE.pirDetected) {
            pirStatus.textContent = 'DETECTADO';
            pirIndicator.classList.add('active');
        } else {
            pirStatus.textContent = 'NO DETECTADO';
            pirIndicator.classList.remove('active');
        }
    }
}

function updateModeDisplay() {
    const currentModeText = document.getElementById('currentMode');
    if(currentModeText) currentModeText.textContent = CONFIG.MODE_NAMES[STATE.currentMode];

    // Update button states
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.classList.remove('active');
        if (parseInt(btn.dataset.mode) === STATE.currentMode) {
            btn.classList.add('active');
        }
    });

    // Visualmente mostrar el contenedor de registros si estamos en modo programado
    const regContainer = document.getElementById('registerCardContainer');
    if(regContainer) {
        regContainer.style.display = STATE.currentMode === 2 ? 'block' : 'none';
    }
}

function updatePWMDisplay() {
    const pwmText = document.getElementById('currentPWM');
    const pwmFill = document.getElementById('pwmFill');
    
    if(pwmText) pwmText.textContent = STATE.currentPWM;
    
    if(pwmFill) {
        pwmFill.style.width = STATE.currentPWM + '%';
    }
}

/**
 * @brief Carga los registros programados guardados desde el ESP32
 */
async function loadSavedRegisters() {
    try {
        const response = await fetch(CONFIG.API_ENDPOINTS.GET_PROGRAMMED);
        if (response.ok) {
            const data = await response.json();
            if (data.registers && Array.isArray(data.registers)) {
                // Actualizar los campos de los registros con los valores guardados
                for (let i = 0; i < data.registers.length && i < CONFIG.NUM_REGISTERS; i++) {
                    const reg = data.registers[i];
                    document.getElementById(`regActive${i}`).checked = reg.active === 1;
                    document.getElementById(`regStart${i}`).value = reg.startTime;
                    document.getElementById(`regEnd${i}`).value = reg.endTime;
                    document.getElementById(`regT0${i}`).value = reg.tempMin;
                    document.getElementById(`regT100${i}`).value = reg.tempMax;
                    
                    // Actualizar el estado visual
                    updateRegisterItemVisualState(i, reg.active === 1);
                }
                console.log('Registros programados cargados desde ESP32');
            }
        }
    } catch (error) {
        console.warn('Error al cargar registros programados:', error);
    }
}

function updateActiveRegisterDisplay() {
    const registerCardContainer = document.getElementById('registerCardContainer');
    const activeRegisterText = document.getElementById('activeRegister');
    
    if(registerCardContainer && activeRegisterText) {
        if (STATE.currentMode === 2 && STATE.activeRegister !== null) {
            registerCardContainer.style.display = 'block';
            activeRegisterText.textContent = `Registro ${STATE.activeRegister + 1}`;
        } else {
            registerCardContainer.style.display = 'none';
        }
    }
}

function updateConnectionStatus() {
    const indicator = document.getElementById('connectionStatus');
    const text = document.getElementById('connectionText');

    if(indicator && text) {
        if (STATE.isConnected) {
            indicator.classList.remove('offline');
            text.textContent = 'Conectado';
        } else {
            indicator.classList.add('offline');
            text.textContent = 'Desconectado';
        }
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
    if(dot) {
        dot.style.opacity = '0.5';
        setTimeout(() => {
            dot.style.opacity = '1';
        }, 200);
    }
}

/**
 * Obtiene la hora del ESP32 sincronizada por SNTP
 */
async function updateTime() {
    try {
        const response = await fetch(CONFIG.API_ENDPOINTS.TIME);
        const headerTime = document.getElementById('headerTime');
        
        if (response.ok && headerTime) {
            const data = await response.json();
            if (data.time && data.time !== 'N/A') {
                headerTime.textContent = `🕐 ${data.time}`;
            } else {
                headerTime.textContent = 'Sincronizando hora...';
            }
        }
    } catch (error) {
        console.warn('Error al obtener la hora:', error);
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
    if(element) {
        element.textContent = message;
        element.className = `feedback-message show ${type}`;

        // Auto-hide after 5 seconds
        setTimeout(() => {
            element.classList.remove('show');
        }, 5000);
    }
}

// ========== CLEANUP ==========
window.addEventListener('beforeunload', () => {
    if (STATE.updateTimer) {
        clearInterval(STATE.updateTimer);
    }
});