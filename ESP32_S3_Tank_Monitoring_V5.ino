#include <WiFi.h>
#include <HTTPClient.h>
#include <SD.h>
#include <time.h>
#include <Preferences.h>
#include <HardwareSerial.h>
#include <WebServer.h>
#include <RTClib.h>
#include <Wire.h>
#include <FS.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h> // New: Include for HTTPS

// Global Variable
WebServer server(80);
RTC_DS3231 rtc;
WiFiClientSecure client; // Changed: Use WiFiClientSecure for HTTPS
HTTPClient http;
Preferences preferences;

HardwareSerial Ultrasonic_Sensor(1);

const int ULTRASONIC_RX_PIN = 18;
const int ULTRASONIC_TX_PIN = 17;
const int LIMIT_SWITCH_PIN = 15;
const int AC_VOLTAGE_PIN = 1;
const int FLOW_SENSOR_PIN = 36;
const int SD_CS_PIN = 40;
const int SD_SPI_MISO = 13;
const int SD_SPI_MOSI = 11;
const int SD_SPI_SCK = 12;
const int DS3231_SDA_PIN = 8;
const int DS3231_SCL_PIN = 9;
const int RTC_MODEM_POWER_RELAY_PIN = 48;

const int ZMPT_SAMPLE_COUNT = 500;
const float AC_COUNT_START_THRESHOLD_V = 210.0;
const int ZMPT_STABLE_DURATION_MS = 500;

const float ZMPT_CALIBRATION_MULTIPLIER = 200.0;
const float ZMPT_ADC_MAX_VALUE = 4095.0;
const float ZMPT_ADC_REFERENCE_VOLTAGE = 3.3;
const float ZMPT_ADC_TO_VOLTAGE = ZMPT_ADC_REFERENCE_VOLTAGE / ZMPT_ADC_MAX_VALUE;

// Konstanta WiFi & Sistem
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 3000;
const unsigned long NTP_SYNC_INTERVAL_MS = 10000;

#define AP_SSID "IoTMaxWin"
#define AP_PASSWORD "MAXwin@2025.,"

const int MAX_WIFI_RECONNECT_FAILURES = 200;

const char *SD_DATA_FILENAME = "/data.txt";
const char *SD_CONFIG_FILENAME = "/config.ini";
const char *SD_TEMP_FILENAME = "/temp_data.txt";
bool sdCardInitialized = false;

// Changed: New HTTPS Endpoints
const char *SERVER_URL_DATA = "https://api.unexa.cloud/sensor/device/tank/data";
const char *SERVER_URL_TANK_OPEN = "https://api.unexa.cloud/sensor/device/tank/open";
const char *SERVER_URL_BACKUP = "https://api.unexa.cloud/sensor/device/tank/data/backup";
const char *NTP_SERVER = "pool.ntp.org";
const float PULSES_PER_LITER = 450.0;

// New: Root CA Certificate
// This is the GTS Root R4 certificate provided valid until2036
const char* root_ca_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"MIICCTCCAY6gAwIBAgINAgPlwGjvYxqccpBQUjAKBggqhkjOPQQDAzBHMQswCQYD\n" \
"VQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEUMBIG\n" \
"A1UEAxMLR1RTIFJvb3QgUjQwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAwMDAw\n" \
"WjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2Vz\n" \
"IExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjQwdjAQBgcqhkjOPQIBBgUrgQQAIgNi\n" \
"AATzdHOnaItgrkO4NcWBMHtLSZ37wWHO5t5GvWvVYRg1rkDdc/eJkTBa6zzuhXyi\n" \
"QHY7qca4R9gq55KRanPpsXI5nymfopjTX15YhmUPoYRlBtHci8nHc8iMai/lxKvR\n" \
"HYqjQjBAMA4GA1UdDwEB/wQEAwIBhjAPBgNVHRMBAf8EBTADAQH/MB0GA1UdDgQW\n" \
"BBSATNbrdP9JNqPV2Py1PsVq8JQdjDAKBggqhkjOPQQDAwNpADBmAjEA6ED/g94D\n" \
"9J+uHXqnLrmvT/aDHQ4thQEd0dlq7A/Cr8deVl5c1RxYIigL9zC2L7F8AjEA8GE8\n" \
"p/SgguMh1YQdc4acLa/KNJvxn7kjNuK8YAOdgLOaVsjh4rsUecrNIdSUtUlD\n" \
"-----END CERTIFICATE-----\n";

char stored_wifi_ssid[32] = AP_SSID;
char stored_wifi_password[64] = AP_PASSWORD;

volatile unsigned int pulseCount = 0;
unsigned long lastFlowPulseTime = 0;
unsigned long lastFlowCalculationTime = 0;
int ultrasonicValue = -1;
long flowValue = -1;

static unsigned long lastSensorLogTime = 0;
const unsigned long SENSOR_LOG_INTERVAL_MS = 5000;
const unsigned long SENSOR_LOG_INTERVAL_MS_OFFLINE = 1000;

bool limitSwitchRawState = false;
bool limitSwitchDebouncedState = false;
unsigned long lastLimitSwitchDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY_MS = 50;

float acCurrentVoltageRMS = 0.0;
bool acPowerDetectedForCounting = false;
int acPowerStatus = 0;
unsigned long acPowerOnSessionStart = 0;
float totalAcRunningSeconds = 0.0;
unsigned long currentAcSessionSeconds = 0;
unsigned long lastAcRunningSecondsSaveTime = 0;
const unsigned long AC_RUN_TIME_SAVE_INTERVAL_MS = 60000;
float reportedAcRunningSeconds = 0.0;

bool isWifiConnected = false;
bool sendAlertNotification = false;

bool isRTCModemPowerOff = false;

const int MODEM_RESTART_HOUR_START = 22;
const int MODEM_RESTART_MINUTE_START = 0;
const int MODEM_RESTART_HOUR_END = 23;
const int MODEM_RESTART_MINUTE_END = 0;

const int SCHEDULED_RESTART_HOUR_1 = 17;
const int SCHEDULED_RESTART_MINUTE_1 = 0;
const int SCHEDULED_RESTART_HOUR_2 = 5;
const int SCHEDULED_RESTART_MINUTE_2 = 0;
static bool restartScheduledForToday = false;

unsigned long lastWifiReconnectAttemptTime = 0;
int wifiReconnectFailures = 0;
static unsigned long lastWifiStatusLogTime = 0;
const unsigned long WIFI_STATUS_LOG_INTERVAL_MS = 10000;

// Sinkronisasi RTC
unsigned long lastNtpSyncTime = 0;

// Timer untuk operasi bersamaan
unsigned long lastSensorDataSendTime = 0;
const unsigned long SEND_INTERVAL_DATA_MS = 500;

unsigned long lastSdBackupSendTime = 0;
const unsigned long SEND_INTERVAL_BACKUP_MS = 10;

unsigned long lastHeapLogTime = 0;
const unsigned long HEAP_LOG_INTERVAL_MS = 60000;

int httpConsecutiveFailures = 0;
const int MAX_HTTP_CONSECUTIVE_FAILURES_FOR_MODEM_RESTART = 10;

void setupPins();
void setupSerial();
bool setupUltrasonicSensor();
void setupFlowSensor();
void setupRTC();
bool setupSDCard(bool force_init = false);
void setupNVS();
void setupWebServer();
void initializeSystem();
void saveWifiCredentials(const String& ssid, const String& password);
void saveStaticIpConfiguration(const String& ip, const String& gateway, const String& subnet, const String& dns);
void loadWifiCredentials();
void connectToWiFi();
void startAPMode();
void WiFiEvent(WiFiEvent_t event);
void IRAM_ATTR pulseCounter();
void readUltrasonicSensor();
void readFlowSensor();
void readLimitSwitch();
float readACVoltageZMPT();
void updateAcPowerStatus();
String getCurrentTimestamp();
void storeDataInSdCard(const String& timestamp, const String& type, const String& payload);
bool sendHttpRequest(const String& timestamp, const String& payload, const char* url, int maxRetries, unsigned long retryDelay, bool canBeBackedUp = true, const String& logType = "HTTP_REQUEST");
void sendCurrentSensorDataTask();
void sendStoredDataFromSdCardTask();
void sendTankAlertNotification();
void loadAcRunningSeconds();
void saveAcRunningSeconds();
void controlRTCModemPower();
void syncTimeWithNTP(bool force);
void logSystemHealth();
void handleModemRestartLogic();

void IRAM_ATTR pulseCounter() {
  pulseCount++;
  lastFlowPulseTime = millis();
}

void handleRoot() {
  preferences.begin("static_ip_cfg", true);
  String savedIp = preferences.getString("ip", "");
  preferences.end();

  String htmlContent = F(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Tank Monitor Data</title>
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 0; background-color: #eef2f7; color: #333; display: flex; flex-direction: column; min-height: 100vh; }
        header { background-color: #2c3e50; color: #ffffff; padding: 20px 30px; text-align: center; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        h1 { margin: 0; font-size: 2.2em; }
        .container { flex: 1; display: flex; flex-direction: column; align-items: center; padding: 20px; box-sizing: border-box; }
        .data-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 20px; width: 100%; max-width: 1200px; margin-bottom: 30px; }
        .card { background-color: #ffffff; padding: 25px; border-radius: 10px; box-shadow: 0 4px 12px rgba(0,0,0,0.08); transition: transform 0.2s ease, box-shadow 0.2s ease; border-left: 5px solid #3498db; }
        .card:hover { transform: translateY(-5px); box-shadow: 0 6px 16px rgba(0,0,0,0.12); }
        .card h2 { color: #2c3e50; margin-top: 0; margin-bottom: 15px; font-size: 1.5em; border-bottom: 1px solid #eee; padding-bottom: 10px; }
        .card p { margin-bottom: 10px; font-size: 1.1em; line-height: 1.6; display: flex; justify-content: space-between; align-items: center; }
        .card p strong { color: #555; }
        .status-connected { color: #28a745; font-weight: bold; }
        .status-disconnected { color: #dc3545; font-weight: bold; }
        .button-group { display: flex; flex-wrap: wrap; justify-content: center; gap: 15px; margin-top: 20px; width: 100%; max-width: 800px; }
        .button-group button, .button-group a { background-color: #007bff; color: white; padding: 12px 25px; border: none; border-radius: 8px; cursor: pointer; font-size: 1.1em; text-decoration: none; display: inline-flex; align-items: center; justify-content: center; transition: background-color 0.2s ease, transform 0.2s ease; }
        .button-group button:hover, .button-group a:hover { background-color: #0056b3; transform: translateY(-2px); }
        .button-group button.red, .button-group a.red { background-color: #dc3545; }
        .button-group button.red:hover, .button-group a.red:hover { background-color: #c82333; }
        footer { background-color: #2c3e50; color: #ffffff; text-align: center; padding: 15px; font-size: 0.9em; margin-top: auto; }
        .modal { display: none; position: fixed; z-index: 1; left: 0; top: 0; width: 100%; height: 100%; overflow: auto; background-color: rgba(0,0,0,0.6); align-items: center; justify-content: center; }
        .modal-content { background-color: #fefefe; margin: auto; padding: 30px; border-radius: 10px; box-shadow: 0 5px 15px rgba(0,0,0,0.3); width: 80%; max-width: 600px; text-align: center; position: relative; }
        .close-button { color: #aaa; float: right; font-size: 28px; font-weight: bold; position: absolute; top: 10px; right: 20px; }
        .close-button:hover, .close-button:focus { color: black; text-decoration: none; cursor: pointer; }
        .modal-buttons button { margin: 10px; padding: 10px 20px; border-radius: 5px; cursor: pointer; }
        .modal-buttons .confirm { background-color: #28a745; color: white; border: none; }
        .modal-buttons .confirm:hover { background-color: #218838; }
        .modal-buttons .cancel { background-color: #6c757d; color: white; border: none; }
        .modal-buttons .cancel:hover { background-color: #5a6268; }
        pre { background-color: #f8f8f8; border: 1px solid #ddd; padding: 15px; border-radius: 5px; overflow-x: auto; max-height: 400px; text-align: left; white-space: pre-wrap; word-break: break-all;}
    </style>
</head>
<body>
    <header>
        <h1>Tank Monitor ESP32-S3</h1>
    </header>
    <div class="container">
        <div class="data-grid">
            <div class="card">
                <h2>Status Sistem</h2>
                <p><strong>WiFi Status:</strong> <span id="wifiStatus">Memuat...</span></p>
                <p><strong>Alamat IP:</strong> <span id="ipAddress">Memuat...</span></p>
                <p><strong>Waktu Saat Ini:</strong> <span id="currentTime">Memuat...</span></p>
            </div>
            <div class="card">
                <h2>Data Sensor</h2>
                <p><strong>Sensor Ultrasonik:</strong> <span id="ultrasonicValue">Memuat...</span> mm</p>
                <p><strong>Saklar Batas:</strong> <span id="limitSwitchState">Memuat...</span></p>
            </div>
            <div class="card">
                <h2>Tegangan & Waktu Kerja AC</h2>
                <p><strong>Status AC:</strong> <span id="acStatusText" class="">Memuat...</span></p>
                <p><strong>Waktu Kerja AC (Sesi):</strong> <span id="currentAcSessionFormatted">Memuat...</span></p>
                <p><strong>Total Waktu Kerja AC:</strong> <span id="totalAcRunningFormatted">Memuat...</span></p>
            </div>
        </div>
        <div class="button-group">
            <a href="/config">Konfigurasi Pengaturan</a>
            <button onclick="showModal('restart')">Restart ESP32</button>
            <button onclick="showModal('factoryReset')" class="red">Factory Reset</button>
            <button onclick="showModal('resetRunningHour')" class="red">Reset Running Hour AC</button>
            <button onclick="showModal('viewLogs')">Lihat Log</button>
            <button onclick="showModal('downloadLogs')">Unduh Log</button>
            <button onclick="showModal('deleteLogs')">Hapus Log</button>
        </div>
    </div>

    <footer>
        &copy; 2025 Sistem Pemantauan Tangki. Semua hak dilindungi undang-undang.
    </footer>

    <div id="restartModal" class="modal">
        <div class="modal-content">
            <span class="close-button" onclick="closeModal('restartModal')">&times;</span>
            <h2>Konfirmasi Restart</h2>
            <p>Anda yakin ingin me-restart ESP32?</p>
            <div class="modal-buttons">
                <button class="confirm" onclick="location.href='/restart'">Ya, Restart</button>
                <button class="cancel" onclick="closeModal('restartModal')">Batal</button>
            </div>
        </div>
    </div>

    <div id="factoryResetModal" class="modal">
        <div class="modal-content">
            <span class="close-button" onclick="closeModal('factoryResetModal')">&times;</span>
            <h2>Konfirmasi Factory Reset</h2>
            <p><strong>PERINGATAN:</strong> Ini akan menghapus semua konfigurasi WiFi dan data tersimpan di NVS dan SD Card! Anda yakin?</p>
            <div class="modal-buttons">
                <button class="confirm red" onclick="location.href='/factoryReset'">Ya, Reset</button>
                <button class="cancel" onclick="closeModal('factoryResetModal')">Batal</button>
            </div>
        </div>
    </div>

    <div id="resetRunningHourModal" class="modal">
        <div class="modal-content">
            <span class="close-button" onclick="closeModal('resetRunningHourModal')">&times;</span>
            <h2>Konfirmasi Reset Waktu Kerja</h2>
            <p>Anda yakin ingin mereset total waktu kerja AC ke 0?</p>
            <div class="modal-buttons">
                <button class="confirm red" onclick="location.href='/resetAcRunningHour'">Ya, Reset</button>
                <button class="cancel" onclick="closeModal('resetRunningHourModal')">Batal</button>
            </div>
        </div>
    </div>

    <div id="viewLogsModal" class="modal">
        <div class="modal-content">
            <span class="close-button" onclick="closeModal('viewLogsModal')">&times;</span>
            <h2>Log Sistem</h2>
            <pre id="logContent">Memuat log...</pre>
            <div class="modal-buttons">
                <button class="cancel" onclick="closeModal('viewLogsModal')">Tutup</button>
            </div>
        </div>
    </div>

    <div id="downloadLogsModal" class="modal">
        <div class="modal-content">
            <span class="close-button" onclick="closeModal('downloadLogsModal')">&times;</span>
            <h2>Konfirmasi Unduh Log</h2>
            <p>Anda yakin ingin mengunduh file log (/data.txt) dari SD Card?</p>
            <div class="modal-buttons">
                <button class="confirm" onclick="location.href='/downloadLogs'">Ya, Unduh</button>
                <button class="cancel" onclick="closeModal('downloadLogsModal')">Batal</button>
            </div>
        </div>
    </div>

    <div id="deleteLogsModal" class="modal">
        <div class="modal-content">
            <span class="close-button" onclick="closeModal('deleteLogsModal')">&times;</span>
            <h2>Konfirmasi Hapus Log</h2>
            <p><strong>PERINGATAN:</strong> Ini akan menghapus semua data log di SD Card! Anda yakin?</p>
            <div class="modal-buttons">
                <button class="confirm red" onclick="location.href='/deleteLogs'">Ya, Hapus</button>
                <button class="cancel" onclick="closeModal('deleteLogsModal')">Batal</button>
            </div>
        </div>
    </div>

    <script>
        let lastSuccessfulSensorData = null;

        function showModal(modalId) {
            const modal = document.getElementById(modalId + 'Modal');
            if (modal) {
                if (modalId === 'viewLogs') {
                    fetch('/viewLogs')
                        .then(response => response.text())
                        .then(data => {
                            document.getElementById('logContent').innerText = data;
                        })
                        .catch(error => {
                            console.error('Error fetching logs:', error);
                            document.getElementById('logContent').innerText = 'Gagal memuat log.';
                        });
                }
                modal.style.display = 'flex';
            }
        }

        function closeModal(modalId) {
            const modal = document.getElementById(modalId);
            if (modal) {
                modal.style.display = 'none';
            }
        }

        function formatSeconds(totalSeconds) {
            if (totalSeconds === -1 || typeof totalSeconds !== 'number' || isNaN(totalSeconds) || totalSeconds < 0) {
                return 'N/A';
            }
            const hours = Math.floor(totalSeconds / 3600);
            const minutes = Math.floor((totalSeconds % 3600) / 60);
            const seconds = Math.floor(totalSeconds % 60);

            const pad = (num) => num.toString().padStart(2, '0');
            return `${pad(hours)}:${pad(minutes)}:${pad(seconds)}`;
        }

        function updateUI(data, isError = false) {
            if (isError) {
                document.getElementById('wifiStatus').innerText = 'Terputus / Error';
                document.getElementById('wifiStatus').className = 'status-disconnected';
                document.getElementById('ipAddress').innerText = 'N/A';
                document.getElementById('currentTime').innerText = 'N/A';

                if (lastSuccessfulSensorData) {
                    data = lastSuccessfulSensorData;
                } else {
                    document.getElementById('ultrasonicValue').innerText = 'N/A';
                    document.getElementById('limitSwitchState').innerText = 'N/A';
                    document.getElementById('acStatusText').innerText = 'N/A';
                    document.getElementById('acStatusText').className = 'status-disconnected';
                    document.getElementById('currentAcSessionFormatted').innerText = 'N/A';
                    document.getElementById('totalAcRunningFormatted').innerText = 'N/A';
                    return;
                }
            } else {
                lastSuccessfulSensorData = data;
                document.getElementById('wifiStatus').innerText = data.isWifiConnected ? 'Terhubung' : 'Terputus';
                document.getElementById('wifiStatus').className = data.isWifiConnected ? 'status-connected' : 'status-disconnected';
                document.getElementById('ipAddress').innerText = data.ipAddress;
                document.getElementById('currentTime').innerText = data.currentTime;
            }

            document.getElementById('ultrasonicValue').innerText = (data.ultrasonicValue === -1) ? 'N/A' : data.ultrasonicValue;
            document.getElementById('limitSwitchState').innerText = (data.limitSwitchDebouncedState === false && data.ultrasonicValue === -1) ? 'N/A' : (data.limitSwitchDebouncedState ? 'TERBUKA' : 'TERTUTUP');

            let acStatusDisplay;
            let currentAcSessionDisplay = 'N/A';
            let totalAcRunningDisplay = 'N/A';

            if (data.acCurrentVoltageRMS === -1 || data.acPowerStatus === 0) {
                acStatusDisplay = 'OFF';
                document.getElementById('acStatusText').className = 'status-disconnected';
                currentAcSessionDisplay = formatSeconds(0);
                totalAcRunningDisplay = formatSeconds(data.totalAcRunningSeconds);
            } else {
                acStatusDisplay = 'ON';
                document.getElementById('acStatusText').className = 'status-connected';
                currentAcSessionDisplay = formatSeconds(data.currentAcSessionSeconds);
                totalAcRunningDisplay = formatSeconds(data.reportedAcRunningSeconds);
            }
            document.getElementById('acStatusText').innerText = acStatusDisplay;
            document.getElementById('currentAcSessionFormatted').innerText = currentAcSessionDisplay;
            document.getElementById('totalAcRunningFormatted').innerText = totalAcRunningDisplay;
        }

        function fetchSensorData() {
            fetch('/getSensorData')
                .then(response => {
                    if (!response.ok) {
                        console.error('HTTP Error: ' + response.status + ' ' + response.statusText);
                        updateUI(null, true);
                        throw new Error('Network response was not ok.');
                    }
                    return response.json();
                })
                .then(data => {
                    updateUI(data, false);
                })
                .catch(error => {
                    console.error('Error fetching sensor data:', error);
                    updateUI(null, true);
                });
        }

        setInterval(fetchSensorData, 1000);
        fetchSensorData();
    </script>
</body>
</html>
)rawliteral");
  server.send(200, F("text/html"), htmlContent);
}

void handleConfig() {
  preferences.begin("static_ip_cfg", true);
  String savedIp = preferences.getString("ip", "");
  String savedGateway = preferences.getString("gateway", "");
  String savedSubnet = preferences.getString("subnet", "");
  String savedDns = preferences.getString("dns", "");
  preferences.end();

  String htmlContent = F(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Konfigurasi WiFi</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; color: #333; }
        h1 { color: #0056b3; }
        label { display: block; margin-bottom: 8px; font-weight: bold; }
        input[type="text"], input[type="password"] { width: calc(100% - 22px); padding: 10px; margin-bottom: 15px; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box;}
        input[type="radio"] { margin-right: 5px; }
        input[type="submit"] { background-color: #28a745; color: white; padding: 10px 15px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; margin-top: 20px;}
        input[type="submit"]:hover { background-color: #218838; }
        .config-section { background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .button-back { background-color: #6c757d; color: white; padding: 10px 15px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; margin-top: 20px; text-decoration: none; display: inline-block;}
        .button-back:hover { background-color: #5a6268; }
    </style>
</head>
<body>
    <h1>Konfigurasi WiFi</h1>
    <div class="config-section">
        <form action="/saveConfig" method="post">
            <label for="ssid">SSID:</label>
            <input type="text" id="ssid" name="ssid" value=")");
  htmlContent += String(stored_wifi_ssid);
  htmlContent += F(R"rawliteral(""><br><br>
            <label for="password">Password:</label>
            <input type="password" id="password" name="password" value=")");
  htmlContent += String(stored_wifi_password);
  htmlContent += F(R"rawliteral(""><br><br>
            <label for="ipType">Tipe IP:</label>
            <input type="radio" id="dhcp" name="ipType" value="dhcp" )rawliteral");
  htmlContent += (savedIp.length() == 0 ? F("checked") : F(""));
  htmlContent += F(R"rawliteral("> <label for="dhcp">DHCP</label>
            <input type="radio" id="static" name="ipType" value="static" )rawliteral");
  htmlContent += (savedIp.length() > 0 ? F("checked") : F(""));
  htmlContent += F(R"rawliteral("> <label for="static">IP Statis</label><br><br>
            <div id="staticIpSettings" style="display: )rawliteral");
  htmlContent += (savedIp.length() > 0 ? F("block") : F("none"));
  htmlContent += F(R"rawliteral(;">
                <label for="ip">Alamat IP:</label>
                <input type="text" id="ip" name="ip" value=")");
  htmlContent += savedIp;
  htmlContent += F(R"rawliteral(""><br><br>
                <label for="gateway">Gateway:</label>
                <input type="text" id="gateway" name="gateway" value=")");
  htmlContent += savedGateway;
  htmlContent += F(R"rawliteral(""><br><br>
                <label for="subnet">Subnet Mask:</label>
                <input type="text" id="subnet" name="subnet" value=")");
  htmlContent += savedSubnet;
  htmlContent += F(R"rawliteral(""><br><br>
                <label for="dns">Server DNS:</label>
                <input type="text" id="dns" name="dns" value=")");
  htmlContent += savedDns;
  htmlContent += F(R"rawliteral(""><br><br>
            </div>
            <input type="submit" value="Simpan Konfigurasi dan Restart">
        </form>
        <a href="/" class="button-back">Kembali ke Beranda</a>
    </div>
    <script>
        const ipTypeRadios = document.getElementsByName('ipType');
        const staticIpSettings = document.getElementById('staticIpSettings');
        ipTypeRadios.forEach(radio => {
            radio.addEventListener('change', () => {
                if (radio.value === 'static') {
                    staticIpSettings.style.display = 'block';
                } else {
                    staticIpSettings.style.display = 'none';
                }
            });
        });
    </script>
</body>
</html>
)rawliteral");
  server.send(200, F("text/html"), htmlContent);
}

void handleSaveConfig() {
  String ssid = server.arg(F("ssid"));
  String password = server.arg(F("password"));
  String ipType = server.arg(F("ipType"));

  strncpy(stored_wifi_ssid, ssid.c_str(), sizeof(stored_wifi_ssid) - 1);
  stored_wifi_ssid[sizeof(stored_wifi_ssid) - 1] = '\0';
  strncpy(stored_wifi_password, password.c_str(), sizeof(stored_wifi_password) - 1);
  stored_wifi_password[sizeof(stored_wifi_password) - 1] = '\0';

  saveWifiCredentials(String(stored_wifi_ssid), String(stored_wifi_password));

  if (ipType == F("static")) {
    String ip = server.arg("ip");
    String gateway = server.arg("gateway");
    String subnet = server.arg("subnet");
    String dns = server.arg("dns");
    saveStaticIpConfiguration(ip, gateway, subnet, dns);
  } else {
    preferences.begin("static_ip_cfg", false);
    preferences.clear();
    preferences.end();
    Serial.println(F("[CFG] DHCP dipilih, IP statis dihapus."));
  }

  server.send(200, F("text/plain"), F("Pengaturan disimpan. Memulai ulang..."));
  delay(1000);
  ESP.restart();
}

void handleRestart() {
  server.send(200, F("text/plain"), F("Memulai ulang ESP32..."));
  delay(1000);
  ESP.restart();
}

void handleGetSensorData() {
    readUltrasonicSensor();
    readFlowSensor();
    readLimitSwitch();
    updateAcPowerStatus();

    StaticJsonDocument<512> doc;

    doc["isWifiConnected"] = isWifiConnected;
    doc["ipAddress"] = (isWifiConnected ? WiFi.localIP().toString() : String(F("N/A")));
    doc["currentTime"] = getCurrentTimestamp();
    doc["ultrasonicValue"] = ultrasonicValue;
    doc["limitSwitchDebouncedState"] = limitSwitchDebouncedState;

    doc["acCurrentVoltageRMS"] = acCurrentVoltageRMS;
    doc["acPowerStatus"] = acPowerDetectedForCounting ? 1 : 0;
    doc["currentAcSessionSeconds"] = currentAcSessionSeconds;
    doc["totalAcRunningSeconds"] = totalAcRunningSeconds;
    doc["reportedAcRunningSeconds"] = reportedAcRunningSeconds;

    String jsonResponse;
    serializeJson(doc, jsonResponse);

    server.send(200, "application/json", jsonResponse);
}

void handleFactoryReset() {
    Serial.println(F("[RESET] Melakukan Reset Pabrik..."));

    if (setupSDCard(true)) {
        if (SD.exists(SD_CONFIG_FILENAME)) {
            if (SD.remove(SD_CONFIG_FILENAME)) {
                Serial.println(F("[RESET] config.ini berhasil dihapus."));
            } else {
                Serial.println(F("[RESET ERROR] Gagal menghapus config.ini."));
            }
        }
        if (SD.exists(SD_DATA_FILENAME)) {
            if (SD.remove(SD_DATA_FILENAME)) {
                Serial.println(F("[RESET] data.txt berhasil dihapus."));
            } else {
                Serial.println(F("[RESET ERROR] Gagal menghapus data.txt."));
            }
        }
        if (SD.exists(SD_TEMP_FILENAME)) {
            if (SD.remove(SD_TEMP_FILENAME)) {
                Serial.println(F("[RESET] temp_data.txt berhasil dihapus."));
            } else {
                Serial.println(F("[RESET ERROR] Gagal menghapus temp_data.txt."));
            }
        }
    } else {
        Serial.println(F("[RESET] SD Gagal. Melanjutkan dengan reset NVS."));
    }

    preferences.begin("static_ip_cfg", false); preferences.clear(); preferences.end();
    Serial.println(F("[RESET] Pengaturan IP statis dihapus dari NVS."));

    preferences.begin("tank_monitor_nvs", false); preferences.clear(); preferences.end();
    Serial.println(F("[RESET] Data waktu berjalan AC dihapus dari NVS."));

    server.send(200, F("text/plain"), F("Reset Pabrik selesai. Memulai ulang ESP32..."));
    delay(1000);
    ESP.restart();
}

void handleResetAcRunningHour() {
    Serial.println(F("[AC] Menerima permintaan untuk MENGATUR ULANG Total Waktu Berjalan AC."));
    totalAcRunningSeconds = 0.0;
    saveAcRunningSeconds();
    Serial.println(F("[AC] Total Waktu Berjalan AC diatur ulang menjadi 0."));
    server.send(200, F("text/plain"), F("Total Waktu Berjalan AC berhasil diatur ulang menjadi 0. Memulai ulang ESP32..."));
    delay(1000);
    ESP.restart();
}

void handleViewLogs() {
    if (!setupSDCard()) {
        server.send(500, F("text/plain"), F("SD Gagal atau tidak tersedia."));
        return;
    }

    File dataFile = SD.open(SD_DATA_FILENAME, FILE_READ);
    if (!dataFile) {
        server.send(404, F("text/plain"), F("File log tidak ditemukan."));
        return;
    }

    String logContent = "";
    const int MAX_LOG_SIZE = 100 * 1024;
    size_t bytesRead = 0;
    while (dataFile.available() && bytesRead < MAX_LOG_SIZE) {
        char c = dataFile.read();
        logContent += c;
        bytesRead++;
    }
    dataFile.close();

    if (bytesRead >= MAX_LOG_SIZE) {
        logContent += F("\n--- Log dipotong karena ukuran ---");
    }
    server.send(200, F("text/plain"), logContent);
}

void handleDownloadLogs() {
    if (!setupSDCard()) {
        server.send(500, F("text/plain"), F("SD Gagal atau tidak tersedia."));
        return;
    }

    File dataFile = SD.open(SD_DATA_FILENAME, FILE_READ);
    if (!dataFile) {
        server.send(404, F("text/plain"), F("File log tidak ditemukan."));
        return;
    }

    server.sendHeader(F("Content-Disposition"), F("attachment; filename=\"data.txt\""));
    server.sendHeader(F("Content-Type"), F("text/plain"));
    server.streamFile(dataFile, F("text/plain"));
    dataFile.close();
}

void handleDeleteLogs() {
    if (!setupSDCard()) {
        server.send(500, F("text/plain"), F("SD Gagal atau tidak tersedia."));
        return;
    }

    if (SD.exists(SD_DATA_FILENAME)) {
        if (SD.remove(SD_DATA_FILENAME)) {
            Serial.println(F("[SD] File log /data.txt berhasil dihapus."));
            server.send(200, F("text/plain"), F("Log berhasil dihapus."));
        } else {
            Serial.println(F("[SD ERROR] Gagal menghapus file /data.txt."));
            server.send(500, F("text/plain"), F("Gagal menghapus file log."));
        }
    } else {
        Serial.println(F("[SD] File log /data.txt tidak ditemukan untuk dihapus."));
        server.send(404, F("text/plain"), F("File log tidak ditemukan untuk dihapus."));
    }
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println(F("[WiFi] Mode STA dimulai."));
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println(F("[WiFi] Terhubung ke AP."));
      isWifiConnected = true;
      wifiReconnectFailures = 0;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println(F("[WiFi] Terputus. Menjadwalkan koneksi ulang."));
      isWifiConnected = false;
      wifiReconnectFailures++;
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print(F("[WiFi] IP: "));
      Serial.println(WiFi.localIP());
      isWifiConnected = true;
      wifiReconnectFailures = 0;
      syncTimeWithNTP(true);
      break;
    case ARDUINO_EVENT_WIFI_AP_START:
      Serial.print(F("[WiFi] Mode AP dimulai. SSID: " AP_SSID " IP: "));
      Serial.println(WiFi.softAPIP());
      isWifiConnected = false;
      break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
      Serial.println(F("[WiFi] Mode AP dihentikan."));
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println(F("[WiFi] IP hilang."));
      isWifiConnected = false;
      break;
    default:
      break;
  }
}

void saveWifiCredentials(const String& ssid, const String& password) {
  if (!setupSDCard()) {
    Serial.println(F("[SD] Gagal menyimpan kredensial."));
    return;
  }
  File configFile = SD.open(SD_CONFIG_FILENAME, FILE_WRITE);
  if (!configFile) {
    Serial.println(F("[SD] Gagal membuka config.ini untuk ditulis."));
    return;
  }

  configFile.println(F("[WiFi]"));
  configFile.print(F("ssid="));
  configFile.println(ssid);
  configFile.print(F("password="));
  configFile.println(password);

  preferences.begin("static_ip_cfg", true);
  String staticIp = preferences.getString("ip", "");
  String staticGateway = preferences.getString("gateway", "");
  String staticSubnet = preferences.getString("subnet", "");
  String staticDns = preferences.getString("dns", "");
  preferences.end();

  if (staticIp.length() > 0 && staticGateway.length() > 0 && staticSubnet.length() > 0) {
    configFile.println(F("[Static IP]"));
    configFile.print(F("ip=")); configFile.println(staticIp);
    configFile.print(F("gateway=")); configFile.println(staticGateway);
    configFile.print(F("subnet=")); configFile.println(staticSubnet);
    configFile.print(F("dns=")); configFile.println(staticDns);
  }
  configFile.close();
  Serial.println(F("[SD] Kredensial WiFi & IP statis disimpan."));
}

void saveStaticIpConfiguration(const String& ip, const String& gateway, const String& subnet, const String& dns) {
  preferences.begin("static_ip_cfg", false);
  preferences.putString("ip", ip);
  preferences.putString("gateway", gateway);
  preferences.putString("subnet", subnet);
  preferences.putString("dns", dns);
  preferences.end();
  Serial.println(F("[NVS] Pengaturan IP statis disimpan."));
}

void loadWifiCredentials() {
  if (!setupSDCard()) {
    Serial.println(F("[SD] Gagal memuat kredensial. Menggunakan AP default."));
    strncpy(stored_wifi_ssid, AP_SSID, sizeof(stored_wifi_ssid) - 1);
    stored_wifi_ssid[sizeof(stored_wifi_ssid) - 1] = '\0';
    strncpy(stored_wifi_password, AP_PASSWORD, sizeof(stored_wifi_password) - 1);
    stored_wifi_password[sizeof(stored_wifi_password) - 1] = '\0';
    return;
  }

  if (!SD.exists(SD_CONFIG_FILENAME)) {
    Serial.println(F("[CFG] config.ini tidak ditemukan! Membuat dengan AP default."));
    saveWifiCredentials(String(AP_SSID), String(AP_PASSWORD));
    strncpy(stored_wifi_ssid, AP_SSID, sizeof(stored_wifi_ssid) - 1);
    stored_wifi_ssid[sizeof(stored_wifi_ssid) - 1] = '\0';
    strncpy(stored_wifi_password, AP_PASSWORD, sizeof(stored_wifi_password) - 1);
    stored_wifi_password[sizeof(stored_wifi_password) - 1] = '\0';
    return;
  }

  File configFile = SD.open(SD_CONFIG_FILENAME, FILE_READ);
  if (!configFile) {
    Serial.println(F("[CFG] Gagal membuka config.ini untuk dibaca."));
    return;
  }

  Serial.println(F("[CFG] Memuat kredensial WiFi dari SD."));
  String line;
  String tempSsid = "", tempPassword = "", tempStaticIp = "", tempStaticGateway = "", tempStaticSubnet = "", tempStaticDns = "";
  bool inWiFiSection = false;
  bool inStaticIpSection = false;

  while (configFile.available()) {
    line = configFile.readStringUntil('\n');
    line.trim();

    if (line == F("[WiFi]")) { inWiFiSection = true; inStaticIpSection = false; }
    else if (line == F("[Static IP]")) { inStaticIpSection = true; inWiFiSection = false; }
    else if (inWiFiSection) {
      if (line.startsWith(F("ssid="))) { tempSsid = line.substring(5); }
      else if (line.startsWith(F("password="))) { tempPassword = line.substring(9); }
    } else if (inStaticIpSection) {
      if (line.startsWith(F("ip="))) { tempStaticIp = line.substring(3); }
      else if (line.startsWith(F("gateway="))) { tempStaticGateway = line.substring(8); }
      else if (line.startsWith(F("subnet="))) { tempStaticSubnet = line.substring(7); }
      else if (line.startsWith(F("dns="))) { tempStaticDns = line.substring(4); }
    }
  }
  configFile.close();

  strncpy(stored_wifi_ssid, tempSsid.c_str(), sizeof(stored_wifi_ssid) - 1); stored_wifi_ssid[sizeof(stored_wifi_ssid) - 1] = '\0';
  strncpy(stored_wifi_password, tempPassword.c_str(), sizeof(stored_wifi_password) - 1); stored_wifi_password[sizeof(stored_wifi_password) - 1] = '\0';
  Serial.print(F("[CFG] SSID: ")); Serial.println(stored_wifi_ssid);

  preferences.begin("static_ip_cfg", false);
  if (tempStaticIp.length() > 0 && tempStaticGateway.length() > 0 && tempStaticSubnet.length() > 0) {
    IPAddress ip, gateway, subnet, dns;
    if (ip.fromString(tempStaticIp) && gateway.fromString(tempStaticGateway) && subnet.fromString(tempStaticSubnet)) {
      if (tempStaticDns.length() > 0 && dns.fromString(tempStaticDns)) {
        if (WiFi.config(ip, gateway, subnet, dns)) {
          Serial.println(F("[CFG] IP statis diterapkan."));
          preferences.putString("ip", tempStaticIp); preferences.putString("gateway", tempStaticGateway);
          preferences.putString("subnet", tempStaticSubnet); preferences.putString("dns", tempStaticDns);
        } else {
          Serial.println(F("[CFG] Gagal mengatur IP statis. Menggunakan DHCP.")); preferences.clear();
        }
      } else {
        if (WiFi.config(ip, gateway, subnet)) {
          Serial.println(F("[CFG] IP statis diterapkan (tanpa DNS)."));
          preferences.putString("ip", tempStaticIp); preferences.putString("gateway", tempStaticGateway);
          preferences.putString("subnet", tempStaticSubnet); preferences.remove("dns");
        } else {
          Serial.println(F("[CFG] Gagal mengatur IP statis (tanpa DNS). Using DHCP.")); preferences.clear();
        }
      }
    } else {
      Serial.println(F("[CFG] Format IP statis tidak valid. Menggunakan DHCP.")); preferences.clear();
    }
  } else {
    preferences.clear();
    Serial.println(F("[CFG] Tidak ada IP statis atau tidak lengkap. Menggunakan DHCP."));
  }
  preferences.end();
}

void connectToWiFi() {
  Serial.println(F("\n--- Manajer Koneksi WiFi ---"));
  loadWifiCredentials();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("[WiFi] Sudah terhubung."));
    isWifiConnected = true;
    return;
  }

  if (strlen(stored_wifi_ssid) == 0 || strlen(stored_wifi_password) == 0) {
    Serial.println(F("[WiFi] Tidak ada kredensial. Memulai mode AP."));
    startAPMode();
    return;
  }

  if (isRTCModemPowerOff) {
    Serial.println(F("[WiFi] Modem mati. Melewati koneksi."));
    if(WiFi.getMode() != WIFI_OFF) { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); }
    isWifiConnected = false;
    return;
  }

  if (WiFi.getMode() != WIFI_STA || WiFi.status() == WL_DISCONNECTED) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(stored_wifi_ssid, stored_wifi_password);
    Serial.print(F("[WiFi] Mencoba koneksi ke: ")); Serial.println(stored_wifi_ssid);
    lastWifiReconnectAttemptTime = millis();
    isWifiConnected = false;
  } else {
    Serial.println(F("[WiFi] Sudah dalam proses koneksi STA."));
  }
}

void startAPMode() {
  Serial.println(F("[WiFi] Memulai SoftAP..."));
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);
  if (apStarted) {
    Serial.print(F("[WiFi] IP AP: ")); Serial.println(WiFi.softAPIP());
    Serial.println(F("[WiFi] Terhubung ke " AP_SSID " untuk konfigurasi."));
  } else {
    Serial.println(F("[WiFi ERROR] Gagal memulai SoftAP."));
  }
  isWifiConnected = false;
}

void setupPins() {
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(RTC_MODEM_POWER_RELAY_PIN, OUTPUT);

  digitalWrite(RTC_MODEM_POWER_RELAY_PIN, HIGH);
  Serial.println(F("[SYS] Pin GPIO diinisialisasi."));

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.println(F("[SYS] ADC dikonfigurasi."));
}

void setupSerial() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\n--- SISTEM BOOTUP ---"));
}

bool setupUltrasonicSensor() {
  Ultrasonic_Sensor.begin(9600, SERIAL_8N1, ULTRASONIC_RX_PIN, ULTRASONIC_TX_PIN);
  if (!Ultrasonic_Sensor) {
    Serial.println(F("[ULTRASONIC ERROR] Gagal menginisialisasi UART1. Periksa konfigurasi pin."));
    return false;
  }
  Serial.println(F("[ULTRASONIC] UART1 diinisialisasi."));
  return true;
}

void setupFlowSensor() {
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);
  Serial.println(F("[FLOW] Interrupt sensor terpasang."));
}

void setupRTC() {
  Wire.begin(DS3231_SDA_PIN, DS3231_SCL_PIN);

  if (!rtc.begin()) {
    Serial.println(F("[RTC ERROR] Tidak dapat menemukan modul! Periksa kabel & daya."));
  } else {
    Serial.println(F("[RTC] DS3231 diinisialisasi."));

    if (rtc.lostPower() || rtc.now().year() < 2024 || rtc.now().unixtime() == 0) {
      Serial.println(F("[RTC] RTC kehilangan daya/tidak valid, mencoba mengatur dari waktu kompilasi."));
      DateTime compileTime = DateTime(F(__DATE__), F(__TIME__));

      DateTime customTime(compileTime.year(), compileTime.month(), compileTime.day(), 5, 30, 0);
      rtc.adjust(customTime);
      Serial.println(F("[RTC] Waktu RTC diatur ke 05:30:00 AM (waktu tanggal kompilasi)."));
    }
  }
}

bool setupSDCard(bool force_init) {
  if (sdCardInitialized && !force_init) {
    return true;
  }

  SPI.begin(SD_SPI_SCK, SD_SPI_MISO, SD_SPI_MOSI, SD_CS_PIN);

  if (!SD.begin(SD_CS_PIN, SPI, 8000000)) {
    Serial.println(F("[SD ERROR] Inisialisasi gagal! Periksa kabel & kartu."));
    sdCardInitialized = false;
    return false;
  }

  if (!sdCardInitialized) {
      Serial.println(F("[SD] Berhasil diinisialisasi."));
      sdCardInitialized = true;
  }

  if (!SD.exists(SD_DATA_FILENAME)) {
    File dataFile = SD.open(SD_DATA_FILENAME, FILE_WRITE);
    if (dataFile) {
      dataFile.println(F("Timestamp - Type - Payload"));
      dataFile.close();
      Serial.println(F("[SD] Membuat file data baru: /data.txt"));
    } else {
      Serial.println(F("[SD ERROR] Gagal membuat file /data.txt."));
    }
  }
  return true;
}

void setupNVS() {
  preferences.begin("tank_monitor_nvs", false);
  Serial.println(F("[NVS] Preferensi diinisialisasi."));
  loadAcRunningSeconds();
  preferences.end();
}

void setupWebServer() {
  server.on(F("/"), HTTP_GET, handleRoot);
  server.on(F("/config"), handleConfig);
  server.on(F("/saveConfig"), HTTP_POST, handleSaveConfig);
  server.on(F("/restart"), HTTP_GET, handleRestart);
  server.on(F("/getSensorData"), HTTP_GET, handleGetSensorData);
  server.on(F("/factoryReset"), HTTP_GET, handleFactoryReset);
  server.on(F("/resetAcRunningHour"), HTTP_GET, handleResetAcRunningHour);
  server.on(F("/viewLogs"), HTTP_GET, handleViewLogs);
  server.on(F("/downloadLogs"), HTTP_GET, handleDownloadLogs);
  server.on(F("/deleteLogs"), HTTP_GET, handleDeleteLogs);

  server.begin();
  Serial.println(F("[WEB] Server dimulai."));
}

void initializeSystem() {
  setupSerial();
  setupPins();
  setupNVS();
  setupRTC();

  setupSDCard(true);
  WiFi.onEvent(WiFiEvent);
  delay(500);

  connectToWiFi();
  setupWebServer();
  setupUltrasonicSensor();
  setupFlowSensor();
  readUltrasonicSensor();
  readFlowSensor();
  readLimitSwitch();
  updateAcPowerStatus();
  Serial.println(F("--- BOOTUP SELESAI ---"));
}

void setup() {
  // New: Set the Root CA certificate for the secure client
  client.setCACert(root_ca_cert);
  initializeSystem();
}

// Loop Utama
void loop() {
  server.handleClient();

  readUltrasonicSensor();
  readFlowSensor();
  readLimitSwitch();
  updateAcPowerStatus();

  controlRTCModemPower();
  handleModemRestartLogic();

  if (!isRTCModemPowerOff) {
    if (!isWifiConnected && WiFi.getMode() == WIFI_STA) {
        if (millis() - lastWifiReconnectAttemptTime >= WIFI_RECONNECT_INTERVAL_MS) {
            Serial.println(F("[WiFi] Menghubungkan ulang..."));
            WiFi.reconnect();
            lastWifiReconnectAttemptTime = millis();
            wifiReconnectFailures++;

            if (wifiReconnectFailures >= MAX_WIFI_RECONNECT_FAILURES) {
                Serial.printf(F("[WiFi KRITIS] %d+ upaya koneksi ulang gagal. Memulai ulang ESP32.\n"), MAX_WIFI_RECONNECT_FAILURES);
                delay(2000);
                ESP.restart();
            }
        } else if (millis() - lastWifiStatusLogTime >= WIFI_STATUS_LOG_INTERVAL_MS) {
            Serial.println(F("[WiFi] Terputus. Menunggu koneksi ulang."));
            lastWifiStatusLogTime = millis();
        }
    }

    if (isWifiConnected && (millis() - lastNtpSyncTime >= NTP_SYNC_INTERVAL_MS || rtc.now().year() < 2024 || rtc.now().unixtime() == 0)) {
      syncTimeWithNTP(false);
      lastNtpSyncTime = millis();
    }
  } else {
    if (WiFi.getMode() != WIFI_OFF) {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      Serial.println(F("[WiFi] Radio OFF (Modem OFF)."));
    }
  }

  static unsigned long lastOfflineDataStoreTime = 0;

  if (isWifiConnected && WiFi.getMode() == WIFI_STA && !isRTCModemPowerOff) {
    if (millis() - lastSensorDataSendTime >= SEND_INTERVAL_DATA_MS) {
      sendCurrentSensorDataTask();
      lastSensorDataSendTime = millis();
    }

    if (millis() - lastSdBackupSendTime >= SEND_INTERVAL_BACKUP_MS) {
      sendStoredDataFromSdCardTask();
      lastSdBackupSendTime = millis();
    }

    if (limitSwitchDebouncedState && !sendAlertNotification) {
      sendTankAlertNotification();
      sendAlertNotification = true;
    } else if (!limitSwitchDebouncedState) {
      sendAlertNotification = false;
    }
  } else {
    if (millis() - lastOfflineDataStoreTime >= SENSOR_LOG_INTERVAL_MS_OFFLINE) {

      if (setupSDCard()) {
        String currentTs = getCurrentTimestamp();
        int switchOpenStatus = limitSwitchDebouncedState ? 1 : 0;
        char dataPayloadBuffer[50];

        unsigned long offlinePayloadAcRunningSeconds = (acPowerStatus == 0) ? 0UL : reportedAcRunningSeconds;

        snprintf(dataPayloadBuffer, sizeof(dataPayloadBuffer), "\"%d|%d|%d|%lu\"",
                  ultrasonicValue, switchOpenStatus, acPowerStatus, offlinePayloadAcRunningSeconds);
        String dataPayloadJson = String(F("{\"data\": ")) + String(dataPayloadBuffer) + String(F("}"));

        Serial.print(F("[SD] Menulis data ke SD (offline): "));
        Serial.print(currentTs);
        Serial.print(F(" - SENSOR_DATA - "));
        Serial.println(dataPayloadJson);
        storeDataInSdCard(currentTs, F("SENSOR_DATA"), dataPayloadJson);

        if (limitSwitchDebouncedState && !sendAlertNotification) {
          String alertPayloadJson = F("{\"tank\": \"true\"}");
          Serial.print(F("[SD] Menulis peringatan ke SD (offline): "));
          Serial.print(currentTs);
          Serial.print(F(" - ALERT_NOTIFICATION - "));
          Serial.println(alertPayloadJson);
          storeDataInSdCard(currentTs, F("ALERT_NOTIFICATION"), alertPayloadJson);
          sendAlertNotification = true;
        } else if (!limitSwitchDebouncedState) {
          sendAlertNotification = false;
        }
      } else {
            Serial.println(F("[MAIN] Kartu SD tidak tersedia, tidak dapat menyimpan data offline."));
      }
      lastOfflineDataStoreTime = millis();
    }

    static unsigned long lastOfflineStatusLogTime = 0;
    if (millis() - lastOfflineStatusLogTime >= WIFI_STATUS_LOG_INTERVAL_MS) {
      if (WiFi.getMode() == WIFI_AP) {
        Serial.println(F("[MAIN] Mode AP. Menunggu konfigurasi."));
      } else if (isRTCModemPowerOff) {
        Serial.println(F("[MAIN] Modem OFF. Data dicatat ke SD."));
      } else {
        Serial.println(F("[MAIN] WiFi terputus. Data dicatat ke SD."));
      }
      lastOfflineStatusLogTime = millis();
    }
  }

  logSystemHealth();

  delay(500);
}

void readUltrasonicSensor() {
  int previousUltrasonicValue = ultrasonicValue;

  while(Ultrasonic_Sensor.available()) {
    Ultrasonic_Sensor.read();
  }

  unsigned long startTime = millis();
  const unsigned long ULTRASONIC_READ_TIMEOUT_MS = 500;

  while (millis() - startTime < ULTRASONIC_READ_TIMEOUT_MS) {
    if (Ultrasonic_Sensor.available()) {

      if (Ultrasonic_Sensor.read() == 0xFF) {

        unsigned long packetReadStartTime = millis();
        while (Ultrasonic_Sensor.available() < 3 && (millis() - packetReadStartTime < 50)) {
          delay(1);
        }

        if (Ultrasonic_Sensor.available() >= 3) {
          uint8_t data_high = Ultrasonic_Sensor.read();
          uint8_t data_low = Ultrasonic_Sensor.read();
          uint8_t checksum = Ultrasonic_Sensor.read();

          uint8_t calculated_checksum = (uint8_t)(0xFF + data_high + data_low);

          if (checksum == calculated_checksum) {
            int distance_mm = (data_high << 8) + data_low;
            if (distance_mm > 0 && distance_mm <= 3500) {
              ultrasonicValue = distance_mm;
            } else {
              ultrasonicValue = 0;
            }
            break;
          } else {
            Serial.println(F("[ULTRASONIC ERROR] Kesalahan checksum. Membuang paket."));
          }
        } else {
          Serial.println(F("[ULTRASONIC ERROR] Paket parsial setelah 0xFF."));
        }
      }
    }
    delay(5);
  }

  if (millis() - startTime >= ULTRASONIC_READ_TIMEOUT_MS && ultrasonicValue == previousUltrasonicValue && ultrasonicValue != 0) {
      ultrasonicValue = -1;
      Serial.println(F("[ULTRASONIC ERROR] Batas waktu pembacaan data ultrasonik."));
  }

  if (ultrasonicValue != previousUltrasonicValue || (millis() - lastSensorLogTime >= SENSOR_LOG_INTERVAL_MS)) {
    Serial.print(F("[SNS] Ultrasonik: "));
    if (ultrasonicValue == -1) { Serial.println(F("N/A"));}
    else if (ultrasonicValue == 0) { Serial.println(F("0 mm (Di Luar Jangkauan)"));}
    else { Serial.print(ultrasonicValue); Serial.println(F(" mm"));}
    lastSensorLogTime = millis();
  }
}

void readFlowSensor() {
  unsigned long currentTime = millis();
  if (currentTime - lastFlowCalculationTime >= 1000) {
    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
    unsigned int pulsePerSecond = pulseCount;
    pulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);

    float flowRate = (float)pulsePerSecond * 60.0 / PULSES_PER_LITER;
    long newFlowValue = (long)round(flowRate);

    if (newFlowValue != flowValue) {
        flowValue = newFlowValue;
        Serial.print(F("[SNS] Aliran: ")); Serial.print(flowValue); Serial.println(F(" L/menit"));
    }
    lastFlowCalculationTime = currentTime;
  }

  if (millis() - lastFlowPulseTime > 3000) {
    if (flowValue != 0 && flowValue != -1) {
      flowValue = 0;
      Serial.println(F("[SNS] Aliran: 0 L/menit (Tidak ada pulsa)."));
    } else if (flowValue == -1) {
        flowValue = 0;
    }
  }
}

void readLimitSwitch() {
  bool currentRawReading = digitalRead(LIMIT_SWITCH_PIN);

  if (currentRawReading != limitSwitchRawState) {
    lastLimitSwitchDebounceTime = millis();
    limitSwitchRawState = currentRawReading;
  }

  if ((millis() - lastLimitSwitchDebounceTime) > DEBOUNCE_DELAY_MS) {
    if (currentRawReading != limitSwitchDebouncedState) {
      limitSwitchDebouncedState = currentRawReading;
      Serial.print(F("[SNS] Saklar Batas: ")); Serial.println(limitSwitchDebouncedState ? F("TERBUKA") : F("TERTUTUP"));
    }
  }
}

float readACVoltageZMPT() {
  float maxADC = 0;
  float minADC = ZMPT_ADC_MAX_VALUE;

  for (int i = 0; i < ZMPT_SAMPLE_COUNT; i++) {
    int adcValue = analogRead(AC_VOLTAGE_PIN);
    if (adcValue > maxADC) maxADC = adcValue;
    if (adcValue < minADC) minADC = adcValue;
  }

  float peakToPeakAdcDiff = maxADC - minADC;
  if (peakToPeakAdcDiff < 50) { return 0.0; }

  float peakToPeakVoltageAtADC = peakToPeakAdcDiff * ZMPT_ADC_TO_VOLTAGE;
  float rmsVoltage = peakToPeakVoltageAtADC * ZMPT_CALIBRATION_MULTIPLIER;

  return rmsVoltage;
}

void updateAcPowerStatus() {
  float previousAcVoltageRMS = acCurrentVoltageRMS;
  acCurrentVoltageRMS = readACVoltageZMPT();
  bool currentAcDetectedForCounting = (acCurrentVoltageRMS >= AC_COUNT_START_THRESHOLD_V);

  if (fabs(acCurrentVoltageRMS - previousAcVoltageRMS) > 1.0 || (millis() - lastSensorLogTime >= SENSOR_LOG_INTERVAL_MS)) {
    Serial.print(F("[SNS] Tegangan AC: "));
    Serial.print(acCurrentVoltageRMS, 2);
    Serial.println(F("V"));
    lastSensorLogTime = millis();
  }

  static bool lastStableAcStateForCounting = false;
  static unsigned long lastAcStateChangeDetectTimeForCounting = 0;

  if (currentAcDetectedForCounting != lastStableAcStateForCounting) {
    if (lastAcStateChangeDetectTimeForCounting == 0) {
      lastAcStateChangeDetectTimeForCounting = millis();
    }

    if (millis() - lastAcStateChangeDetectTimeForCounting >= ZMPT_STABLE_DURATION_MS) {
      acPowerDetectedForCounting = currentAcDetectedForCounting;
      acPowerStatus = acPowerDetectedForCounting ? 1 : 0;
      lastStableAcStateForCounting = currentAcDetectedForCounting;
      lastAcStateChangeDetectTimeForCounting = 0;

      if (acPowerDetectedForCounting) {
        acPowerOnSessionStart = millis();
        currentAcSessionSeconds = 0;
        lastAcRunningSecondsSaveTime = millis();
        Serial.println(F("[AC] ON untuk penghitungan. Sesi baru dimulai."));
      } else {
        if (acPowerOnSessionStart > 0) {
          unsigned long sessionDurationMillis = millis() - acPowerOnSessionStart;
          totalAcRunningSeconds += (float)sessionDurationMillis / 1000.0;
          saveAcRunningSeconds();
          Serial.print(F("[AC] OFF untuk penghitungan. Ditambahkan "));
          Serial.print((float)sessionDurationMillis / 1000.0, 1);
          Serial.print(F("s. Total: ")); Serial.print(totalAcRunningSeconds, 0); Serial.println(F("s."));
          acPowerOnSessionStart = 0;
        } else {
          Serial.println(F("[AC] OFF, tidak ada sesi aktif untuk ditambahkan."));
        }
        currentAcSessionSeconds = 0;
        reportedAcRunningSeconds = totalAcRunningSeconds;
      }
    }
  } else {
    lastAcStateChangeDetectTimeForCounting = 0;
  }

  if (acPowerDetectedForCounting && acPowerOnSessionStart > 0) {
    currentAcSessionSeconds = (millis() - acPowerOnSessionStart) / 1000UL;
    reportedAcRunningSeconds = totalAcRunningSeconds + currentAcSessionSeconds;

    if (millis() - lastAcRunningSecondsSaveTime >= AC_RUN_TIME_SAVE_INTERVAL_MS) {
      saveAcRunningSeconds();
      Serial.print(F("[AC] ON. Berkala: Total disimpan ")); Serial.print(totalAcRunningSeconds, 0); Serial.println(F("s."));
      lastAcRunningSecondsSaveTime = millis();
    }
  } else {
    currentAcSessionSeconds = 0;
    reportedAcRunningSeconds = totalAcRunningSeconds;
  }
}

String getCurrentTimestamp() {
  DateTime rtcNow = rtc.now();

  if (rtc.lostPower() || rtcNow.year() < 2024 || rtcNow.unixtime() == 0) {

    Serial.println(F("[TIME ERROR] RTC tidak valid/tidak disinkronkan."));
    return F("RTC Not Valid/Unsynced");
  }

  time_t unixTime = rtcNow.unixtime();
  struct tm *timeinfo = gmtime(&unixTime);

  char timestamp[50];
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", timeinfo);

  return String(timestamp);
}

void storeDataInSdCard(const String& timestamp, const String& type, const String& payload) {
  if (!setupSDCard()) {
    Serial.println(F("[SD ERROR] Gagal menyimpan data (SD tidak siap)."));
    return;
  }
  File dataFile = SD.open(SD_DATA_FILENAME, FILE_APPEND);
  if (dataFile) {
    dataFile.print(timestamp); dataFile.print(F(" - "));
    dataFile.print(type); dataFile.print(F(" - "));
    dataFile.println(payload);
    dataFile.close();
  } else {
    Serial.println(F("[SD ERROR] Gagal membuka file data untuk ditulis!"));
  }
}

bool sendHttpRequest(const String& timestamp, const String& payload, const char* url, int maxRetries, unsigned long retryDelay, bool canBeBackedUp, const String& logType) {
  Serial.printf("[HTTP] Mengirim %s ke %s pada %s\n", logType.c_str(), url, timestamp.c_str());
  Serial.print(F("[HTTP] Payload: "));
  Serial.println(payload);

  if (!isWifiConnected || isRTCModemPowerOff) {
    if (canBeBackedUp) { storeDataInSdCard(timestamp, logType, payload); }
    Serial.printf("[HTTP] %s GAGAL! (Offline/Modem OFF). Status: N/A\n", logType.c_str());
    httpConsecutiveFailures++;
    return false;
  }

  int retryCount = 0;
  while (retryCount <= maxRetries) {
    if (retryCount > 0) {
      Serial.printf("[HTTP] Mengirim ulang %s ke %s (Upaya %d/%d)...\n", logType.c_str(), url, retryCount + 1, maxRetries + 1);
      Serial.print(F("[HTTP] Payload: "));
      Serial.println(payload);
    }

    http.begin(client, url); // Changed: Use the secure client
    http.addHeader(F("Content-Type"), F("application/json"));
    http.addHeader(F("x-sensor-token"), F("Ocxszwx6XO6T"));

    // testing token : 8KMnJV56Slmp
    // rasau token   : Ocxszwx6XO6T
    // sanggau token : YyIRBdaktEHK

    int httpResponseCode = http.POST(payload);
    String responsePayload = http.getString();
    http.end();

    Serial.printf("[HTTP] %s ke %s. Status: %d\n", logType.c_str(), url, httpResponseCode);
    Serial.printf("[HTTP] Respon server: %s\n", responsePayload.c_str());

    if (httpResponseCode == HTTP_CODE_OK || httpResponseCode == HTTP_CODE_CREATED || httpResponseCode == HTTP_CODE_ACCEPTED) {
      Serial.printf("[HTTP] %s berhasil dikirim.\n", logType.c_str());
      httpConsecutiveFailures = 0;
      return true;
    } else {
      httpConsecutiveFailures++;
      Serial.printf("[HTTP] %s GAGAL! Respon: %s\n", logType.c_str(), responsePayload.c_str());
      if (httpResponseCode == HTTP_CODE_UNAUTHORIZED || httpResponseCode == HTTP_CODE_FORBIDDEN) {
        Serial.println(F("[HTTP ERROR] Akses tidak sah. Periksa token atau konfigurasi server."));
      } else if (httpResponseCode == -1) {
        Serial.println(F("[HTTP ERROR] Koneksi ditolak atau tidak ada jaringan."));
      }

      retryCount++;
      if (retryCount <= maxRetries) {
        Serial.printf("[HTTP] Mencoba lagi dalam %.1fs...\n", (float)retryDelay / 1000.0);
        delay(retryDelay);
      }
    }
  }

  Serial.printf("[HTTP] %s GAGAL! (Upaya maksimal tercapai)\n", logType.c_str());
  if (canBeBackedUp) { storeDataInSdCard(timestamp, logType, payload); }
  return false;
}

void sendCurrentSensorDataTask() {
  String currentTs = getCurrentTimestamp();
  int switchOpenStatus = limitSwitchDebouncedState ? 1 : 0;
  char dataPayloadBuffer[50];

  unsigned long payloadAcRunningSecondsValue = (acPowerStatus == 0) ? 0UL : reportedAcRunningSeconds;

  snprintf(dataPayloadBuffer, sizeof(dataPayloadBuffer), "\"%d|%d|%d|%lu\"",
            ultrasonicValue, switchOpenStatus, acPowerStatus, payloadAcRunningSecondsValue);
  String dataPayloadJson = String(F("{\"data\": ")) + String(dataPayloadBuffer) + String(F("}"));

  Serial.print(F("[HTTP] Payload SENSOR_DATA: "));
  Serial.println(dataPayloadJson);

  sendHttpRequest(currentTs, dataPayloadJson, SERVER_URL_DATA, 3, 1300, true, "SENSOR_DATA");
}

void sendStoredDataFromSdCardTask() {
  static File dataFile;
  static File tempFile;
  static bool fileOpened = false;

  if (!fileOpened) {
    if (!setupSDCard()) {
      Serial.println(F("[SD] SD Gagal. Tidak dapat memproses cadangan."));
      return;
    }

    dataFile = SD.open(SD_DATA_FILENAME, FILE_READ);
    if (!dataFile) {
      Serial.println(F("[SD] File data cadangan tidak ditemukan."));
      fileOpened = false;
      return;
    }

    if (SD.exists(SD_TEMP_FILENAME)) {
        if (SD.remove(SD_TEMP_FILENAME)) {
            Serial.println(F("[SD] temp_data.txt lama berhasil dihapus."));
        } else {
            Serial.println(F("[SD ERROR] Gagal menghapus temp_data.txt lama."));
        }
    }
    tempFile = SD.open(SD_TEMP_FILENAME, FILE_WRITE);
    if (!tempFile) {
      Serial.println(F("[SD ERROR] Gagal membuat file sementara."));
      dataFile.close();
      fileOpened = false;
      return;
    }

    Serial.println(F("[SD] Memulai unggah data cadangan dari SD."));
    String headerLine = dataFile.readStringUntil('\n');
    tempFile.println(headerLine.length() > 0 ? headerLine : F("Timestamp - Type - Payload"));
    fileOpened = true;
  }

  if (dataFile.available()) {
    String line = dataFile.readStringUntil('\n');
    line.trim();

    if (line.length() > 0 && !line.startsWith(F("Timestamp"))) {
      int firstDelimiterIndex = line.indexOf(F(" - "));
      int secondDelimiterIndex = line.indexOf(F(" - "), firstDelimiterIndex + 3);

      if (firstDelimiterIndex != -1 && secondDelimiterIndex != -1) {
        String timestamp = line.substring(0, firstDelimiterIndex);
        String type = line.substring(firstDelimiterIndex + 3, secondDelimiterIndex);
        String payloadJson = line.substring(secondDelimiterIndex + 3);

        const char* currentUrl;
        String finalPayloadForServer;

        if (type == F("SENSOR_DATA")) {
          currentUrl = SERVER_URL_BACKUP;
          StaticJsonDocument<512> backupDoc;
          backupDoc["data"] = line;
          serializeJson(backupDoc, finalPayloadForServer);
        } else if (type == F("ALERT_NOTIFICATION")) {
          currentUrl = SERVER_URL_TANK_OPEN;
          if (payloadJson != F("{\"tank\": \"true\"}")) { payloadJson = F("{\"tank\": \"true\"}"); }
          finalPayloadForServer = payloadJson;
        } else {
          Serial.print(F("[SD] Tipe data tidak dikenal: ")); Serial.println(line);
          tempFile.println(line);
          return;
        }

        Serial.print(F("[HTTP] Mengirim cadangan dari SD Card (timestamp asli): "));
        Serial.println(timestamp);

        bool sentOK = sendHttpRequest(timestamp, finalPayloadForServer, currentUrl, 3, 2000, false, type);
        if (!sentOK) {
          Serial.println(F("[SD] Gagal mengirim cadangan. Baris akan disimpan kembali."));
          tempFile.println(line);
        } else {
          Serial.println(F("[SD] Data cadangan berhasil dikirim, dihapus dari antrean."));
        }
      } else {
        Serial.print(F("[SD ERROR] Baris yang salah format: ")); Serial.println(line);
        tempFile.println(line);
      }
    } else if (line.length() > 0) {
        tempFile.println(line);
    }
  } else {
    dataFile.close();
    tempFile.close();

    File checkTemp = SD.open(SD_TEMP_FILENAME, FILE_READ);
    if (checkTemp) {
      String checkHeader = checkTemp.readStringUntil('\n');
      size_t header_len = checkHeader.length() + (checkHeader.indexOf('\r') != -1 ? 1 : 0) + 1;

      if (checkTemp.size() <= header_len) {
          checkTemp.close();
          Serial.print(F("[SD] Mencoba menghapus: ")); Serial.println(SD_DATA_FILENAME);
          if (SD.remove(SD_DATA_FILENAME)) {
              Serial.println(F("[SD] data.txt lama berhasil dihapus (sebelum SD dikosongkan)."));
          } else {
              Serial.println(F("[SD ERROR] Gagal menghapus data.txt lama (sebelum SD dikosongkan)."));
          }
          Serial.print(F("[SD] Mencoba menghapus: ")); Serial.println(SD_TEMP_FILENAME);
          if (SD.remove(SD_TEMP_FILENAME)) {
              Serial.println(F("[SD] temp_data.txt kosong berhasil dihapus."));
          } else {
              Serial.println(F("[SD ERROR] Gagal menghapus temp_data.txt kosong."));
          }
          Serial.println(F("[SD] Semua data cadangan berhasil dikirim. SD dikosongkan."));
      } else {
          checkTemp.close();
          Serial.print(F("[SD] Mencoba menghapus: ")); Serial.println(SD_DATA_FILENAME);
          if (SD.remove(SD_DATA_FILENAME)) {
              Serial.println(F("[SD] data.txt lama berhasil dihapus (sebelum dipindahkan)."));
          } else {
              Serial.println(F("[SD ERROR] Gagal menghapus data.txt lama (sebelum dipindahkan)."));
          }
          Serial.print(F("[SD] Mencoba mengganti nama dari ")); Serial.print(SD_TEMP_FILENAME); Serial.print(F(" ke ")); Serial.println(SD_DATA_FILENAME);
          if (SD.rename(SD_TEMP_FILENAME, SD_DATA_FILENAME)) {
              Serial.println(F("[SD] temp_data.txt berhasil diganti nama menjadi data.txt."));
          } else {
              Serial.println(F("[SD ERROR] Gagal mengganti nama temp_data.txt menjadi data.txt."));
          }
          Serial.println(F("[SD] Tidak semua data cadangan terkirim. Sisanya dipindahkan."));
      }
    } else {
      Serial.println(F("[SD ERROR] Gagal memverifikasi file sementara."));
    }
    fileOpened = false;
  }
}

void sendTankAlertNotification() {
  String currentTs = getCurrentTimestamp();
  String alertPayloadJson = F("{\"tank\": \"true\"}");

  sendHttpRequest(currentTs, alertPayloadJson, SERVER_URL_TANK_OPEN, 3, 2000, true, "ALERT_NOTIFICATION");
}

void syncTimeWithNTP(bool force) {
  if (isRTCModemPowerOff || !isWifiConnected || WiFi.getMode() != WIFI_STA) {
    Serial.println(F("[NTP] Sinkronisasi dibatalkan: Modem OFF atau WiFi tidak terhubung."));
    return;
  }

  DateTime rtcNow = rtc.now();
  if (force || rtc.lostPower() || rtcNow.year() < 2024 || rtcNow.unixtime() == 0) {
    Serial.println(F("[NTP] Memulai sinkronisasi waktu NTP..."));

    configTime(0, 0, NTP_SERVER);

    struct tm timeinfo;
    unsigned long ntpStartTime = millis();
    bool timeSet = false;
    while(!timeSet && (millis() - ntpStartTime < 10000)) {
        delay(1);
        if(getLocalTime(&timeinfo)) { timeSet = true; }
        else { delay(500); Serial.print(F(".")); }
    }
    Serial.println();

    if (timeSet) {
      Serial.println(F("[NTP] Sinkronisasi NTP berhasil. Memperbarui RTC ke UTC."));
      rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                          timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
      lastNtpSyncTime = millis();

      Serial.print(F("[NTP] Waktu RTC yang baru disinkronkan (UTC): "));
      Serial.println(getCurrentTimestamp());
    } else {
      Serial.println(F("[NTP] Sinkronisasi NTP gagal."));
    }
  }
}

void controlRTCModemPower() {
  DateTime now = rtc.now();
  static bool loggedInvalidTimeOn = false;
  static bool loggedInvalidTimeOff = false;

  if (rtc.lostPower() || now.year() < 2024 || now.unixtime() == 0) {
    if (digitalRead(RTC_MODEM_POWER_RELAY_PIN) == LOW) {
      digitalWrite(RTC_MODEM_POWER_RELAY_PIN, HIGH);
      Serial.println(F("[PWR] RTC tidak valid. Modem ON."));
      delay(100);
      connectToWiFi();
    }
    isRTCModemPowerOff = false;
    if (!loggedInvalidTimeOn) {
        Serial.println(F("[PWR] RTC tidak valid, modem tetap ON."));
        loggedInvalidTimeOn = true; loggedInvalidTimeOff = false;
    }
    return;
  } else {
      if (loggedInvalidTimeOn || loggedInvalidTimeOff) {
        Serial.println(F("[PWR] RTC valid."));
        loggedInvalidTimeOn = false; loggedInvalidTimeOff = false;
      }
      if (now.hour() == 0 && now.minute() == 1 && restartScheduledForToday) {
        restartScheduledForToday = false;
        Serial.println(F("[RESTART] Flag restart harian direset."));
      }
  }

  int currentHour = now.hour();
  int currentMinute = now.minute();

  bool shouldModemBeOff = false;

  int startMinutes = MODEM_RESTART_HOUR_START * 60 + MODEM_RESTART_MINUTE_START;
  int endMinutes = MODEM_RESTART_HOUR_END * 60 + MODEM_RESTART_MINUTE_END;
  int currentTotalMinutes = currentHour * 60 + currentMinute;

  if (startMinutes < endMinutes) {
    if (currentTotalMinutes >= startMinutes && currentTotalMinutes < endMinutes) {
      shouldModemBeOff = true;
    }
  } else {
    if (currentTotalMinutes >= startMinutes || currentTotalMinutes < endMinutes) {
      shouldModemBeOff = true;
    }
  }

  if (shouldModemBeOff) {
    if (digitalRead(RTC_MODEM_POWER_RELAY_PIN) == HIGH) {
      digitalWrite(RTC_MODEM_POWER_RELAY_PIN, LOW);
      Serial.println(F("[PWR] Modem OFF (Scheduled restart)."));
      isRTCModemPowerOff = true;
      if (WiFi.getMode() != WIFI_OFF) { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); }
      loggedInvalidTimeOff = false; loggedInvalidTimeOn = false;
    } else {
      isRTCModemPowerOff = true;
      if (WiFi.getMode() != WIFI_OFF) { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); }
    }
  } else {
    if (digitalRead(RTC_MODEM_POWER_RELAY_PIN) == LOW) {
      digitalWrite(RTC_MODEM_POWER_RELAY_PIN, HIGH);
      Serial.println(F("[PWR] Modem ON."));
      isRTCModemPowerOff = false;
      delay(100);
      syncTimeWithNTP(true);
      connectToWiFi();
      loggedInvalidTimeOff = false; loggedInvalidTimeOn = false;
    } else {
      isRTCModemPowerOff = false;
    }
  }

  if (!restartScheduledForToday && !isRTCModemPowerOff) {
    if ((currentHour == SCHEDULED_RESTART_HOUR_1 && currentMinute == SCHEDULED_RESTART_MINUTE_1) ||
        (currentHour == SCHEDULED_RESTART_HOUR_2 && currentMinute == SCHEDULED_RESTART_MINUTE_2)) {

      static unsigned long lastRestartTriggerTime = 0;
      if (millis() - lastRestartTriggerTime > 60000) {
        Serial.printf("[RESTART] Jadwal restart terpicu pada %02d:%02d UTC. Memulai ulang ESP32...\n", currentHour, currentMinute);
        restartScheduledForToday = true;
        lastRestartTriggerTime = millis();
        delay(2000);
        ESP.restart();
      }
    }
  }
}

void handleModemRestartLogic() {
  enum ModemRestartState {
    MODEM_IDLE,
    MODEM_TURNING_OFF,
    MODEM_OFF_DELAY,
  };
  static ModemRestartState modemRestartCurrentState = MODEM_IDLE;
  static unsigned long lastModemPowerToggleTime = 0;
  const unsigned long MODEM_OFF_DURATION_MS = 5000;

  switch (modemRestartCurrentState) {
    case MODEM_IDLE:
      if (httpConsecutiveFailures >= MAX_HTTP_CONSECUTIVE_FAILURES_FOR_MODEM_RESTART) {
        if (isRTCModemPowerOff) {
          Serial.println(F("[MODEM RESTART] Modem dijadwalkan OFF. Melewati restart reaktif."));
          return;
        }

        Serial.printf(F("[MODEM RESTART] %d+ upaya pengiriman HTTP gagal berturut-turut. Memulai ulang modem...\n"), MAX_HTTP_CONSECUTIVE_FAILURES_FOR_MODEM_RESTART);

        digitalWrite(RTC_MODEM_POWER_RELAY_PIN, LOW);
        Serial.println(F("[MODEM RESTART] Modem dimatikan (relay LOW)."));
        lastModemPowerToggleTime = millis();
        modemRestartCurrentState = MODEM_OFF_DELAY;
      }
      break;

    case MODEM_OFF_DELAY:
      if (millis() - lastModemPowerToggleTime >= MODEM_OFF_DURATION_MS) {
        digitalWrite(RTC_MODEM_POWER_RELAY_PIN, HIGH);
        Serial.println(F("[MODEM RESTART] Modem dihidupkan (relay HIGH)."));
        httpConsecutiveFailures = 0;
        modemRestartCurrentState = MODEM_IDLE;
      }
      break;
  }
}

void loadAcRunningSeconds() {
  preferences.begin("tank_monitor_nvs", false);
  totalAcRunningSeconds = preferences.getFloat("ac_run_sec", 0.0);
  Serial.print(F("[NVS] Total waktu berjalan AC dimuat: ")); Serial.print(totalAcRunningSeconds, 0); Serial.println(F("s."));
  preferences.end();
}

void saveAcRunningSeconds() {
  preferences.begin("tank_monitor_nvs", false);
  preferences.putFloat("ac_run_sec", totalAcRunningSeconds);
  Serial.print(F("[NVS] Total waktu berjalan AC disimpan: ")); Serial.print(totalAcRunningSeconds, 0); Serial.println(F("s."));
  preferences.end();
}

void logSystemHealth() {
    if (millis() - lastHeapLogTime >= HEAP_LOG_INTERVAL_MS) {
        Serial.printf("[HEALTH] Heap Tersedia: %lu bytes\n", ESP.getFreeHeap());
        lastHeapLogTime = millis();
    }
}