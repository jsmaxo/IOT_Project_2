/*
 * CSE4052 - Group 08
 * Water Level and Water Quality Monitoring System - Project Part 2
 *
 * ESP32 #1 - MAIN CONTROLLER
 */

#define BLYNK_PRINT Serial
#include "secrets.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BlynkSimpleEsp32.h>
#include <esp_task_wdt.h>

// ---------------- PIN MAP ----------------
#define I2C_SDA             21
#define I2C_SCL             22

#define TRIG_PIN             5
#define ECHO_PIN            18

#define RELAY_PIN           26
#define BUZZER_PIN          25
#define RED_LED_PIN         32
#define BLUE_LED_PIN        33

#define CONFIG_BUTTON_PIN   27
#define FLOW_PIN            34

// UART2 to ESP32 #2
#define QUALITY_RX          16
#define QUALITY_TX          17
#define QUALITY_BAUD     115200

#define AP_SSID             "ESP32_AP"
#define AP_PASSWORD         "waterconfig"
#define WS_PORT              81

// Change this if your relay is active HIGH.
#define RELAY_ON_LEVEL      LOW
#define RELAY_OFF_LEVEL     HIGH

#define LED_ON_MS      5000UL    // LEDs stay on for 5 s
#define BUZZER_ON_MS   5000UL    // buzzer sounds for 10 s

LiquidCrystal_I2C lcd(0x27, 16, 2);
WebServer server(80);
WebSocketsServer webSocket(WS_PORT);
Preferences preferences;
HardwareSerial QualitySerial(2);

// ---------------- CONFIGURATION ----------------
struct SystemConfig {
  char deviceName[32];
  char wifiSsid[64];
  char wifiPass[64];

  float lowLevel;
  float fullLevel;
  float tdsLimit;

  float emptyTankDistance;
  float fullTankDistance;

  uint32_t sampleIntervalMs;
};

SystemConfig cfg;

// ---------------- DATA ----------------
struct QualityData {
  float tds;
  float ec;
  float temperature;
  uint32_t sequence;
};

struct FlowData {
  float flowLMin;
  float totalLitres;
};

QualityData quality = {0, 0, 25.0f, 0};
FlowData flowData = {0, 0};

float waterLevel = 0;
float waterHeightCm = 0;
bool pumpOn = false;
bool qualityAlert = false;
bool configMode = false;

// ---------------- RTOS ----------------
SemaphoreHandle_t dataMutex = nullptr;
SemaphoreHandle_t lcdMutex = nullptr;

QueueHandle_t qualityQueue = nullptr;
QueueHandle_t alertQueue = nullptr;

TaskHandle_t waterLevelTaskHandle = nullptr;
TaskHandle_t pumpTaskHandle = nullptr;
TaskHandle_t flowTaskHandle = nullptr;
TaskHandle_t communicationTaskHandle = nullptr;
TaskHandle_t displayTaskHandle = nullptr;
TaskHandle_t cloudTaskHandle = nullptr;
TaskHandle_t alertTaskHandle = nullptr;
TaskHandle_t webTaskHandle = nullptr;

// ---------------- ISR STATE ----------------
volatile uint32_t flowPulses = 0;
volatile uint32_t lastFlowInterruptUs = 0;

volatile uint32_t lastButtonInterruptMs = 0;
volatile bool runtimeButtonEvent = false;

// ---------------- ALERTS ----------------
enum AlertType : uint8_t {
  ALERT_LOW = 1,
  ALERT_FULL = 2,
  ALERT_QUALITY = 3,
  ALERT_BUTTON = 4
};

// ---------------- DEFAULT CONFIG ----------------
void setDefaultConfig() {
  memset(&cfg, 0, sizeof(cfg));

  strlcpy(cfg.deviceName, "Group08-WaterMonitor", sizeof(cfg.deviceName));

  // These are deliberately placeholders. Put your real Wi-Fi values
  // in the configuration portal or in NVS.
  strlcpy(cfg.wifiSsid, "", sizeof(cfg.wifiSsid));
  strlcpy(cfg.wifiPass, "", sizeof(cfg.wifiPass));

  cfg.lowLevel = 20.0f;
  cfg.fullLevel = 95.0f;
  cfg.tdsLimit = 500.0f;

  cfg.emptyTankDistance = 30.0f;
  cfg.fullTankDistance = 5.0f;

  cfg.sampleIntervalMs = 1000;
}

// ---------------- NVS: B1 ----------------
void loadConfig() {
  setDefaultConfig();

  preferences.begin("watercfg", true);

  String s;

  s = preferences.getString("name", cfg.deviceName);
  strlcpy(cfg.deviceName, s.c_str(), sizeof(cfg.deviceName));

  s = preferences.getString("ssid", cfg.wifiSsid);
  strlcpy(cfg.wifiSsid, s.c_str(), sizeof(cfg.wifiSsid));

  s = preferences.getString("pass", cfg.wifiPass);
  strlcpy(cfg.wifiPass, s.c_str(), sizeof(cfg.wifiPass));

  cfg.lowLevel =
      preferences.getFloat("low", cfg.lowLevel);

  cfg.fullLevel =
      preferences.getFloat("full", cfg.fullLevel);

  cfg.tdsLimit =
      preferences.getFloat("tds", cfg.tdsLimit);

  cfg.emptyTankDistance =
      preferences.getFloat("empty", cfg.emptyTankDistance);

  cfg.fullTankDistance =
      preferences.getFloat("fullDist", cfg.fullTankDistance);

  cfg.sampleIntervalMs =
      preferences.getULong("interval", cfg.sampleIntervalMs);

  preferences.end();

  // Safety validation
  cfg.lowLevel = constrain(cfg.lowLevel, 1.0f, 90.0f);
  cfg.fullLevel = constrain(cfg.fullLevel, 10.0f, 100.0f);
  cfg.tdsLimit = max(cfg.tdsLimit, 1.0f);
  cfg.emptyTankDistance = max(cfg.emptyTankDistance, 1.0f);
  cfg.fullTankDistance = max(cfg.fullTankDistance, 0.5f);
  cfg.sampleIntervalMs = constrain(cfg.sampleIntervalMs, 200UL, 60000UL);

  Serial.printf("Loaded cfg: low=%.1f fullLevel=%.1f empty=%.1f fullDist=%.1f\n",
                cfg.lowLevel, cfg.fullLevel,
                cfg.emptyTankDistance, cfg.fullTankDistance);
}

void saveConfig() {
  preferences.begin("watercfg", false);

  preferences.putString("name", cfg.deviceName);
  preferences.putString("ssid", cfg.wifiSsid);
  preferences.putString("pass", cfg.wifiPass);

  preferences.putFloat("low", cfg.lowLevel);
  preferences.putFloat("full", cfg.fullLevel);
  preferences.putFloat("tds", cfg.tdsLimit);

  preferences.putFloat("empty", cfg.emptyTankDistance);
  preferences.putFloat("fullDist", cfg.fullTankDistance);

  preferences.putULong("interval", cfg.sampleIntervalMs);

  preferences.end();
}

// ---------------- WATCHDOG: B2 ----------------
void initWatchdog() {
  esp_task_wdt_config_t twdtConfig = {
    .timeout_ms = 10000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };

  esp_task_wdt_init(&twdtConfig);
}

void registerCurrentTaskWithWatchdog() {
  esp_task_wdt_add(nullptr);
}

void feedWatchdog() {
  esp_task_wdt_reset();
}

// ---------------- INTERRUPTS: A3 ----------------
void IRAM_ATTR flowISR() {
  const uint32_t now = micros();

  // Ignore extremely close pulses/noise.
  if ((now - lastFlowInterruptUs) >= 3000) {
    flowPulses++;
    lastFlowInterruptUs = now;

    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (flowTaskHandle != nullptr) {
      vTaskNotifyGiveFromISR(
        flowTaskHandle,
        &higherPriorityTaskWoken
      );
    }

    if (higherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}

void IRAM_ATTR buttonISR() {
  const uint32_t now = millis();

  if ((now - lastButtonInterruptMs) >= 300) {
    lastButtonInterruptMs = now;
    runtimeButtonEvent = true;

    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (alertTaskHandle != nullptr) {
      xTaskNotifyFromISR(
        alertTaskHandle,
        0x01,
        eSetBits,
        &higherPriorityTaskWoken
      );
    }

    if (higherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}

// ---------------- WATER LEVEL ----------------
float readWaterDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 30 ms timeout prevents blocking forever.
  unsigned long duration =
      pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0) {
    return -1.0f;
  }

  return (duration * 0.0343f) / 2.0f;
}

float distanceToLevel(float distanceCm) {
  if (distanceCm < 0) {
    return waterLevel;
  }

  float span =
      cfg.emptyTankDistance - cfg.fullTankDistance;

  if (fabs(span) < 0.01f) {
    return 0;
  }

  float percentage =
      ((cfg.emptyTankDistance - distanceCm) / span) * 100.0f;

  return constrain(percentage, 0.0f, 100.0f);
}

float levelToHeight(float level) {
  float height =
      cfg.emptyTankDistance - cfg.fullTankDistance;

  return (level / 100.0f) * height;
}

// ---------------- UART QUALITY DATA ----------------
bool parseQualityFrame(const String &line, QualityData &out) {
  // Expected:
  // Q,sequence,tds,ec,temperature

  if (!line.startsWith("Q,")) {
    return false;
  }

  char buffer[160];
  line.toCharArray(buffer, sizeof(buffer));

  char *token = strtok(buffer, ",");
  if (!token) return false;

  token = strtok(nullptr, ",");
  if (!token) return false;
  out.sequence = strtoul(token, nullptr, 10);

  token = strtok(nullptr, ",");
  if (!token) return false;
  out.tds = atof(token);

  token = strtok(nullptr, ",");
  if (!token) return false;
  out.ec = atof(token);

  token = strtok(nullptr, ",");
  if (!token) return false;
  out.temperature = atof(token);

  return true;
}

// ---------------- WEB SOCKET ----------------
const char CONFIG_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Water Level and Water Quality Monitoring</title>
<style>
body{
  font-family:Arial,sans-serif;
  margin:18px;
  max-width:760px;
}
.card{
  border:1px solid #ccc;
  border-radius:12px;
  padding:16px;
  margin-bottom:14px;
}
.grid{
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:12px;
}
input{
  width:100%;
  padding:9px;
  box-sizing:border-box;
  margin-top:5px;
}
button{
  padding:11px 16px;
  margin-top:12px;
}
.value{
  font-size:1.2em;
  font-weight:bold;
}
@media(max-width:600px){
  .grid{grid-template-columns:1fr;}
}
</style>
</head>

<body>

<h2>Group 08 Water Monitor</h2>

<div class="card">
<h3>WebSocket Configuration</h3>

<p id="status">Connecting...</p>

<label>Device Name</label>
<input id="name">

<label>Wi-Fi SSID</label>
<input id="ssid">

<label>Wi-Fi Password</label>
<input id="pass" type="password">

<div class="grid">

<div>
<label>Low Water Level (%)</label>
<input id="low" type="number" step="0.1">
</div>

<div>
<label>Full Water Level (%)</label>
<input id="full" type="number" step="0.1">
</div>

<div>
<label>TDS Limit (ppm)</label>
<input id="tds" type="number" step="1">
</div>

<div>
<label>Empty Tank Distance (cm)</label>
<input id="empty" type="number" step="0.1">
</div>

<div>
<label>Full Tank Distance (cm)</label>
<input id="fullDist" type="number" step="0.1">
</div>

<div>
<label>Sampling Interval (ms)</label>
<input id="interval" type="number" min="200">
</div>

</div>

<button onclick="saveConfig()">Save Configuration</button>

<p>
Settings are stored in ESP32 NVS.
Reboot the ESP32 after changing Wi-Fi settings.
</p>

</div>

<div class="card">

<h3>Live Dashboard</h3>

<p>Water Level:
<span class="value" id="level">--</span> %</p>

<p>Water Height:
<span class="value" id="height">--</span> cm</p>

<p>TDS:
<span class="value" id="tdsLive">--</span> ppm</p>

<p>EC:
<span class="value" id="ec">--</span> mS/cm</p>

<p>Temperature:
<span class="value" id="temp">--</span> C</p>

<p>Flow:
<span class="value" id="flow">--</span> L/min</p>

<p>Total Water:
<span class="value" id="total">--</span> L</p>

<p>Pump:
<span class="value" id="pump">--</span></p>

</div>

<script>

let ws;

function connectWS(){

  ws = new WebSocket(
    "ws://" + location.hostname + ":81/"
  );

  ws.onopen = function(){

    document.getElementById("status").textContent =
      "WebSocket connected";

    ws.send(JSON.stringify({
      cmd:"getConfig"
    }));

  };

  ws.onclose = function(){

    document.getElementById("status").textContent =
      "Disconnected - retrying...";

    setTimeout(connectWS,1500);
  };

  ws.onmessage = function(event){

    let data;

    try{
      data = JSON.parse(event.data);
    }catch(e){
      return;
    }

    if(data.type === "config"){

      document.getElementById("name").value =
        data.name;

      document.getElementById("ssid").value =
        data.ssid;

      document.getElementById("pass").value =
        data.pass;

      document.getElementById("low").value =
        data.low;

      document.getElementById("full").value =
        data.full;

      document.getElementById("tds").value =
        data.tds;

      document.getElementById("empty").value =
        data.empty;

      document.getElementById("fullDist").value =
        data.fullDist;

      document.getElementById("interval").value =
        data.interval;
    }

    if(data.type === "live"){

      document.getElementById("level").textContent =
        Number(data.level).toFixed(1);

      document.getElementById("height").textContent =
        Number(data.height).toFixed(1);

      document.getElementById("tdsLive").textContent =
        Number(data.tds).toFixed(0);

      document.getElementById("ec").textContent =
        Number(data.ec).toFixed(3);

      document.getElementById("temp").textContent =
        Number(data.temp).toFixed(1);

      document.getElementById("flow").textContent =
        Number(data.flow).toFixed(2);

      document.getElementById("total").textContent =
        Number(data.total).toFixed(2);

      document.getElementById("pump").textContent =
        data.pump ? "ON" : "OFF";
    }
  };
}

function saveConfig(){

  const payload = {
    cmd:"save",

    name:document.getElementById("name").value,
    ssid:document.getElementById("ssid").value,
    pass:document.getElementById("pass").value,

    low:Number(document.getElementById("low").value),
    full:Number(document.getElementById("full").value),

    tds:Number(document.getElementById("tds").value),

    empty:Number(document.getElementById("empty").value),
    fullDist:Number(document.getElementById("fullDist").value),

    interval:Number(document.getElementById("interval").value)
  };

  ws.send(JSON.stringify(payload));

  alert(
    "Configuration saved to NVS. " +
    "Reboot after changing Wi-Fi settings."
  );
}

connectWS();

</script>

</body>
</html>
)HTML";

void sendConfigToClient(uint8_t client) {

  DynamicJsonDocument doc(1024);

  doc["type"] = "config";
  doc["name"] = cfg.deviceName;
  doc["ssid"] = cfg.wifiSsid;
  doc["pass"] = cfg.wifiPass;

  doc["low"] = cfg.lowLevel;
  doc["full"] = cfg.fullLevel;
  doc["tds"] = cfg.tdsLimit;

  doc["empty"] = cfg.emptyTankDistance;
  doc["fullDist"] = cfg.fullTankDistance;

  doc["interval"] = cfg.sampleIntervalMs;

  String json;
  serializeJson(doc, json);

  webSocket.sendTXT(client, json);
}

void broadcastLiveData() {

  QualityData q;
  FlowData f;
  float level;
  float height;
  bool pump;

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {

    q = quality;
    f = flowData;
    level = waterLevel;
    height = waterHeightCm;
    pump = pumpOn;

    xSemaphoreGive(dataMutex);

  } else {
    return;
  }

  DynamicJsonDocument doc(768);

  doc["type"] = "live";
  doc["level"] = level;
  doc["height"] = height;
  doc["tds"] = q.tds;
  doc["ec"] = q.ec;
  doc["temp"] = q.temperature;
  doc["flow"] = f.flowLMin;
  doc["total"] = f.totalLitres;
  doc["pump"] = pump;

  String json;
  serializeJson(doc, json);

  webSocket.broadcastTXT(json);
}

void webSocketEvent(
  uint8_t client,
  WStype_t type,
  uint8_t *payload,
  size_t length
) {

  if (type == WStype_CONNECTED) {

    sendConfigToClient(client);

    return;
  }

  if (type != WStype_TEXT) {
    return;
  }

  DynamicJsonDocument doc(2048);

  DeserializationError error =
      deserializeJson(doc, payload, length);

  if (error) {
    return;
  }

  const char *cmd = doc["cmd"] | "";

  if (strcmp(cmd, "getConfig") == 0) {

    sendConfigToClient(client);

    return;
  }

  if (strcmp(cmd, "save") == 0) {

    strlcpy(
      cfg.deviceName,
      doc["name"] | cfg.deviceName,
      sizeof(cfg.deviceName)
    );

    strlcpy(
      cfg.wifiSsid,
      doc["ssid"] | cfg.wifiSsid,
      sizeof(cfg.wifiSsid)
    );

    strlcpy(
      cfg.wifiPass,
      doc["pass"] | cfg.wifiPass,
      sizeof(cfg.wifiPass)
    );

    cfg.lowLevel =
        constrain(
          (float)(doc["low"] | cfg.lowLevel),
          1.0f,
          90.0f
        );

    cfg.fullLevel =
        constrain(
          (float)(doc["full"] | cfg.fullLevel),
          10.0f,
          100.0f
        );

    cfg.tdsLimit =
        max(
          (float)(doc["tds"] | cfg.tdsLimit),
          1.0f
        );

    cfg.emptyTankDistance =
        max(
          (float)(doc["empty"] | cfg.emptyTankDistance),
          1.0f
        );

    cfg.fullTankDistance =
        max(
          (float)(doc["fullDist"] | cfg.fullTankDistance),
          0.5f
        );

    cfg.sampleIntervalMs =
        constrain(
          (uint32_t)(doc["interval"] | cfg.sampleIntervalMs),
          200UL,
          60000UL
        );

    saveConfig();

    sendConfigToClient(client);
  }
}

// ---------------- WEB SERVER ----------------
void setupWebServer() {

  server.on("/", HTTP_GET, []() {
    server.send_P(
      200,
      "text/html",
      CONFIG_HTML
    );
  });

  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

// ---------------- WIFI ----------------
void startConfigurationAP() {

  WiFi.mode(WIFI_AP);

  WiFi.softAP(
    AP_SSID,
    AP_PASSWORD
  );

  Serial.println();
  Serial.println("================================");
  Serial.println("CONFIGURATION MODE");
  Serial.println("Connect to:");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("================================");
}

bool connectToNormalWiFi() {

  if (strlen(cfg.wifiSsid) == 0) {

    Serial.println(
      "Wi-Fi credentials are empty."
    );

    return false;
  }

  WiFi.mode(WIFI_STA);

  WiFi.begin(
    cfg.wifiSsid,
    cfg.wifiPass
  );

  Serial.print("Connecting to Wi-Fi");

  unsigned long start = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - start < 15000
  ) {

    Serial.print(".");
    delay(250);
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {

    Serial.print("Wi-Fi connected. IP: ");
    Serial.println(WiFi.localIP());

    return true;
  }

  Serial.println(
    "Wi-Fi connection failed."
  );

  return false;
}

// ---------------- TASK: WATER LEVEL ----------------
void taskWaterLevel(void *) {

  registerCurrentTaskWithWatchdog();

  TickType_t lastWake =
      xTaskGetTickCount();

  for (;;) {

    float distance =
        readWaterDistanceCm();

    if (distance >= 0) {

      float level =
          distanceToLevel(distance);

      float height =
          levelToHeight(level);

      if (xSemaphoreTake(
            dataMutex,
            pdMS_TO_TICKS(20)
          ) == pdTRUE) {

        waterLevel = level;
        waterHeightCm = height;

        xSemaphoreGive(dataMutex);
      }
    }

    feedWatchdog();

    vTaskDelayUntil(
      &lastWake,
      pdMS_TO_TICKS(cfg.sampleIntervalMs)
    );
  }
}

// ---------------- TASK: PUMP ----------------
void taskPump(void *) {

  registerCurrentTaskWithWatchdog();

  bool previousPumpState = false;
  bool lowAlertSent = false;
  bool fullAlertSent = false;

  for (;;) {

    float level;

    if (xSemaphoreTake(
          dataMutex,
          pdMS_TO_TICKS(20)
        ) == pdTRUE) {

      level = waterLevel;

      xSemaphoreGive(dataMutex);

    } else {

      feedWatchdog();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    bool newPumpState = pumpOn;

    // Hysteresis-style control:
    // ON at/below low threshold.
    // OFF at/above full threshold.
    if (level <= cfg.lowLevel) {
      newPumpState = true;
    }

    if (level >= cfg.fullLevel) {
      newPumpState = false;
    }

    if (newPumpState != previousPumpState) {

      digitalWrite(
        RELAY_PIN,
        newPumpState
          ? RELAY_ON_LEVEL
          : RELAY_OFF_LEVEL
      );

      if (xSemaphoreTake(
            dataMutex,
            pdMS_TO_TICKS(20)
          ) == pdTRUE) {

        pumpOn = newPumpState;

        xSemaphoreGive(dataMutex);
      }

      previousPumpState = newPumpState;

      Serial.print("Pump changed: ");
      Serial.println(
        newPumpState ? "ON" : "OFF"
      );
    }

    if (level <= cfg.lowLevel &&
        !lowAlertSent) {

      AlertType a = ALERT_LOW;
      xQueueSend(
        alertQueue,
        &a,
        0
      );

      lowAlertSent = true;
      fullAlertSent = false;
    }

    if (level >= cfg.fullLevel &&
        !fullAlertSent) {

      AlertType a = ALERT_FULL;
      xQueueSend(
        alertQueue,
        &a,
        0
      );

      fullAlertSent = true;
      lowAlertSent = false;
    }

    feedWatchdog();

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ---------------- TASK: FLOW ----------------
void taskFlow(void *) {

  registerCurrentTaskWithWatchdog();

  uint32_t lastPulseCount = 0;
  uint32_t lastCalculation = millis();

  for (;;) {

    // A3: wait for interrupt notification.
    // Timeout lets us calculate zero flow if no pulses arrive.
    ulTaskNotifyTake(
      pdTRUE,
      pdMS_TO_TICKS(1000)
    );

    uint32_t now = millis();

    if (now - lastCalculation >= 1000) {

      uint32_t pulses;

      noInterrupts();
      pulses = flowPulses;
      interrupts();

      uint32_t delta =
          pulses - lastPulseCount;

      lastPulseCount = pulses;


      float frequency =
          delta /
          ((now - lastCalculation) / 1000.0f);

      float flowLMin =
          frequency / 7.5f;

      float litres =
          flowLMin *
          ((now - lastCalculation) / 60000.0f);

      if (xSemaphoreTake(
            dataMutex,
            pdMS_TO_TICKS(20)
          ) == pdTRUE) {

        flowData.flowLMin = flowLMin;
        flowData.totalLitres += litres;

        xSemaphoreGive(dataMutex);
      }

      lastCalculation = now;
    }

    feedWatchdog();
  }
}

// ---------------- TASK: UART COMMUNICATION ----------------
void taskCommunication(void *) {

  registerCurrentTaskWithWatchdog();

  String line;

  for (;;) {

    while (QualitySerial.available()) {

      char c =
          (char)QualitySerial.read();

      if (c == '\n') {

        QualityData received;

        if (parseQualityFrame(
              line,
              received
            )) {

          if (xQueueSend(
                qualityQueue,
                &received,
                pdMS_TO_TICKS(20)
              ) != pdTRUE) {

            Serial.println(
              "Quality queue full"
            );
          }
        }

        line = "";

      } else if (c != '\r') {

        if (line.length() < 150) {
          line += c;
        } else {
          line = "";
        }
      }
    }

    // Process queued data.
    QualityData received;

    if (xQueueReceive(
          qualityQueue,
          &received,
          pdMS_TO_TICKS(20)
        ) == pdTRUE) {

      if (xSemaphoreTake(
            dataMutex,
            pdMS_TO_TICKS(20)
          ) == pdTRUE) {

        quality = received;

        xSemaphoreGive(dataMutex);
      }

      Serial.printf(
        "RX Quality -> TDS: %.1f ppm | EC: %.3f mS/cm | Temp: %.1f C\n",
        received.tds,
        received.ec,
        received.temperature
      );
    }

    feedWatchdog();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------------- TASK: LCD ----------------
void taskDisplay(void *) {

  registerCurrentTaskWithWatchdog();

  bool page = false;

  for (;;) {

    QualityData q;
    FlowData f;
    float level;
    float height;
    bool pump;

    if (xSemaphoreTake(
          dataMutex,
          pdMS_TO_TICKS(20)
        ) == pdTRUE) {

      q = quality;
      f = flowData;
      level = waterLevel;
      height = waterHeightCm;
      pump = pumpOn;

      xSemaphoreGive(dataMutex);

    } else {

      feedWatchdog();
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (xSemaphoreTake(
          lcdMutex,
          pdMS_TO_TICKS(50)
        ) == pdTRUE) {

      lcd.clear();

      if (!page) {

        lcd.setCursor(0, 0);
        lcd.print("Level:");
        lcd.print(level);
        lcd.print("%");

        lcd.setCursor(0, 1);
        lcd.print("H:");
        lcd.print(height, 1);
        lcd.print("cm ");

        lcd.print(
          pump ? "P:ON" : "P:OFF"
        );

      } else {

        lcd.setCursor(0, 0);
        lcd.print("TDS:");
        lcd.print(q.tds, 0);
        lcd.print("ppm");

        lcd.setCursor(0, 1);
        lcd.print("T:");
        lcd.print(q.temperature, 1);
        lcd.print("C ");
        lcd.print("F:");
        lcd.print(f.flowLMin, 1);
      }

      xSemaphoreGive(lcdMutex);
    }

    page = !page;

    feedWatchdog();

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ---------------- TASK: ALERTS ----------------
void taskAlerts(void *) {
  registerCurrentTaskWithWatchdog();

  bool ledActive = false;
  bool buzzerActive = false;
  uint32_t ledOffAt = 0;
  uint32_t buzzerOffAt = 0;

  for (;;) {
    // Button interrupt event: short beep (unless an alert is already sounding)
    uint32_t notificationValue = 0;
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &notificationValue,
                        pdMS_TO_TICKS(50)) == pdTRUE) {
      if ((notificationValue & 0x01) && !buzzerActive) {
        Serial.println("Button interrupt event");
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerActive = true;
        buzzerOffAt = millis() + 100;
      }
    }

    // New alert from the queue
    AlertType alert;
    if (xQueueReceive(alertQueue, &alert, 0) == pdTRUE) {
      bool valid = true;

      if (alert == ALERT_LOW) {
        digitalWrite(RED_LED_PIN, HIGH);
        digitalWrite(BLUE_LED_PIN, LOW);
        Serial.println("ALERT: LOW WATER LEVEL");
      } else if (alert == ALERT_FULL) {
        digitalWrite(BLUE_LED_PIN, HIGH);
        digitalWrite(RED_LED_PIN, LOW);
        Serial.println("ALERT: TANK FULL");
      } else if (alert == ALERT_QUALITY) {
        digitalWrite(RED_LED_PIN, HIGH);
        Serial.println("ALERT: WATER QUALITY");
      } else {
        valid = false;
      }

      if (valid) {
        uint32_t now = millis();
        ledActive = true;
        ledOffAt = now + LED_ON_MS;
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerActive = true;
        buzzerOffAt = now + BUZZER_ON_MS;
      }
    }

    // Turn things off when their time is up
    uint32_t now = millis();
    if (ledActive && (int32_t)(now - ledOffAt) >= 0) {
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(BLUE_LED_PIN, LOW);
      ledActive = false;
    }
    if (buzzerActive && (int32_t)(now - buzzerOffAt) >= 0) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerActive = false;
    }

    feedWatchdog();
  }
}

// ---------------- TASK: BLYNK ----------------
void taskCloud(void *) {

  registerCurrentTaskWithWatchdog();

  if (!configMode &&
      WiFi.status() == WL_CONNECTED) {

    Blynk.config(
      BLYNK_AUTH_TOKEN
    );

    Blynk.connect(5000);
  }

  unsigned long lastUpload = 0;

  for (;;) {

    if (!configMode &&
        WiFi.status() == WL_CONNECTED) {

      Blynk.run();

      if (millis() - lastUpload >= 1000) {

        QualityData q;
        FlowData f;
        float level;
        float height;
        bool pump;

        if (xSemaphoreTake(
              dataMutex,
              pdMS_TO_TICKS(20)
            ) == pdTRUE) {

          q = quality;
          f = flowData;
          level = waterLevel;
          height = waterHeightCm;
          pump = pumpOn;

          xSemaphoreGive(dataMutex);

          Blynk.virtualWrite(
            V0,
            level
          );

          Blynk.virtualWrite(
            V1,
            height
          );

          Blynk.virtualWrite(
            V2,
            q.tds
          );

          Blynk.virtualWrite(
            V3,
            q.ec
          );

          Blynk.virtualWrite(
            V4,
            q.temperature
          );

          Blynk.virtualWrite(
            V5,
            f.flowLMin
          );

          Blynk.virtualWrite(
            V6,
            pump ? 1 : 0
          );
        }

        lastUpload = millis();
      }
    }

    feedWatchdog();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------------- TASK: WEB ----------------
void taskWeb(void *) {

  registerCurrentTaskWithWatchdog();

  unsigned long lastBroadcast = 0;

  for (;;) {

    server.handleClient();
    webSocket.loop();

    if (millis() - lastBroadcast >= 1000) {

      broadcastLiveData();

      lastBroadcast = millis();
    }

    feedWatchdog();

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ---------------- SETUP ----------------
void setup() {

  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(
    "=========================================="
  );
  Serial.println(
    "GROUP 08 - WATER MONITORING PART 2"
  );
  Serial.println(
    "ESP32 #1 MAIN CONTROLLER"
  );
  Serial.println(
    "=========================================="
  );

  // GPIO
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  pinMode(
    CONFIG_BUTTON_PIN,
    INPUT_PULLUP
  );

  // GPIO34 has no internal pull-up.
  // Use external pull-up for YF-S201 if required.
  pinMode(FLOW_PIN, INPUT);

  digitalWrite(
    RELAY_PIN,
    RELAY_OFF_LEVEL
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  digitalWrite(
    RED_LED_PIN,
    LOW
  );

  digitalWrite(
    BLUE_LED_PIN,
    LOW
  );

  // I2C / LCD
  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );

  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Water Level");
  lcd.setCursor(0, 1);
  lcd.print("Water Quality");

  delay(1500);

  // Load persistent settings.
  loadConfig();

  // Check button while booting.
  // Requirement: hold button during reset to enter configuration mode.
  configMode =
      digitalRead(CONFIG_BUTTON_PIN) == LOW;

  // UART to quality node.
  QualitySerial.begin(
    QUALITY_BAUD,
    SERIAL_8N1,
    QUALITY_RX,
    QUALITY_TX
  );

  // RTOS objects.
  dataMutex =
      xSemaphoreCreateMutex();

  lcdMutex =
      xSemaphoreCreateMutex();

  qualityQueue =
      xQueueCreate(
        5,
        sizeof(QualityData)
      );

  alertQueue =
      xQueueCreate(
        5,
        sizeof(AlertType)
      );

  if (!dataMutex ||
      !lcdMutex ||
      !qualityQueue ||
      !alertQueue) {

    Serial.println(
      "RTOS object creation failed."
    );

    while (true) {
      delay(1000);
    }
  }

  // Watchdog.
  initWatchdog();

  // Interrupts: A3.
  attachInterrupt(
    digitalPinToInterrupt(FLOW_PIN),
    flowISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(CONFIG_BUTTON_PIN),
    buttonISR,
    FALLING
  );

  // Network.
  if (configMode) {

    startConfigurationAP();

  } else {

    if (!connectToNormalWiFi()) {

      Serial.println(
        "Starting AP because normal Wi-Fi failed."
      );

      startConfigurationAP();

      configMode = true;
    }
  }

  setupWebServer();

  // FreeRTOS tasks: A1/B4.
  xTaskCreatePinnedToCore(
    taskWaterLevel,
    "WaterLevel",
    4096,
    nullptr,
    5,
    &waterLevelTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    taskPump,
    "PumpControl",
    3072,
    nullptr,
    4,
    &pumpTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    taskFlow,
    "Flow",
    3072,
    nullptr,
    4,
    &flowTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    taskCommunication,
    "QualityUART",
    4096,
    nullptr,
    4,
    &communicationTaskHandle,
    0
  );

  xTaskCreatePinnedToCore(
    taskDisplay,
    "LCD",
    3072,
    nullptr,
    2,
    &displayTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    taskCloud,
    "Blynk",
    4096,
    nullptr,
    3,
    &cloudTaskHandle,
    0
  );

  xTaskCreatePinnedToCore(
    taskAlerts,
    "Alerts",
    3072,
    nullptr,
    4,
    &alertTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    taskWeb,
    "WebSocket",
    6144,
    nullptr,
    3,
    &webTaskHandle,
    0
  );

  Serial.println(
    "All FreeRTOS tasks started."
  );

  if (configMode) {
    Serial.println(
      "Configuration mode active."
    );
  }
}

// Arduino loop remains intentionally lightweight.
// Application work is performed by FreeRTOS tasks.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
