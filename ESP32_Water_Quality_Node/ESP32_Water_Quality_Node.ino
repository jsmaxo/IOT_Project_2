/*
 * CSE4052 - Group 08
 * Water Level and Water Quality Monitoring System - Project Part 2
 *
 * ESP32 #2 - DISTRIBUTED WATER QUALITY NODE
 *
 * Sensors:
 *   - Existing TDS sensor
 *   - Existing EC calculation
 *   - DS18B20 waterproof temperature sensor
 *
 * Display:
 *   - 16x2 I2C LCD
 *
 * Mandatory:
 *   A1 - FreeRTOS multitasking
 *   A4 - Distributed ESP32 system using UART
 *
 * Optional:
 *   B2 - Watchdog
 *   B4 - Mutex / task-based architecture
 *
 * UART frame sent to ESP32 #1:
 *   Q,sequence,tds,ec,temperature
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_task_wdt.h>

// =====================================================
// PIN MAP
// =====================================================

#define TDS_PIN          34
#define DS18B20_PIN       4

// UART2
#define UART_RX          16
#define UART_TX          17
#define UART_BAUD    115200

// I2C LCD
#define LCD_SDA          21
#define LCD_SCL          22
#define LCD_ADDRESS    0x27

// =====================================================
// TDS CALIBRATION
// =====================================================

const float TDS_VREF = 3.3f;
const float TDS_EC_CALIBRATION = 1.0f;
const float TDS_OFFSET = 0.14f;

// =====================================================
// HARDWARE OBJECTS
// =====================================================

HardwareSerial MainSerial(2);

OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

LiquidCrystal_I2C lcd(
  LCD_ADDRESS,
  16,
  2
);

// =====================================================
// SHARED DATA
// =====================================================

struct QualityData {

  float tds;
  float ec;
  float temperature;

  uint32_t sequence;
};

QualityData quality = {

  0.0f,
  0.0f,
  25.0f,
  0
};

// Mutex
SemaphoreHandle_t dataMutex = nullptr;

// =====================================================
// TASK HANDLES
// =====================================================

TaskHandle_t tdsTaskHandle = nullptr;

TaskHandle_t temperatureTaskHandle = nullptr;

TaskHandle_t communicationTaskHandle = nullptr;

TaskHandle_t lcdTaskHandle = nullptr;

// =====================================================
// WATCHDOG
// =====================================================

void initWatchdog() {

  esp_task_wdt_config_t twdtConfig = {

    .timeout_ms = 10000,

    .idle_core_mask = 0,

    .trigger_panic = true
  };

  esp_task_wdt_init(&twdtConfig);
}

// -----------------------------------------------------

void registerCurrentTaskWithWatchdog() {

  esp_task_wdt_add(nullptr);
}

// -----------------------------------------------------

void feedWatchdog() {

  esp_task_wdt_reset();
}

// =====================================================
// ADC READING
// =====================================================

float readAverageADC(
  uint8_t pin,
  uint8_t samples = 20
) {

  uint32_t sum = 0;

  for (
    uint8_t i = 0;
    i < samples;
    i++
  ) {

    sum += analogRead(pin);

    vTaskDelay(
      pdMS_TO_TICKS(2)
    );
  }

  return
    (float)sum /
    (float)samples;
}

// =====================================================
// TDS TASK
// =====================================================

void taskTDS(void *) {

  registerCurrentTaskWithWatchdog();

  for (;;) {

    float averageADC =
      readAverageADC(
        TDS_PIN,
        20
      );

    float voltage =
      averageADC *
      TDS_VREF /
      4095.0f;

    float temperature = 25.0f;

    // Get current temperature
    if (
      xSemaphoreTake(
        dataMutex,
        pdMS_TO_TICKS(20)
      ) == pdTRUE
    ) {

      temperature =
        quality.temperature;

      xSemaphoreGive(
        dataMutex
      );
    }

    // Temperature compensation
    float temperatureCoefficient =
      1.0f +
      0.02f *
      (temperature - 25.0f);

    float ec =
      (
        (
          voltage /
          temperatureCoefficient
        ) *
        TDS_EC_CALIBRATION
      ) -
      TDS_OFFSET;

    if (ec < 0) {

      ec = 0;
    }

    // TDS calculation
    float tds =
      (
        133.42f * powf(ec, 3) -
        255.86f * powf(ec, 2) +
        857.39f * ec
      ) *
      0.5f;

    if (tds < 0) {

      tds = 0;
    }

    // Update shared data
    if (
      xSemaphoreTake(
        dataMutex,
        pdMS_TO_TICKS(20)
      ) == pdTRUE
    ) {

      quality.ec = ec;

      quality.tds = tds;

      xSemaphoreGive(
        dataMutex
      );
    }

    feedWatchdog();

    vTaskDelay(
      pdMS_TO_TICKS(1000)
    );
  }
}

// =====================================================
// TEMPERATURE TASK
// =====================================================

void taskTemperature(void *) {

  registerCurrentTaskWithWatchdog();

  ds18b20.begin();

  for (;;) {

    ds18b20.requestTemperatures();

    float temperature =
      ds18b20.getTempCByIndex(0);

    if (
      temperature >= -55.0f &&
      temperature <= 125.0f
    ) {

      if (
        xSemaphoreTake(
          dataMutex,
          pdMS_TO_TICKS(20)
        ) == pdTRUE
      ) {

        quality.temperature =
          temperature;

        xSemaphoreGive(
          dataMutex
        );
      }

    } else {

      Serial.println(
        "DS18B20 read error."
      );
    }

    feedWatchdog();

    vTaskDelay(
      pdMS_TO_TICKS(1000)
    );
  }
}

// =====================================================
// UART COMMUNICATION TASK
// =====================================================

void taskCommunication(void *) {

  registerCurrentTaskWithWatchdog();

  for (;;) {

    QualityData local;

    if (
      xSemaphoreTake(
        dataMutex,
        pdMS_TO_TICKS(20)
      ) == pdTRUE
    ) {

      local = quality;

      quality.sequence++;

      local.sequence =
        quality.sequence;

      xSemaphoreGive(
        dataMutex
      );
    }

    // -------------------------------------------------
    // Send data to ESP32 #1
    // -------------------------------------------------

    MainSerial.printf(
      "Q,%lu,%.2f,%.3f,%.2f\n",
      (unsigned long)local.sequence,
      local.tds,
      local.ec,
      local.temperature
    );

    // -------------------------------------------------
    // Serial Monitor
    // -------------------------------------------------

    Serial.printf(
      "TX -> TDS: %.1f ppm | "
      "EC: %.3f mS/cm | "
      "Temp: %.1f C\n",

      local.tds,
      local.ec,
      local.temperature
    );

    feedWatchdog();

    vTaskDelay(
      pdMS_TO_TICKS(1000)
    );
  }
}

// =====================================================
// LCD TASK
// =====================================================

void taskLCD(void *) {

  registerCurrentTaskWithWatchdog();

  uint8_t screen = 0;

  for (;;) {

    QualityData local;

    // -----------------------------------------------
    // Copy shared data safely
    // -----------------------------------------------

    if (
      xSemaphoreTake(
        dataMutex,
        pdMS_TO_TICKS(20)
      ) == pdTRUE
    ) {

      local = quality;

      xSemaphoreGive(
        dataMutex
      );
    }

    // -----------------------------------------------
    // Screen 1
    // -----------------------------------------------

    if (screen == 0) {

      lcd.clear();

      lcd.setCursor(0, 0);

      lcd.print("Temp: ");

      lcd.print(
        local.temperature,
        1
      );

      lcd.print(" C");

      lcd.setCursor(0, 1);

      lcd.print("TDS: ");

      lcd.print(
        local.tds,
        1
      );

      lcd.print(" ppm");
    }

    // -----------------------------------------------
    // Screen 2
    // -----------------------------------------------

    else {

      lcd.clear();

      lcd.setCursor(0, 0);

      lcd.print("EC: ");

      lcd.print(
        local.ec,
        3
      );

      lcd.print("mS/cm");

      lcd.setCursor(0, 1);

      lcd.print("Seq: ");

      lcd.print(
        local.sequence
      );
    }

    // Switch screen
    screen++;

    if (screen >= 2) {

      screen = 0;
    }

    feedWatchdog();

    // Change LCD screen every 2 seconds
    vTaskDelay(
      pdMS_TO_TICKS(2000)
    );
  }
}

// =====================================================
// SETUP
// =====================================================

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
    "ESP32 #2 WATER QUALITY NODE"
  );

  Serial.println(
    "TDS + EC + DS18B20 + LCD"
  );

  Serial.println(
    "=========================================="
  );

  // =================================================
  // ADC
  // =================================================

  analogReadResolution(12);

  pinMode(
    TDS_PIN,
    INPUT
  );

  pinMode(
    DS18B20_PIN,
    INPUT_PULLUP
  );

  // =================================================
  // DS18B20
  // =================================================

  ds18b20.begin();

  // =================================================
  // UART2
  // =================================================

  MainSerial.begin(
    UART_BAUD,
    SERIAL_8N1,
    UART_RX,
    UART_TX
  );

  // =================================================
  // I2C LCD
  // =================================================

  Wire.begin(
    LCD_SDA,
    LCD_SCL
  );

  lcd.init();

  lcd.backlight();

  lcd.clear();

  lcd.setCursor(0, 0);

  lcd.print("Water Monitor");

  lcd.setCursor(0, 1);

  lcd.print("Starting...");

  delay(2000);

  lcd.clear();

  // =================================================
  // MUTEX
  // =================================================

  dataMutex =
    xSemaphoreCreateMutex();

  if (!dataMutex) {

    Serial.println(
      "Failed to create mutex."
    );

    lcd.clear();

    lcd.setCursor(0, 0);

    lcd.print("Mutex ERROR");

    while (true) {

      delay(1000);
    }
  }

  // =================================================
  // WATCHDOG
  // =================================================

  initWatchdog();

  // =================================================
  // TEMPERATURE TASK
  // =================================================

  xTaskCreatePinnedToCore(

    taskTemperature,

    "Temperature",

    3072,

    nullptr,

    4,

    &temperatureTaskHandle,

    1
  );

  // =================================================
  // TDS TASK
  // =================================================

  xTaskCreatePinnedToCore(

    taskTDS,

    "TDS",

    4096,

    nullptr,

    4,

    &tdsTaskHandle,

    1
  );

  // =================================================
  // UART TASK
  // =================================================

  xTaskCreatePinnedToCore(

    taskCommunication,

    "UART",

    3072,

    nullptr,

    4,

    &communicationTaskHandle,

    0
  );

  // =================================================
  // LCD TASK
  // =================================================

  xTaskCreatePinnedToCore(

    taskLCD,

    "LCD",

    3072,

    nullptr,

    2,

    &lcdTaskHandle,

    0
  );

  Serial.println(
    "ESP32 #2 tasks started."
  );

  Serial.println(
    "LCD initialized."
  );
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // All application processing
  // occurs inside FreeRTOS tasks.

  vTaskDelay(
    pdMS_TO_TICKS(1000)
  );
}