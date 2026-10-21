/*
 * Project: ESP32-S3 Touch AMOLED 1.75 - Cloud Firebase & AI-Ready Rain Station Gateway
 * Target Board: Waveshare ESP32-S3-Touch-AMOLED-1.75
 * Board Selection in Arduino IDE: "ESP32S3 Dev Module"
 *   - USB CDC On Boot: Enabled
 *   - Flash Size: 16MB (128Mb)
 *   - PSRAM: OPI PSRAM
 *   - Partition Scheme: 16MB (Huge APP or 3MB App / 9.9MB FATFS)
 * 
 * Required Libraries:
 *   - "GFX Library for Arduino" by moononournation (via Arduino Library Manager)
 *   - "ArduinoJson" (v6 or v7) by Benoit Blanchon (via Arduino Library Manager)
 * 
 * Ecosystem Role:
 *   - Central BLE Hub: Receives real-time tipping-bucket rain telemetry from Nano 33 BLE
 *   - Bidirectional Controller: Relays remote RUN / STOP / RESET commands from Firebase to Nano 33 BLE
 *   - Local HMI: 466x466 circular AMOLED display + ES8311 high-fidelity acoustic splash & chime engine
 *   - Cloud Gateway: Connects to Wi-Fi and syncs with Google Firebase Realtime Database
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <driver/i2s.h>
#include <Arduino_GFX_Library.h>

// ==========================================
// 1. WI-FI & FIREBASE CREDENTIALS
// ==========================================
// Replace with your field / lab Wi-Fi credentials
const char* WIFI_SSID     = "A25B";
const char* WIFI_PASSWORD = "infinity";

// Firebase Realtime Database URL (without https:// and trailing slash)
// Example: "my-rain-station-default-rtdb.firebaseio.com"
const char* FIREBASE_HOST = "rain-station-iot-default-rtdb.asia-southeast1.firebasedatabase.app";
// Firebase Database Secret or Auth Token (Optional for public test rules, or enter Auth token)
const char* FIREBASE_AUTH = ""; // Leave blank if test rules allow read/write

// Telemetry cloud sync intervals
const unsigned long CLOUD_SYNC_INTERVAL_MS = 3500; // 3.5 seconds periodic sync
const unsigned long CLOUD_POLL_CMD_MS      = 3000; // 3.0 seconds command check

// ==========================================
// 2. PIN DEFINITIONS (Waveshare 1.75 AMOLED)
// ==========================================
#define LCD_SDIO0   4
#define LCD_SDIO1   5
#define LCD_SDIO2   6
#define LCD_SDIO3   7
#define LCD_SCLK    38
#define LCD_CS      12
#define LCD_RESET   39
#define LCD_WIDTH   466
#define LCD_HEIGHT  466

// I2C (Touch, PMIC, Audio Codec)
#define IIC_SDA     15
#define IIC_SCL     14

// Audio I2S & Speaker Amplifier (ES8311)
#define I2S_PORT    I2S_NUM_0
#define MCLKPIN     42
#define BCLKPIN     9
#define WSPIN       45
#define DOPIN       8   // I2S Data Out
#define PA_EN_PIN   46  // Speaker amplifier enable pin
#define ES8311_ADDR 0x18

// ==========================================
// 3. DISPLAY DRIVER SETUP
// ==========================================
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3
);

Arduino_CO5300 *gfx = new Arduino_CO5300(
  bus, LCD_RESET, 0 /* rotation */, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0
);

// UI Palette (16-bit 565 RGB)
#define COLOR_BG          0x0000 // Pure AMOLED black (#000000)
#define COLOR_TEXT        0xFFFF // Crisp white
#define COLOR_SUBTITLE    0x9CD3 // Slate gray / muted cyan (#94a3b8)
#define COLOR_TIP_NUMBER  0xFD20 // Vibrant amber gold (#f59e0b)
#define COLOR_ACCENT      0x3DFE // Bright cyan / sky blue (#38bdf8)
#define COLOR_PILL_BG     0x0000 // Pitch black background
#define COLOR_PILL_BORDER 0x0459 // Cyan border (#0284c7)
#define COLOR_DEPTH       0x170A // Emerald green (#10b981)
#define COLOR_STATUS      0x8C71 // Soft slate
#define COLOR_RING_TRACK  0x18E3 // Dark track ring (#1e293b)
#define COLOR_RING_FILL   0x3DFE // Progress ring fill (#38bdf8)
#define COLOR_RING_TIP    0xFD20 // Full tip flash ring
#define COLOR_BADGE_RUN   0x1E05 // Dark green
#define COLOR_BADGE_STOP  0x7800 // Dark red

// Tipping Bucket Constants
const uint32_t DROPS_PER_TIP = 126;
const float MM_PER_TIP       = 0.20f;

// ==========================================
// 4. BLE CONFIGURATION
// ==========================================
static BLEUUID serviceUUID("19B10000-E8F2-537E-4F6C-D104768A1214");
static BLEUUID rainCharUUID("19B10001-E8F2-537E-4F6C-D104768A1214");
static BLEUUID ctrlCharUUID("19B10002-E8F2-537E-4F6C-D104768A1214");

static volatile boolean doConnect = false;
static volatile boolean connected = false;
static volatile boolean doScan    = false;
static BLERemoteCharacteristic* pRainCharacteristic = nullptr;
static BLERemoteCharacteristic* pCtrlCharacteristic = nullptr;
static BLEAdvertisedDevice* myDevice = nullptr;
static BLEClient* pClient = nullptr;

// ==========================================
// 5. APPLICATION STATE
// ==========================================
volatile uint32_t targetRainCount = 0;
uint32_t lastProcessedCount = 0;
uint32_t accumulatedTotalDrops = 0;
uint32_t lastNanoReportedCount = 0;
bool isFirstNanoPacket = true;
uint32_t displayedTips = 0;
uint32_t displayedRemaining = 0;
int lastDrawnAngle = -1;
String bleStatus = "Scanning Nano 33...";
String wifiStatus = "Connecting...";
String currentRunState = "RUNNING";
String weatherClassification = "Clear";
float calculatedRainRateMmH = 0.0f;

unsigned long lastCloudSyncTime = 0;
unsigned long lastCloudPollTime = 0;
unsigned long lastRateCalcTime = 0;
uint32_t dropsAtLastInterval = 0;
static float sineTable[256];

// Forward declarations
void initAudio();
void initES8311Codec();
void playSynthesizedRainSound();
void playMechanicalTipChime();
void drawUI();
void updateTippingDisplay(uint32_t dropCount, bool forceRedraw = false);
void updateProgressArc(uint32_t remainingDrops);
void updateStatusDisplay(String status);
void updateTopBadges();
bool connectToServer();
void syncTelemetryToFirebase();
void pollCommandsFromFirebase();
void sendBleControlCommand(byte cmd);
void calculateRainfallRate();

// ==========================================
// 6. BLE CLIENT CALLBACKS
// ==========================================
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override {
    connected = true;
    bleStatus = "Nano 33 Connected";
    BLEDevice::getScan()->stop(); // Stop scanning once connected to free 2.4GHz RF for Wi-Fi!
    Serial.println("[BLE] Connected to Nano 33 BLE Simulator! Scan stopped.");
  }

  void onDisconnect(BLEClient* pclient) override {
    connected = false;
    isFirstNanoPacket = true; // Mark next packet as reconnection baseline
    bleStatus = "BLE Disconnected";
    Serial.println("[BLE] Disconnected from Nano 33 BLE. Scanning for reconnect...");
    doScan = true;
  }
};

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  
  uint32_t incoming = 0;
  if (length >= sizeof(uint32_t)) {
    memcpy(&incoming, pData, sizeof(uint32_t));
  } else if (length == 1) {
    incoming = pData[0];
  } else {
    incoming = lastNanoReportedCount + 1;
  }

  // Handle first packet or power loss recovery
  if (isFirstNanoPacket) {
    lastNanoReportedCount = incoming;
    isFirstNanoPacket = false;
    Serial.print("[BLE SYNC] First packet received after connect: ");
    Serial.println(incoming);
  } else {
    if (incoming >= lastNanoReportedCount) {
      uint32_t delta = incoming - lastNanoReportedCount;
      accumulatedTotalDrops += delta;
      lastNanoReportedCount = incoming;
    } else {
      // Nano power loss / reset detected! Nano rebooted and started from 0
      Serial.print("[BLE RECOVERY] Nano reboot detected! Current total: ");
      Serial.print(accumulatedTotalDrops);
      Serial.print(" drops. Resuming seamlessly with new count: ");
      Serial.println(incoming);
      accumulatedTotalDrops += incoming;
      lastNanoReportedCount = incoming;
    }
  }

  targetRainCount = accumulatedTotalDrops;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (doConnect || connected) return;

    bool match = false;
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      match = true;
    } else if (advertisedDevice.haveName() && advertisedDevice.getName() == "Nano33_Rain") {
      match = true;
    }

    if (match) {
      BLEDevice::getScan()->stop();
      if (myDevice != nullptr) {
        delete myDevice;
        myDevice = nullptr;
      }
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
    }
  }
};

// ==========================================
// 7. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("===============================================================");
  Serial.println(" ESP32-S3 Touch AMOLED 1.75 - Cloud Firebase Rain Station Hub  ");
  Serial.println(" Bi-Directional BLE + Wi-Fi Gateway + ES8311 Audio Engine     ");
  Serial.println("===============================================================");

  // 1. Initialize I2C bus
  Wire.begin(IIC_SDA, IIC_SCL);

  // 2. Initialize AMOLED Display & Power
  if (!gfx->begin()) {
    Serial.println("[ERROR] Display initialization failed!");
  } else {
    Serial.println("[DISPLAY] AMOLED initialized.");
  }
  gfx->setBrightness(220);

  // Sine lookup table for rain sound DSP
  for (int i = 0; i < 256; i++) {
    sineTable[i] = sinf((2.0f * PI * i) / 256.0f);
  }

  // 3. Initialize Audio
  initAudio();

  // 4. Draw UI Frame
  drawUI();

  // 5. Connect to Wi-Fi
  Serial.print("[WIFI] Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // 6. Initialize BLE Client with Wi-Fi coexistence friendly timing
  BLEDevice::init("AMOLED_Rain_Cloud_Hub");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(320); // 200ms
  pBLEScan->setWindow(80);    // 50ms (leaves 75% radio time for Wi-Fi & TLS)
  pBLEScan->setActiveScan(true);
  pBLEScan->start(0, nullptr, false);

  Serial.println("[BLE] Coexistence scanning started for 'Nano33_Rain'...");
}

// ==========================================
// 8. MAIN LOOP
// ==========================================
void loop() {
  // 1. Handle BLE Connection State
  if (doConnect) {
    doConnect = false;
    updateStatusDisplay("Connecting BLE...");
    if (connectToServer()) {
      Serial.println("[BLE] Successfully connected to Nano 33 BLE!");
      updateStatusDisplay("Connected to Nano");
    } else {
      updateStatusDisplay("BLE Connect Failed");
      doScan = true;
    }
  }

  if (doScan) {
    doScan = false;
    updateStatusDisplay("Scanning...");
    if (myDevice != nullptr) {
      delete myDevice;
      myDevice = nullptr;
    }
    BLEScan* pScan = BLEDevice::getScan();
    pScan->clearResults();
    pScan->start(0, nullptr, false);
  }

  // 2. Wi-Fi Status Check
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiStatus != "Wi-Fi: OK") {
      wifiStatus = "Wi-Fi: OK";
      updateTopBadges();
      Serial.print("[WIFI] Connected! IP Address: ");
      Serial.println(WiFi.localIP());
    }
  } else {
    if (wifiStatus != "Wi-Fi: Disconnected") {
      wifiStatus = "Wi-Fi: Disconnected";
      updateTopBadges();
    }
  }

  // 3. Process incoming droplet / tipping events
  if (targetRainCount > lastProcessedCount) {
    uint32_t previousDrops = lastProcessedCount;
    lastProcessedCount = targetRainCount;

    bool triggeredTip = (lastProcessedCount / DROPS_PER_TIP) > (previousDrops / DROPS_PER_TIP);

    // Audio acoustic feedback
    if (triggeredTip) {
      playMechanicalTipChime();
    } else {
      playSynthesizedRainSound();
    }

    // Refresh AMOLED Screen
    updateTippingDisplay(lastProcessedCount);
  }

  // 4. Rate calculation (every 5 seconds)
  if (millis() - lastRateCalcTime >= 5000) {
    calculateRainfallRate();
    lastRateCalcTime = millis();
  }

  // 5. Cloud Firebase Telemetry Sync
  if (WiFi.status() == WL_CONNECTED && (millis() - lastCloudSyncTime >= CLOUD_SYNC_INTERVAL_MS)) {
    lastCloudSyncTime = millis();
    syncTelemetryToFirebase();
  }

  // 6. Cloud Remote Control Command Polling
  if (WiFi.status() == WL_CONNECTED && (millis() - lastCloudPollTime >= CLOUD_POLL_CMD_MS)) {
    lastCloudPollTime = millis();
    pollCommandsFromFirebase();
  }

  yield();
  delay(10);
}

// ==========================================
// 9. BLE CONNECTION LOGIC
// ==========================================
bool connectToServer() {
  if (myDevice == nullptr) return false;

  Serial.print("[BLE] Connecting to: ");
  Serial.println(myDevice->getAddress().toString().c_str());

  // Reuse client instance without deleting (deleting causes Bluedroid map panic)
  if (pClient == nullptr) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());
  }

  if (!pClient->connect(myDevice)) {
    Serial.println("[BLE] Connection attempt failed.");
    return false;
  }

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("[BLE ERROR] Failed to find Rain Service UUID!");
    pClient->disconnect();
    return false;
  }

  // 1. Rain Telemetry Characteristic
  pRainCharacteristic = pRemoteService->getCharacteristic(rainCharUUID);
  if (pRainCharacteristic == nullptr) {
    pClient->disconnect();
    return false;
  }

  if (pRainCharacteristic->canNotify()) {
    pRainCharacteristic->registerForNotify(notifyCallback);
  }

  // Initial read on connect
  if (pRainCharacteristic->canRead()) {
    String valStr = pRainCharacteristic->readValue();
    uint32_t initialCount = 0;
    if (valStr.length() >= sizeof(uint32_t)) {
      memcpy(&initialCount, valStr.c_str(), sizeof(uint32_t));
    } else if (valStr.length() == 1) {
      initialCount = (uint8_t)valStr[0];
    }
    lastNanoReportedCount = initialCount;
    isFirstNanoPacket = false;
    Serial.print("[BLE] Initial Nano count synced: ");
    Serial.println(initialCount);
  }

  // 2. Control Characteristic (Bi-Directional)
  pCtrlCharacteristic = pRemoteService->getCharacteristic(ctrlCharUUID);
  if (pCtrlCharacteristic != nullptr) {
    Serial.println("[BLE] Control Characteristic (19B10002) found and bound!");
  } else {
    Serial.println("[BLE] Note: Nano 33 is running read-only firmware; remote write disabled.");
  }

  return true;
}

void sendBleControlCommand(byte cmd) {
  if (pCtrlCharacteristic != nullptr && connected) {
    pCtrlCharacteristic->writeValue(&cmd, 1, false);
    Serial.print("[BLE CMD SENT] Sent 0x");
    Serial.print(cmd, HEX);
    Serial.println(" to Nano 33 BLE.");
  }
}

// ==========================================
// 10. METEOROLOGICAL RATE CLASSIFIER
// ==========================================
void calculateRainfallRate() {
  uint32_t deltaDrops = (lastProcessedCount >= dropsAtLastInterval) ? (lastProcessedCount - dropsAtLastInterval) : 0;
  dropsAtLastInterval = lastProcessedCount;

  // Rate in mm/h based on 5-second interval:
  // mm = deltaDrops * (0.20 / 126); mm/h = mm * (3600 / 5)
  float mmInWindow = (float)deltaDrops * (MM_PER_TIP / (float)DROPS_PER_TIP);
  calculatedRainRateMmH = mmInWindow * 720.0f; // 3600 / 5 = 720

  if (calculatedRainRateMmH <= 0.05f) {
    weatherClassification = "Clear / Dry";
  } else if (calculatedRainRateMmH < 2.5f) {
    weatherClassification = "Light Rain";
  } else if (calculatedRainRateMmH < 10.0f) {
    weatherClassification = "Moderate Rain";
  } else if (calculatedRainRateMmH < 50.0f) {
    weatherClassification = "Heavy Rain";
  } else {
    weatherClassification = "Thunderstorm";
  }
}

// ==========================================
// 11. GOOGLE FIREBASE REST INTEGRATION
// ==========================================
void syncTelemetryToFirebase() {
  if (strcmp(FIREBASE_HOST, "YOUR_PROJECT_ID-default-rtdb.firebaseio.com") == 0) {
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(3);

  HTTPClient https;
  https.setTimeout(2500);

  String url = "https://" + String(FIREBASE_HOST) + "/rain_station/telemetry.json";
  if (strlen(FIREBASE_AUTH) > 0) {
    url += "?auth=" + String(FIREBASE_AUTH);
  }

  if (https.begin(client, url)) {
    https.addHeader("Content-Type", "application/json");

    char requestBody[320];
    snprintf(requestBody, sizeof(requestBody),
      "{\"drops\":%u,\"tips\":%u,\"remaining_drops\":%u,\"depth_mm\":%.2f,\"rain_rate_mmh\":%.2f,\"weather_status\":\"%s\",\"run_state\":\"%s\",\"timestamp\":%lu}",
      lastProcessedCount,
      lastProcessedCount / DROPS_PER_TIP,
      lastProcessedCount % DROPS_PER_TIP,
      (lastProcessedCount / DROPS_PER_TIP) * MM_PER_TIP,
      calculatedRainRateMmH,
      weatherClassification.c_str(),
      currentRunState.c_str(),
      millis()
    );

    int httpCode = https.PATCH((uint8_t*)requestBody, strlen(requestBody));
    if (httpCode <= 0) {
      Serial.printf("[FIREBASE] Telemetry PATCH failed: %s\n", https.errorToString(httpCode).c_str());
    }
    https.end();
  }
  client.stop(); // Explicitly free SSL session memory from heap
}

void pollCommandsFromFirebase() {
  if (strcmp(FIREBASE_HOST, "YOUR_PROJECT_ID-default-rtdb.firebaseio.com") == 0) {
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(3);

  HTTPClient https;
  https.setTimeout(2500);

  String url = "https://" + String(FIREBASE_HOST) + "/rain_station/control.json";
  if (strlen(FIREBASE_AUTH) > 0) {
    url += "?auth=" + String(FIREBASE_AUTH);
  }

  String payload = "";
  bool needAck = false;

  if (https.begin(client, url)) {
    int httpCode = https.GET();
    if (httpCode == HTTP_CODE_OK) {
      payload = https.getString();
      
      bool isUnhandled = (payload.indexOf("\"handled\":false") != -1) || 
                         (payload.indexOf("\"handled\": false") != -1);

      if (isUnhandled) {
        needAck = true;
        Serial.print("[FIREBASE CMD] New command received: ");
        Serial.println(payload);

        if (payload.indexOf("\"STOP\"") != -1) {
          currentRunState = "STOPPED";
          sendBleControlCommand(0x00);
          updateTopBadges();
        } else if (payload.indexOf("\"RUN\"") != -1) {
          currentRunState = "RUNNING";
          sendBleControlCommand(0x01);
          updateTopBadges();
        } else if (payload.indexOf("\"RESET\"") != -1) {
          sendBleControlCommand(0x02);
          accumulatedTotalDrops = 0;
          lastNanoReportedCount = 0;
          isFirstNanoPacket = true;
          targetRainCount = 0;
          lastProcessedCount = 0;
          updateTippingDisplay(0, true);
        } else if (payload.indexOf("\"TIP_NOW\"") != -1) {
          sendBleControlCommand(0x03);
        }
      }
    }
    https.end();
  }
  client.stop();

  // If command was processed, send acknowledgment in a clean, separate connection
  if (needAck) {
    if (https.begin(client, url)) {
      https.addHeader("Content-Type", "application/json");
      https.PATCH((uint8_t*)"{\"handled\":true}", 16);
      https.end();
    }
    client.stop();
  }
}

// ==========================================
// 12. UI RENDERING ENGINE (Round AMOLED)
// ==========================================
void drawUI() {
  gfx->fillScreen(COLOR_BG);

  // Outer Circular Track Ring (Radius = 214-218, centered at 233, 233)
  for (int r = 214; r <= 218; r++) {
    gfx->drawCircle(233, 233, r, COLOR_RING_TRACK);
  }

  // Header Subtitle: "RAIN STATION HUB"
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_SUBTITLE, COLOR_BG);
  int headerX = 138;
  int headerY = 52;
  gfx->setCursor(headerX, headerY);
  gfx->print("RAIN STATION HUB");

  // Title: "TIPS ACCUMULATED"
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_TEXT, COLOR_BG);
  int tipTitleX = 135;
  int tipTitleY = 112;
  gfx->setCursor(tipTitleX, tipTitleY);
  gfx->print("TIPS ACCUMULATED");

  updateTopBadges();
  updateTippingDisplay(0, true);
  updateStatusDisplay(bleStatus);
}

void updateTopBadges() {
  gfx->fillRect(110, 80, 246, 26, COLOR_BG);
  gfx->setTextSize(2);

  // Left side: Wi-Fi status
  if (wifiStatus == "Wi-Fi: OK") {
    gfx->setTextColor(COLOR_DEPTH, COLOR_BG); // Green
    gfx->setCursor(120, 84);
    gfx->print("WF:OK");
  } else {
    gfx->setTextColor(0xF800, COLOR_BG); // Red
    gfx->setCursor(120, 84);
    gfx->print("WF:--");
  }

  // Right side: Station Run/Stop state
  if (currentRunState == "STOPPED") {
    gfx->setTextColor(0xF800, COLOR_BG); // Red
    gfx->setCursor(220, 84);
    gfx->print("CMD:STOP");
  } else {
    gfx->setTextColor(COLOR_ACCENT, COLOR_BG); // Cyan
    gfx->setCursor(220, 84);
    gfx->print("CMD:RUN");
  }
}

void updateTippingDisplay(uint32_t dropCount, bool forceRedraw) {
  uint32_t tips = dropCount / DROPS_PER_TIP;
  uint32_t rem  = dropCount % DROPS_PER_TIP;
  float depthMm = tips * MM_PER_TIP;
  bool tipsChanged = (tips != displayedTips) || forceRedraw;

  // 1. Big Tip Counter (Large Amber Font, TextSize 7)
  if (tipsChanged) {
    gfx->fillRect(130, 142, 206, 75, COLOR_BG);
    gfx->setTextSize(7);
    gfx->setTextColor(COLOR_TIP_NUMBER, COLOR_BG);

    char tipBuffer[10];
    if (tips < 10) {
      snprintf(tipBuffer, sizeof(tipBuffer), "0%lu", (unsigned long)tips);
    } else {
      snprintf(tipBuffer, sizeof(tipBuffer), "%lu", (unsigned long)tips);
    }

    int tipX = (tips < 100) ? 190 : 165;
    gfx->setCursor(tipX, 150);
    gfx->print(tipBuffer);
  }

  // 2. Sub-Counter Pill ("DROPS: rem / 126")
  if (rem != displayedRemaining || forceRedraw) {
    displayedRemaining = rem;

    gfx->fillRoundRect(110, 235, 246, 44, 22, COLOR_PILL_BG);
    gfx->drawRoundRect(110, 235, 246, 44, 22, COLOR_PILL_BORDER);
    gfx->drawRoundRect(111, 236, 244, 42, 21, COLOR_PILL_BORDER);

    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_ACCENT, COLOR_PILL_BG);

    char dropBuffer[30];
    snprintf(dropBuffer, sizeof(dropBuffer), "DROPS: %lu / 126", (unsigned long)rem);
    int strLen = strlen(dropBuffer);
    int pillTextX = 233 - (strLen * 6);
    gfx->setCursor(pillTextX, 250);
    gfx->print(dropBuffer);

    updateProgressArc(rem);
  }

  // 3. Precipitation Depth ("DEPTH: 0.20 mm")
  if (tipsChanged) {
    gfx->fillRect(120, 295, 226, 32, COLOR_BG);
    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_DEPTH, COLOR_BG);

    char depthBuffer[30];
    snprintf(depthBuffer, sizeof(depthBuffer), "DEPTH: %.2f mm", depthMm);
    int strLen = strlen(depthBuffer);
    int depthX = 233 - (strLen * 6);
    gfx->setCursor(depthX, 302);
    gfx->print(depthBuffer);
  }

  // 4. Weather Classification Status Display
  if (tipsChanged || forceRedraw) {
    gfx->fillRect(100, 335, 266, 25, COLOR_BG);
    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_SUBTITLE, COLOR_BG);
    int strLen = weatherClassification.length();
    int wxX = 233 - (strLen * 6);
    if (wxX < 30) wxX = 30;
    gfx->setCursor(wxX, 340);
    gfx->print(weatherClassification);
  }

  displayedTips = tips;
}

void updateProgressArc(uint32_t remainingDrops) {
  int targetAngle = (int)((remainingDrops * 360.0f) / DROPS_PER_TIP);
  if (targetAngle > 360) targetAngle = 360;

  const int cx = 233;
  const int cy = 233;
  const int rMin = 214;
  const int rMax = 218;

  if (lastDrawnAngle < 0 || remainingDrops == 0) {
    for (int r = rMin; r <= rMax; r++) {
      gfx->drawCircle(cx, cy, r, COLOR_RING_TRACK);
    }
    lastDrawnAngle = 0;
  }

  if (targetAngle > lastDrawnAngle) {
    for (int a = lastDrawnAngle; a <= targetAngle; a += 2) {
      float rad = (a - 90) * (PI / 180.0f);
      float cosA = cosf(rad);
      float sinA = sinf(rad);
      for (int r = rMin; r <= rMax; r++) {
        gfx->drawPixel((int)(cx + r * cosA), (int)(cy + r * sinA), COLOR_RING_FILL);
      }
    }
    lastDrawnAngle = targetAngle;
  }
}

void updateStatusDisplay(String status) {
  bleStatus = status;
  gfx->fillRect(80, 370, 306, 30, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_STATUS, COLOR_BG);

  int strLen = status.length();
  int statusX = 233 - (strLen * 6);
  if (statusX < 30) statusX = 30;
  gfx->setCursor(statusX, 376);
  gfx->print(bleStatus);
}

// ==========================================
// 13. AUDIO SYNTHESIS & I2S ENGINE
// ==========================================
void initAudio() {
  pinMode(PA_EN_PIN, OUTPUT);
  digitalWrite(PA_EN_PIN, HIGH); // Enable speaker amp

  initES8311Codec();

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = BCLKPIN,
    .ws_io_num = WSPIN,
    .data_out_num = DOPIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_set_clk(I2S_PORT, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

void initES8311Codec() {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(0x00);
  Wire.write(0x80); // Reset
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(0x32);
  Wire.write(0xBF); // High output gain
  Wire.endTransmission();
}

void playSynthesizedRainSound() {
  const int sampleRate = 16000;
  const int durationMs = 110;
  const int numSamples = (sampleRate * durationMs) / 1000;
  static int16_t buffer[256 * 2]; // static: ZERO stack allocation!

  float startFreq = random(1400, 2100);
  float endFreq   = random(550, 850);
  uint32_t phaseAcc = 0;

  int sampleIndex = 0;
  while (sampleIndex < numSamples) {
    int chunkSize = min(256, numSamples - sampleIndex);

    for (int i = 0; i < chunkSize; i++) {
      int cur = sampleIndex + i;
      float progress = (float)cur / (float)numSamples;

      float currentFreq = startFreq + (endFreq - startFreq) * progress;
      float envelope = (1.0f - progress) * (1.0f - progress);

      uint8_t tableIdx = (phaseAcc >> 8) & 0xFF;
      float tone = sineTable[tableIdx];
      float noise = ((float)random(-80, 80) / 100.0f) * (1.0f - progress);
      float sampleVal = (tone * 0.8f + noise * 0.2f) * envelope * 20000.0f;

      uint32_t phaseInc = (uint32_t)((currentFreq * 256.0f * 256.0f) / sampleRate);
      phaseAcc += phaseInc;

      int16_t pcm = (int16_t)sampleVal;
      buffer[i * 2]     = pcm; // Left
      buffer[i * 2 + 1] = pcm; // Right
    }

    size_t bytesWritten = 0;
    i2s_write(I2S_PORT, buffer, chunkSize * 4, &bytesWritten, portMAX_DELAY);
    sampleIndex += chunkSize;
  }
}

void playMechanicalTipChime() {
  const int sampleRate = 16000;
  const int durationMs = 320; // 320ms resonant chime
  const int numSamples = (sampleRate * durationMs) / 1000;
  static int16_t buffer[256 * 2]; // static: ZERO stack allocation!

  // Dual bell frequencies (Fundamental 880 Hz + Fifth 1320 Hz)
  const float freq1 = 880.0f;
  const float freq2 = 1320.0f;
  uint32_t phaseAcc1 = 0;
  uint32_t phaseAcc2 = 0;

  uint32_t phaseInc1 = (uint32_t)((freq1 * 256.0f * 256.0f) / sampleRate);
  uint32_t phaseInc2 = (uint32_t)((freq2 * 256.0f * 256.0f) / sampleRate);

  int sampleIndex = 0;
  while (sampleIndex < numSamples) {
    int chunkSize = min(256, numSamples - sampleIndex);

    for (int i = 0; i < chunkSize; i++) {
      int cur = sampleIndex + i;
      float progress = (float)cur / (float)numSamples;

      // Exponential bell decay curve
      float envelope = expf(-4.5f * progress);

      uint8_t idx1 = (phaseAcc1 >> 8) & 0xFF;
      uint8_t idx2 = (phaseAcc2 >> 8) & 0xFF;
      float tone = (sineTable[idx1] * 0.65f) + (sineTable[idx2] * 0.35f);

      // Metallic strike transient in the first 15ms
      float strike = 0.0f;
      if (cur < 240) {
        float strikeProgress = (float)cur / 240.0f;
        strike = ((float)random(-100, 100) / 100.0f) * (1.0f - strikeProgress) * 0.3f;
      }

      float sampleVal = (tone + strike) * envelope * 24000.0f;

      phaseAcc1 += phaseInc1;
      phaseAcc2 += phaseInc2;

      int16_t pcm = (int16_t)sampleVal;
      buffer[i * 2]     = pcm; // Left
      buffer[i * 2 + 1] = pcm; // Right
    }

    size_t bytesWritten = 0;
    i2s_write(I2S_PORT, buffer, chunkSize * 4, &bytesWritten, portMAX_DELAY);
    sampleIndex += chunkSize;
  }
}
