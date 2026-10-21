/*
 * Project: Bi-Directional BLE Rain Simulator Hub
 * Target Board: Arduino Nano 33 BLE (Nordic nRF52840)
 * Environment: Arduino IDE (Arduino Mbed OS Nano Boards)
 * 
 * Part of: Firebase Cloud & AI-Ready Rain Monitoring System
 * 
 * Features:
 * - Multi-Central BLE Peripheral (Concurrent broadcast to ESP32-S3 AMOLED + Web Cloud Gateway)
 * - Tipping-Bucket Specification: 1 Mechanical Tip = 126 Water Drops = 0.20 mm Precipitation Depth
 * - Bi-Directional Control via BLE:
 *     * Characteristic 19B10001: Rain Drops / Tips Counter (Notify | Read)
 *     * Characteristic 19B10002: Remote Station Control (Write | Read)
 *         - 0x00: STOP simulation (Pause auto rain)
 *         - 0x01: RUN simulation (Resume auto rain)
 *         - 0x02: RESET counter to 0
 *         - 0x03: FAST-FORWARD to next tip event
 *         - 0x04: Single manual droplet trigger
 * - Serial Monitor commands: 'r' (drop), 't' (fast tip), 'a' (toggle auto), 'c' (reset), 's' (stats)
 */

#include <ArduinoBLE.h>

// BLE UUIDs for Rain Service and Characteristics
const char* BLE_SERVICE_UUID        = "19B10000-E8F2-537E-4F6C-D104768A1214";
const char* BLE_RAIN_CHAR_UUID      = "19B10001-E8F2-537E-4F6C-D104768A1214";
const char* BLE_CONTROL_CHAR_UUID   = "19B10002-E8F2-537E-4F6C-D104768A1214";

// Service & Characteristic definitions
BLEService rainService(BLE_SERVICE_UUID);
BLEUnsignedIntCharacteristic rainChar(BLE_RAIN_CHAR_UUID, BLERead | BLENotify);
BLEByteCharacteristic controlChar(BLE_CONTROL_CHAR_UUID, BLERead | BLEWrite | BLEWriteWithoutResponse);

// Tipping-Bucket Meteorological Constants
const uint32_t DROPS_PER_TIP = 126;      // 1 Tip = 126 water drops
const float MM_PER_TIP       = 0.20f;    // 0.20 mm rainfall depth per tip

// Application state
uint32_t rainCount = 0;                  // Total cumulative drops
unsigned long lastRainTime = 0;
unsigned long rainInterval = 2200;       // Simulated droplet interval (ms)
bool autoRain = true;                    // Run / Stop simulation state
int connectedCentralCount = 0;           // Number of active connected centrals
volatile bool requestReadvertise = false;

// LED pin for visual indicator
const int LED_PIN = LED_BUILTIN;
unsigned long ledPulseOffTime = 0;
int ledBlinkStage = 0;

// Forward declarations
void triggerRainEvent(bool isConnected);
void triggerTipFastForward();
void printStats();
void handleSerialCommands();
void updateLedPulse(bool isConnected);
unsigned long getNextRainInterval();

// ==========================================
// BLE EVENT HANDLERS & CALLBACKS
// ==========================================
void bleConnectHandler(BLEDevice central) {
  connectedCentralCount++;
  Serial.print("[BLE] Central connected: ");
  Serial.print(central.address());
  Serial.print(" (Total connected: ");
  Serial.print(connectedCentralCount);
  Serial.println(")");

  digitalWrite(LED_PIN, HIGH);
  requestReadvertise = true;
}

void bleDisconnectHandler(BLEDevice central) {
  if (connectedCentralCount > 0) {
    connectedCentralCount--;
  }
  Serial.print("[BLE] Central disconnected: ");
  Serial.print(central.address());
  Serial.print(" (Remaining connected: ");
  Serial.print(connectedCentralCount);
  Serial.println(")");

  if (connectedCentralCount == 0) {
    digitalWrite(LED_PIN, LOW);
  }
  requestReadvertise = true;
}

// Characteristic Write Callback (Remote Command from ESP32-S3 / Cloud)
void controlCharWritten(BLEDevice central, BLECharacteristic characteristic) {
  byte cmd = controlChar.value();
  Serial.print("[BLE COMMAND RECEIVED] Value: 0x");
  Serial.print(cmd, HEX);
  Serial.print(" from: ");
  Serial.println(central.address());

  switch (cmd) {
    case 0x00: // STOP
      autoRain = false;
      Serial.println(">>> REMOTE COMMAND: SIMULATION PAUSED (STOP) <<<");
      break;

    case 0x01: // RUN
      autoRain = true;
      lastRainTime = millis();
      Serial.println(">>> REMOTE COMMAND: SIMULATION RESUMED (RUN) <<<");
      break;

    case 0x02: // RESET
      rainCount = 0;
      rainChar.writeValue(rainCount);
      Serial.println(">>> REMOTE COMMAND: RAIN COUNTER RESET TO 0 <<<");
      printStats();
      break;

    case 0x03: // FAST-FORWARD TIP
      Serial.println(">>> REMOTE COMMAND: FAST-FORWARD TO NEXT TIP <<<");
      triggerTipFastForward();
      break;

    case 0x04: // SINGLE DROPLET
      Serial.println(">>> REMOTE COMMAND: SINGLE DROPLET TRIGGERED <<<");
      triggerRainEvent(connectedCentralCount > 0 || BLE.connected());
      break;

    default:
      Serial.print("[WARN] Unknown BLE control command: ");
      Serial.println(cmd);
      break;
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 2500));

  Serial.println("===============================================================");
  Serial.println(" Arduino Nano 33 BLE - Bi-Directional Rain Simulator Node     ");
  Serial.println(" Firebase & AI-Ready Ecosystem | 1 Tip = 126 Drops = 0.20 mm   ");
  Serial.println(" Remote Commands: 0x00 (STOP), 0x01 (RUN), 0x02 (RESET)        ");
  Serial.println("===============================================================");

  if (!BLE.begin()) {
    Serial.println("[ERROR] Failed to initialize BLE hardware!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }

  BLE.setLocalName("Nano33_Rain");
  BLE.setDeviceName("Nano33_Rain");
  BLE.setAdvertisedService(rainService);

  BLE.setConnectionInterval(12, 28); // 15ms - 35ms for responsive commands
  BLE.setSupervisionTimeout(600);    // 6.0s

  BLE.setEventHandler(BLEConnected, bleConnectHandler);
  BLE.setEventHandler(BLEDisconnected, bleDisconnectHandler);

  // Attach control characteristic write listener
  controlChar.setEventHandler(BLEWritten, controlCharWritten);

  // Add characteristics
  rainService.addCharacteristic(rainChar);
  rainService.addCharacteristic(controlChar);
  BLE.addService(rainService);

  // Initial values
  rainChar.writeValue(rainCount);
  controlChar.writeValue((byte)0x01); // Default RUN

  BLE.advertise();
  Serial.println("[BLE] Advertising started as 'Nano33_Rain'...");
  Serial.println("[BLE] Control Characteristic (19B10002) is READY for remote RUN/STOP!");
  printStats();
  Serial.println("---------------------------------------------------------------");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  BLE.poll();

  bool isConnected = (connectedCentralCount > 0) || BLE.connected();

  if (requestReadvertise) {
    requestReadvertise = false;
    if (connectedCentralCount < 2) {
      BLE.advertise();
      Serial.println("[BLE] Advertising resumed for additional Central...");
    }
  }

  // Handle simulated rainfall if running
  if (autoRain && isConnected && (millis() - lastRainTime >= rainInterval)) {
    lastRainTime = millis();
    triggerRainEvent(isConnected);
    rainInterval = getNextRainInterval();
  }

  handleSerialCommands();
  updateLedPulse(isConnected);

  delay(5);
}

// ==========================================
// RAIN & TIPPING ACCUMULATOR
// ==========================================
void triggerRainEvent(bool isConnected) {
  rainCount++;

  uint32_t currentTips = rainCount / DROPS_PER_TIP;
  uint32_t remainingDrops = rainCount % DROPS_PER_TIP;
  float totalDepthMm = currentTips * MM_PER_TIP;

  bool isTipEvent = (remainingDrops == 0 && rainCount > 0);

  if (isTipEvent) {
    Serial.println();
    Serial.println("***************************************************************");
    Serial.print("🔔 [TIPPING EVENT #");
    Serial.print(currentTips);
    Serial.print("] Reached 126 Drops! Total Depth: ");
    Serial.print(totalDepthMm, 2);
    Serial.println(" mm");
    Serial.println("***************************************************************");
  } else {
    Serial.print("[DROP #");
    Serial.print(rainCount);
    Serial.print("] Progress: ");
    Serial.print(remainingDrops);
    Serial.print("/126 drops | Tips: ");
    Serial.print(currentTips);
    Serial.print(" | ");
    Serial.print(totalDepthMm, 2);
    Serial.println(" mm");
  }

  if (isConnected) {
    rainChar.writeValue(rainCount);
  }

  if (isTipEvent) {
    digitalWrite(LED_PIN, LOW);
    ledBlinkStage = 1;
    ledPulseOffTime = millis() + 60;
  } else {
    digitalWrite(LED_PIN, isConnected ? LOW : HIGH);
    ledBlinkStage = 0;
    ledPulseOffTime = millis() + 80;
  }
}

void triggerTipFastForward() {
  uint32_t remainingDrops = rainCount % DROPS_PER_TIP;
  uint32_t dropsNeeded = (remainingDrops == 0) ? DROPS_PER_TIP : (DROPS_PER_TIP - remainingDrops);
  Serial.print("[CMD] Fast-forwarding ");
  Serial.print(dropsNeeded);
  Serial.println(" drops to trigger next mechanical tip event...");

  for (uint32_t i = 0; i < dropsNeeded; i++) {
    triggerRainEvent(connectedCentralCount > 0 || BLE.connected());
    BLE.poll();
    delay(15);
  }
}

unsigned long getNextRainInterval() {
  int dice = random(0, 100);
  if (dice < 50) {
    return random(800, 1600);
  } else if (dice < 85) {
    return random(1600, 2800);
  } else {
    return random(2800, 4200);
  }
}

void updateLedPulse(bool isConnected) {
  if (ledPulseOffTime > 0 && millis() >= ledPulseOffTime) {
    if (ledBlinkStage == 1) {
      digitalWrite(LED_PIN, HIGH);
      ledBlinkStage = 2;
      ledPulseOffTime = millis() + 80;
    } else {
      ledPulseOffTime = 0;
      ledBlinkStage = 0;
      digitalWrite(LED_PIN, isConnected ? HIGH : LOW);
    }
  }
}

void printStats() {
  uint32_t currentTips = rainCount / DROPS_PER_TIP;
  uint32_t remainingDrops = rainCount % DROPS_PER_TIP;
  float totalDepthMm = currentTips * MM_PER_TIP;

  Serial.println("---------------------------------------------------------------");
  Serial.println(" CURRENT METRICS & STATE:");
  Serial.print("  • Simulation Status:       ");
  Serial.println(autoRain ? "RUNNING (Active)" : "STOPPED (Paused)");
  Serial.print("  • Cumulative Drops:        ");
  Serial.println(rainCount);
  Serial.print("  • Total Bucket Tips:       ");
  Serial.println(currentTips);
  Serial.print("  • Progress to Next Tip:    ");
  Serial.print(remainingDrops);
  Serial.print(" / 126 drops (");
  Serial.print((remainingDrops * 100.0f) / DROPS_PER_TIP, 1);
  Serial.println("%)");
  Serial.print("  • Precipitation Depth:     ");
  Serial.print(totalDepthMm, 2);
  Serial.println(" mm");
  Serial.println(" Commands: [r] Drop, [t] Fast Tip, [a] Toggle Run/Stop, [c] Reset, [s] Stats");
  Serial.println("---------------------------------------------------------------");
}

void handleSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      triggerRainEvent(connectedCentralCount > 0 || BLE.connected());
    } else if (cmd == 't' || cmd == 'T') {
      triggerTipFastForward();
    } else if (cmd == 'a' || cmd == 'A') {
      autoRain = !autoRain;
      controlChar.writeValue((byte)(autoRain ? 0x01 : 0x00));
      Serial.print("[CMD] Rain Simulation: ");
      Serial.println(autoRain ? "RUNNING" : "STOPPED");
    } else if (cmd == 'c' || cmd == 'C') {
      rainCount = 0;
      rainChar.writeValue(rainCount);
      Serial.println("[CMD] Rain Counter Reset.");
      printStats();
    } else if (cmd == 's' || cmd == 'S') {
      printStats();
    }
  }
}
