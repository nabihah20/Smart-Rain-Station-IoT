/*
 * ==============================================================================
 * ESP32-S3 Touch AMOLED 1.75 - Onboard Microphone Droplet & Tip Calibrator
 * ==============================================================================
 * Project: Smart Rain Station IoT (Workstream 05: Real-Time Droplet Counting)
 * Target Hardware: Waveshare ESP32-S3 Touch AMOLED 1.75
 * Codec / ADC: Onboard ES7210 4-Channel Audio ADC + Dual MEMS Microphones
 * Display: 466x466 Round CO5300 QSPI AMOLED (GFX_Library_for_Arduino)
 *
 * PURPOSE:
 * 1. Capture real physical water drop acoustics via the onboard MEMS mic array.
 * 2. Graph the real-time acoustic waveform in Arduino Serial Plotter (Tools > Serial Plotter).
 * 3. Provide an interactive on-screen VU meter and real droplet counter on the AMOLED display.
 * 4. Allow dynamic threshold adjustment (+ / - keys in Serial Monitor) to tune sensitivity.
 *
 * HARDWARE PIN DEFINITIONS:
 * - I2C Bus:        SDA = GPIO 15, SCL = GPIO 14 (Shared with PMIC, RTC, IMU, Touch)
 * - I2S MCLK:       GPIO 42 (Master Clock output to ES7210 / ES8311)
 * - I2S BCLK:       GPIO 9  (Bit Clock)
 * - I2S WS / LRCK:  GPIO 45 (Word Select / Frame Clock)
 * - I2S DOUT:       GPIO 8  (Data out to ES8311 speaker)
 * - I2S DIN:        GPIO 10 (Data IN from ES7210 dual microphones)
 * - PA_EN:          GPIO 46 (Speaker amplifier enable)
 * - ES7210 Address: 0x40 (I2C 7-bit slave address)
 * ==============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <Arduino_GFX_Library.h>
#include <math.h>

// ==========================================
// 1. PIN DEFINITIONS
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

// I2C Control Bus (Shared)
#define IIC_SDA     15
#define IIC_SCL     14

// Audio I2S Bus
#define I2S_PORT    I2S_NUM_0
#define MCLKPIN     42
#define BCLKPIN     9
#define WSPIN       45
#define DOPIN       8
#define DIPIN       10  // Microphone Audio IN from ES7210
#define PA_EN_PIN   46  // Speaker Amp Enable
#define ES7210_ADDR 0x40
#define ES8311_ADDR 0x18
#define AXP2101_ADDR 0x34

// ==========================================
// 2. DISPLAY DRIVER SETUP
// ==========================================
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3
);

Arduino_CO5300 *gfx = new Arduino_CO5300(
  bus, LCD_RESET, 0 /* rotation */, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0
);

// UI Colors (RGB565)
#define COLOR_BG            0x0000 // Pure AMOLED black
#define COLOR_TEXT_WHITE    0xFFFF
#define COLOR_TEXT_MUTED    0x9CD3 // Slate gray
#define COLOR_CYAN          0x3DFE // Electric cyan
#define COLOR_AMBER         0xFD20 // Warning / amber
#define COLOR_EMERALD       0x170A // Green / success
#define COLOR_CRIMSON       0xF800 // Red / peak
#define COLOR_TRACK         0x18E3 // Dark gray track
#define COLOR_BORDER        0x0459 // Cyan border

// ==========================================
// 3. AUDIO BUFFER & DETECTION CONSTANTS
// ==========================================
#define SAMPLE_RATE         16000
#define SAMPLES_PER_READ    256

static int16_t audioBuffer[SAMPLES_PER_READ * 2]; // 16-bit stereo interleaved

// Detection State Variables
int32_t currentThreshold   = 3500;  // Default acoustic trigger threshold (tune with +/-)
int32_t peakAmplitude      = 0;
float   rmsEnergy          = 0.0f;
float   noiseFloor         = 300.0f;
uint32_t physicalDropCount = 0;
unsigned long lastDropTime = 0;
const unsigned long DEBOUNCE_MS = 80; // Minimum delay between valid drops

// UI Refresh timers
unsigned long lastUiUpdate  = 0;
const unsigned long UI_INTERVAL_MS = 40; // ~25 FPS UI refresh
bool dropTriggerVisualFlash = false;
unsigned long flashExpireTime = 0;

// ==========================================
// 4. I2C REGISTER HELPERS
// ==========================================
bool writeI2CReg(uint8_t devAddr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

uint8_t readI2CReg(uint8_t devAddr, uint8_t reg) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0xFF;
  Wire.requestFrom((uint8_t)devAddr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// ==========================================
// 5. POWER MANAGEMENT (AXP2101) & ES7210 INIT
// ==========================================
void initPowerAndI2C() {
  Wire.begin(IIC_SDA, IIC_SCL, 400000);
  delay(50);
  Serial.println("\n[I2C] Scanning bus...");
  
  bool foundES7210 = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  -> Found I2C Device at 0x%02X", addr);
      if (addr == ES7210_ADDR)  { Serial.print(" (ES7210 Mic ADC)"); foundES7210 = true; }
      if (addr == ES8311_ADDR)  { Serial.print(" (ES8311 Audio DAC)"); }
      if (addr == AXP2101_ADDR) { Serial.print(" (AXP2101 PMIC)"); }
      Serial.println();
    }
  }

  // Ensure Audio Power Rails are turned on via AXP2101 if present
  // ALDO1-4 & DLDO power the analog circuitry on Waveshare boards
  writeI2CReg(AXP2101_ADDR, 0x90, 0xFF); // Enable ALDOs
  writeI2CReg(AXP2101_ADDR, 0x91, 0xFF); // Enable BLDOs / DLDOs
  delay(20);

  if (!foundES7210) {
    Serial.println("[WARN] ES7210 not immediately responding. Clocking I2S MCLK first...");
  }
}

// Configure ES7210 4-Channel Audio ADC Registers
void initES7210() {
  Serial.println("[ES7210] Initializing microphone ADC registers...");
  
  // Software Reset
  writeI2CReg(ES7210_ADDR, 0x00, 0xFF);
  delay(10);
  writeI2CReg(ES7210_ADDR, 0x00, 0x32); // Normal mode, MCLK enabled
  
  // Clock Management & Sample Rate Settings (16kHz standard)
  writeI2CReg(ES7210_ADDR, 0x01, 0x3F); // Clock gating enable
  writeI2CReg(ES7210_ADDR, 0x02, 0x00); // Analog ADC enable & normal power
  writeI2CReg(ES7210_ADDR, 0x03, 0x20); // 16-bit I2S format, Standard Philips
  writeI2CReg(ES7210_ADDR, 0x04, 0x01); // Slot 0 enabled
  writeI2CReg(ES7210_ADDR, 0x05, 0x00); // Slave mode (ESP32 is Master)
  writeI2CReg(ES7210_ADDR, 0x06, 0x00);
  writeI2CReg(ES7210_ADDR, 0x07, 0x00);
  writeI2CReg(ES7210_ADDR, 0x08, 0x00);

  // Digital Volume / Output
  writeI2CReg(ES7210_ADDR, 0x11, 0x60); // 16-bit data depth
  writeI2CReg(ES7210_ADDR, 0x12, 0x02); // SDOUT1 enable

  // Analog Microphone Bias & Gain Configuration
  // Boost analog gain so subtle water drops generate sharp spikes
  writeI2CReg(ES7210_ADDR, 0x40, 0x42); // Power up bias circuit
  writeI2CReg(ES7210_ADDR, 0x41, 0x70); // Mic Bias voltage = 2.8V
  writeI2CReg(ES7210_ADDR, 0x42, 0x70); 
  writeI2CReg(ES7210_ADDR, 0x43, 0x1E); // MIC1/2 PGA Gain: +24 dB (high sensitivity)
  writeI2CReg(ES7210_ADDR, 0x44, 0x1E); // MIC3/4 PGA Gain: +24 dB
  writeI2CReg(ES7210_ADDR, 0x47, 0x00); // Channel 1/2 un-mute
  writeI2CReg(ES7210_ADDR, 0x48, 0x00); // Channel 3/4 un-mute

  Serial.println("[ES7210] Microphone ADC configured successfully!");
}

// ==========================================
// 6. I2S PERIPHERAL INITIALIZATION
// ==========================================
void initI2S() {
  Serial.println("[I2S] Configuring ESP32-S3 I2S Master Audio Bus...");

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = SAMPLES_PER_READ,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .mck_io_num = MCLKPIN,   // GPIO 42 (MCLK)
    .bck_io_num = BCLKPIN,   // GPIO 9  (BCLK)
    .ws_io_num = WSPIN,      // GPIO 45 (LRCK/WS)
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = DIPIN     // GPIO 10 (MIC DATA IN)
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("[I2S] Driver install failed: %d\n", err);
  }
  
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("[I2S] Set pin failed: %d\n", err);
  }

  i2s_zero_dma_buffer(I2S_PORT);
  Serial.println("[I2S] Microphone input active on GPIO 10.");
}

// ==========================================
// 7. UI RENDERING (AMOLED ROUND SCREEN)
// ==========================================
void renderStaticUI() {
  gfx->fillScreen(COLOR_BG);
  
  // Outer perimeter aesthetic ring
  gfx->drawCircle(233, 233, 230, COLOR_BORDER);
  gfx->drawCircle(233, 233, 229, COLOR_TRACK);

  // Header Title
  gfx->setTextColor(COLOR_CYAN);
  gfx->setTextSize(2);
  gfx->setCursor(95, 45);
  gfx->println("ACOUSTIC CALIBRATOR");

  gfx->setTextColor(COLOR_TEXT_MUTED);
  gfx->setTextSize(1);
  gfx->setCursor(140, 70);
  gfx->println("ES7210 DUAL-MIC ARRAY");

  // Center Drop Counter Box
  gfx->drawRoundRect(83, 115, 300, 110, 16, COLOR_TRACK);
  gfx->setTextColor(COLOR_TEXT_MUTED);
  gfx->setTextSize(2);
  gfx->setCursor(120, 130);
  gfx->println("REAL DROPS DETECTED");

  // VU Meter Track Outline
  gfx->drawRoundRect(63, 250, 340, 36, 10, COLOR_TRACK);
  
  // Sub-labels
  gfx->setTextColor(COLOR_TEXT_MUTED);
  gfx->setTextSize(2);
  gfx->setCursor(75, 310);
  gfx->print("PEAK:");
  gfx->setCursor(245, 310);
  gfx->print("NOISE:");

  gfx->setCursor(75, 345);
  gfx->print("THRESHOLD:");

  gfx->setCursor(95, 405);
  gfx->setTextSize(1);
  gfx->setTextColor(COLOR_CYAN);
  gfx->println("[Serial: '+' or '-' to adjust threshold | 'r' to reset]");
}

void updateDynamicUI() {
  // 1. Update Real Drops Count
  gfx->fillRect(90, 155, 285, 60, COLOR_BG);
  gfx->setTextColor(COLOR_AMBER);
  gfx->setTextSize(6);
  gfx->setCursor(180, 160);
  gfx->printf("%lu", physicalDropCount);

  // 2. Update VU Meter Bar
  // Map peak amplitude (0 - 20000) to bar width (0 - 334 pixels)
  int barWidth = map(constrain(peakAmplitude, 0, 20000), 0, 20000, 0, 334);
  gfx->fillRect(66, 253, 334, 30, COLOR_BG); // Clear previous bar
  
  uint16_t barColor = COLOR_EMERALD;
  if (peakAmplitude > (currentThreshold * 0.7)) barColor = COLOR_AMBER;
  if (peakAmplitude >= currentThreshold)       barColor = COLOR_CRIMSON;
  
  if (barWidth > 0) {
    gfx->fillRoundRect(66, 253, barWidth, 30, 8, barColor);
  }
  
  // Draw threshold marker tick on VU meter
  int thresholdX = map(constrain(currentThreshold, 0, 20000), 0, 20000, 0, 334) + 66;
  gfx->drawFastVLine(thresholdX, 248, 40, COLOR_WHITE);

  // 3. Update Numeric Readings
  // Clear metric numbers region
  gfx->fillRect(145, 310, 90, 20, COLOR_BG);
  gfx->fillRect(320, 310, 90, 20, COLOR_BG);
  gfx->fillRect(215, 345, 120, 20, COLOR_BG);

  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_TEXT_WHITE);
  gfx->setCursor(145, 310);
  gfx->printf("%ld", peakAmplitude);

  gfx->setCursor(320, 310);
  gfx->printf("%d", (int)noiseFloor);

  gfx->setTextColor(COLOR_CYAN);
  gfx->setCursor(215, 345);
  gfx->printf("%ld", currentThreshold);

  // 4. Trigger Banner Indicator
  if (dropTriggerVisualFlash) {
    if (millis() < flashExpireTime) {
      gfx->fillRoundRect(133, 375, 200, 26, 6, COLOR_EMERALD);
      gfx->setTextColor(COLOR_TEXT_WHITE);
      gfx->setTextSize(2);
      gfx->setCursor(160, 380);
      gfx->println("DROP HIT! *");
    } else {
      dropTriggerVisualFlash = false;
      gfx->fillRect(133, 375, 200, 26, COLOR_BG);
    }
  }
}

// ==========================================
// 8. SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==============================================");
  Serial.println("  ESP32-S3 AMOLED - Droplet Acoustic Calibrator");
  Serial.println("==============================================");

  // Turn on display
  gfx->begin();
  gfx->fillScreen(COLOR_BG);
  renderStaticUI();

  // Initialize I2C Bus & Codecs
  initPowerAndI2C();
  initI2S();     // Starts MCLK & sets GPIO 10 as input
  initES7210();  // Configures microphone ADC registers

  Serial.println("\n[READY] Calibration Active!");
  Serial.println("-> Open Arduino IDE > Tools > Serial Plotter (Ctrl+Shift+L)");
  Serial.println("-> Format: RawSignal, Threshold, DropTrigger, DropCount");
}

void loop() {
  size_t bytesRead = 0;
  
  // Read raw audio samples from I2S
  esp_err_t result = i2s_read(I2S_PORT, audioBuffer, sizeof(audioBuffer), &bytesRead, 100 / portTICK_PERIOD_MS);

  int32_t currentPeak = 0;
  int64_t sumSquares = 0;
  int samplesRead = bytesRead / sizeof(int16_t);

  if (result == ESP_OK && samplesRead > 0) {
    for (int i = 0; i < samplesRead; i++) {
      int16_t sample = audioBuffer[i];
      int32_t absVal = abs(sample);
      if (absVal > currentPeak) {
        currentPeak = absVal;
      }
      sumSquares += (int64_t)sample * sample;
    }
    peakAmplitude = currentPeak;
    rmsEnergy = sqrt((float)sumSquares / samplesRead);
    
    // Slow exponential rolling average for noise floor
    noiseFloor = (noiseFloor * 0.95f) + ((float)peakAmplitude * 0.05f);
  }

  // Droplet Impact Trigger Logic
  int dropTriggerSpike = 0;
  unsigned long now = millis();
  
  if (peakAmplitude > currentThreshold && (now - lastDropTime > DEBOUNCE_MS)) {
    lastDropTime = now;
    physicalDropCount++;
    dropTriggerSpike = currentThreshold * 1.5; // Visual spike in Serial Plotter
    dropTriggerVisualFlash = true;
    flashExpireTime = now + 180; // 180ms on-screen flash
    
    Serial.printf("\n[*** DROP HIT #%lu ***] Peak: %ld | Noise: %.1f | Threshold: %ld\n", 
                  physicalDropCount, peakAmplitude, noiseFloor, currentThreshold);
  }

  // Stream data to Arduino Serial Plotter
  // Format: Name:Value,Name:Value enables native colored legend lines in Arduino IDE 2.x
  Serial.printf("RawSignal:%ld,Threshold:%ld,DropTrigger:%d,DropCount:%lu\n",
                peakAmplitude, currentThreshold, dropTriggerSpike, physicalDropCount);

  // Check for User Input in Serial Monitor
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '+' || c == '=') {
      currentThreshold += 250;
      Serial.printf("[CONFIG] Threshold increased to: %ld\n", currentThreshold);
    } else if (c == '-' || c == '_') {
      if (currentThreshold > 500) currentThreshold -= 250;
      Serial.printf("[CONFIG] Threshold decreased to: %ld\n", currentThreshold);
    } else if (c == 'r' || c == 'R') {
      physicalDropCount = 0;
      Serial.println("[CONFIG] Drop count reset to 0.");
    }
  }

  // Periodic Display Refresh (~25 FPS)
  if (now - lastUiUpdate >= UI_INTERVAL_MS) {
    lastUiUpdate = now;
    updateDynamicUI();
  }
}
