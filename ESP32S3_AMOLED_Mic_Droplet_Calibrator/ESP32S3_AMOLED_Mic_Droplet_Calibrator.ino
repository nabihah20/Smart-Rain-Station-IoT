/*
 * ==============================================================================
 * ESP32-S3 Touch AMOLED 1.75 - High-Gain Onboard Dual-Mic Droplet & Tip Calibrator
 * ==============================================================================
 * Project: Smart Rain Station IoT (Workstream 05: Real-Time Acoustic Detection)
 * Target Hardware: Waveshare ESP32-S3 Touch AMOLED 1.75
 * Codec / ADC: Onboard ES7210 4-Channel Audio ADC + Dual MEMS Microphones
 * Display: 466x466 Round CO5300 QSPI AMOLED (GFX_Library_for_Arduino)
 *
 * KEY UPDATES:
 * 1. Enables AXP2101 power rails and TCA9554 expander.
 * 2. Un-mutes and powers on MIC1 & MIC2 analog bias/PGA on the ES7210.
 * 3. Sets Maximum Preamplifier Gain (+37.5 dB) on both front microphones.
 * 4. Configures 16-bit Standard I2S output on SDOUT1 (DIPIN = GPIO 10).
 * 5. Dual Detection: Detects both subtle water drops AND loud mechanical bucket tips!
 * 6. Live Waveform streaming to Arduino Serial Plotter (Ctrl + Shift + L).
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
#define MCLKPIN     42  // GPIO 42 (Shared MCLK for ES7210 & ES8311)
#define BCLKPIN     9   // GPIO 9  (Bit Clock)
#define WSPIN       45  // GPIO 45 (LRCK / Word Select)
#define DOPIN       8   // GPIO 8  (ESP32 TX to ES8311 Speaker)
#define DIPIN       10  // GPIO 10 (ES7210 Mic SDOUT1 to ESP32 RX)
#define PA_EN_PIN   46  // Speaker Amp Enable

// I2C Addresses
#define ES7210_ADDR   0x40  // Microphone Audio ADC
#define ES8311_ADDR   0x18  // Speaker Audio DAC
#define TCA9554_ADDR  0x20  // Onboard IO Expander
#define AXP2101_ADDR  0x34  // Power Management IC

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
#define COLOR_WHITE         0xFFFF
#define COLOR_TEXT_MUTED    0x9CD3 // Slate gray
#define COLOR_CYAN          0x3DFE // Electric cyan
#define COLOR_AMBER         0xFD20 // Warning / amber
#define COLOR_EMERALD       0x170A // Green / success
#define COLOR_CRIMSON       0xF800 // Red / peak
#define COLOR_VIOLET        0x8B5C // Violet / mechanical tip
#define COLOR_TRACK         0x18E3 // Dark gray track
#define COLOR_BORDER        0x0459 // Cyan border

// ==========================================
// 3. AUDIO BUFFER & DETECTION CONSTANTS
// ==========================================
#define SAMPLE_RATE         16000
#define SAMPLES_PER_READ    256

static int16_t audioBuffer[SAMPLES_PER_READ * 2]; // 16-bit stereo interleaved

// Detection State Variables
int32_t dropThreshold      = 2200;   // Sensitivity threshold for subtle water drop
int32_t tipThreshold       = 12000;  // Threshold for loud mechanical bucket tip click
int32_t peakAmplitude      = 0;
float   rmsEnergy          = 0.0f;
float   noiseFloor         = 300.0f;

uint32_t physicalDropCount = 0;
uint32_t physicalTipCount  = 0;

unsigned long lastDropTime = 0;
unsigned long lastTipTime  = 0;
const unsigned long DROP_DEBOUNCE_MS = 60;   // Drop debounce
const unsigned long TIP_DEBOUNCE_MS  = 300;  // Tip mechanical bounce suppression

// Visual flash banner
bool triggerVisualFlash = false;
char flashText[32] = "";
uint16_t flashColor = COLOR_EMERALD;
unsigned long flashExpireTime = 0;

// UI Refresh timer
unsigned long lastUiUpdate = 0;
const unsigned long UI_INTERVAL_MS = 40; // ~25 FPS

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
// 5. HARDWARE INITIALIZATION
// ==========================================
void initHardwarePower() {
  Wire.begin(IIC_SDA, IIC_SCL, 400000);
  delay(50);
  Serial.println("\n[I2C] Scanning connected peripherals...");
  
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  -> Found I2C Device at 0x%02X", addr);
      if (addr == ES7210_ADDR)  Serial.print(" (ES7210 Microphone ADC)");
      if (addr == ES8311_ADDR)  Serial.print(" (ES8311 Speaker Codec)");
      if (addr == TCA9554_ADDR) Serial.print(" (TCA9554 IO Expander)");
      if (addr == AXP2101_ADDR) Serial.print(" (AXP2101 PMIC)");
      Serial.println();
    }
  }

  // 1. Ensure AXP2101 powers all internal analog audio rails
  writeI2CReg(AXP2101_ADDR, 0x90, 0xFF); // ALDO1-4 Enable (Analog Codec Power)
  writeI2CReg(AXP2101_ADDR, 0x91, 0xFF); // BLDO1-2 & DLDO Enable
  delay(30);

  // 2. Initialize TCA9554 IO Expander (P0-P7 outputs)
  writeI2CReg(TCA9554_ADDR, 0x03, 0x00); // Set all pins as output
  writeI2CReg(TCA9554_ADDR, 0x01, 0xFF); // Drive high
  delay(20);
}

// Initialize I2S peripheral (provides MCLK to ES7210 on GPIO 42)
void initI2S() {
  Serial.println("[I2S] Initializing I2S peripheral & MCLK (GPIO 42)...");

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
    .mck_io_num = MCLKPIN,   // GPIO 42 (MCLK out)
    .bck_io_num = BCLKPIN,   // GPIO 9  (BCLK out)
    .ws_io_num = WSPIN,      // GPIO 45 (LRCK/WS out)
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = DIPIN     // GPIO 10 (MIC DATA IN from ES7210 SDOUT1)
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("[I2S] Install error: %d\n", err);
  }
  
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("[I2S] Set pin error: %d\n", err);
  }

  i2s_zero_dma_buffer(I2S_PORT);
  delay(50); // Allow MCLK clock to stabilize before ES7210 register writes
  Serial.println("[I2S] I2S Audio Bus & MCLK active.");
}

// Complete official ES7210 configuration sequence for high sensitivity
void initES7210() {
  Serial.println("[ES7210] Configuring ES7210 microphone registers...");

  // 1. Chip Software Reset
  writeI2CReg(ES7210_ADDR, 0x00, 0xFF);
  delay(15);
  writeI2CReg(ES7210_ADDR, 0x00, 0x41); // Normal wake mode
  delay(10);

  // 2. Clock Gating & Power Domains
  writeI2CReg(ES7210_ADDR, 0x01, 0x3F); // Clock gating enable
  writeI2CReg(ES7210_ADDR, 0x09, 0x30); // State cycles
  writeI2CReg(ES7210_ADDR, 0x0A, 0x30); // Power on state cycle

  // 3. High Pass Filters (removes low hum / DC offset)
  writeI2CReg(ES7210_ADDR, 0x23, 0x2A);
  writeI2CReg(ES7210_ADDR, 0x22, 0x0A);
  writeI2CReg(ES7210_ADDR, 0x20, 0x0A);
  writeI2CReg(ES7210_ADDR, 0x21, 0x2A);

  // 4. Set Slave Mode (ESP32 provides clocks)
  writeI2CReg(ES7210_ADDR, 0x08, 0x00);

  // 5. Analog Power & Microphone Bias (2.87V Bias Voltage for MEMS)
  writeI2CReg(ES7210_ADDR, 0x40, 0x43); // Analog power up
  writeI2CReg(ES7210_ADDR, 0x41, 0x70); // MIC1/2 Bias = 2.87V
  writeI2CReg(ES7210_ADDR, 0x42, 0x70); // MIC3/4 Bias = 2.87V
  writeI2CReg(ES7210_ADDR, 0x07, 0x20); // OSR setting
  writeI2CReg(ES7210_ADDR, 0x02, 0xC1); // Main clock clear & divide

  // 6. Power Up Pre-Amplifiers (PGA) for MIC1 & MIC2
  writeI2CReg(ES7210_ADDR, 0x47, 0x08); // MIC1 Power normal
  writeI2CReg(ES7210_ADDR, 0x48, 0x08); // MIC2 Power normal
  writeI2CReg(ES7210_ADDR, 0x49, 0x08);
  writeI2CReg(ES7210_ADDR, 0x4A, 0x08);

  // 7. Enable ADC Power for MIC1/2
  writeI2CReg(ES7210_ADDR, 0x4B, 0x00); // 0x00 = Power ON MIC1/2 ADC & PGA
  writeI2CReg(ES7210_ADDR, 0x4C, 0xFF); // Power down unused MIC3/4

  // 8. Set MAXIMUM ANALOG GAIN (+37.5 dB) on both front microphones!
  // Bit 4 = Enable (0x10), Bits 3:0 = GAIN_37_5DB (0x0E) -> 0x1E
  writeI2CReg(ES7210_ADDR, 0x43, 0x1E); // MIC1 Gain: +37.5 dB (Max sensitivity)
  writeI2CReg(ES7210_ADDR, 0x44, 0x1E); // MIC2 Gain: +37.5 dB (Max sensitivity)

  // 9. Digital Interface: Standard Philips I2S, 16-bit
  writeI2CReg(ES7210_ADDR, 0x11, 0x60); // 16-bit standard I2S format
  writeI2CReg(ES7210_ADDR, 0x12, 0x00); // 0x00 = Standard Stereo output on SDOUT1

  // 10. Un-mute ADC digital outputs
  writeI2CReg(ES7210_ADDR, 0x15, 0x00); // Un-mute ADC 1 & 2
  writeI2CReg(ES7210_ADDR, 0x14, 0x03); // Mute unused ADC 3 & 4

  // 11. Final Power Enable
  writeI2CReg(ES7210_ADDR, 0x06, 0x00); // Clear power down
  writeI2CReg(ES7210_ADDR, 0x01, 0x00); // Enable all ADC clocks

  // Re-latch state
  writeI2CReg(ES7210_ADDR, 0x00, 0x71);
  delay(10);
  writeI2CReg(ES7210_ADDR, 0x00, 0x41);

  Serial.println("[ES7210] Microphones powered up at MAXIMUM +37.5 dB Gain!");
}

// ==========================================
// 6. UI RENDERING (AMOLED ROUND SCREEN)
// ==========================================
void renderStaticUI() {
  gfx->fillScreen(COLOR_BG);
  
  // Outer perimeter rings
  gfx->drawCircle(233, 233, 230, COLOR_BORDER);
  gfx->drawCircle(233, 233, 229, COLOR_TRACK);

  // Title
  gfx->setTextColor(COLOR_CYAN);
  gfx->setTextSize(2);
  gfx->setCursor(105, 38);
  gfx->println("ACOUSTIC STATION");

  gfx->setTextColor(COLOR_TEXT_MUTED);
  gfx->setTextSize(1);
  gfx->setCursor(140, 62);
  gfx->println("ES7210 ONBOARD DUAL MIC");

  // Dual Counters Box
  // Left: DROPS
  gfx->drawRoundRect(40, 85, 185, 125, 16, COLOR_TRACK);
  gfx->setTextColor(COLOR_TEXT_MUTED);
  gfx->setTextSize(2);
  gfx->setCursor(95, 98);
  gfx->println("DROPS");

  // Right: TIPS
  gfx->drawRoundRect(241, 85, 185, 125, 16, COLOR_TRACK);
  gfx->setTextColor(COLOR_TEXT_MUTED);
  gfx->setTextSize(2);
  gfx->setCursor(308, 98);
  gfx->println("TIPS");

  // VU Meter Outline
  gfx->drawRoundRect(40, 230, 386, 32, 8, COLOR_TRACK);

  // Sub-metrics
  gfx->setTextColor(COLOR_TEXT_MUTED);
  gfx->setTextSize(2);
  gfx->setCursor(50, 280);
  gfx->print("PEAK:");
  gfx->setCursor(240, 280);
  gfx->print("NOISE:");

  gfx->setCursor(50, 312);
  gfx->print("DROP THRESH:");
  gfx->setCursor(50, 342);
  gfx->print("TIP  THRESH:");

  gfx->setCursor(55, 410);
  gfx->setTextSize(1);
  gfx->setTextColor(COLOR_CYAN);
  gfx->println("[Serial: '+' / '-' to adjust Drop Thresh | 'r' to reset]");
}

void updateDynamicUI() {
  // 1. Update Drop & Tip Numbers
  gfx->fillRect(45, 122, 175, 80, COLOR_BG);
  gfx->setTextColor(COLOR_CYAN);
  gfx->setTextSize(6);
  gfx->setCursor(85, 132);
  gfx->printf("%lu", physicalDropCount);

  gfx->fillRect(246, 122, 175, 80, COLOR_BG);
  gfx->setTextColor(COLOR_AMBER);
  gfx->setTextSize(6);
  gfx->setCursor(295, 132);
  gfx->printf("%lu", physicalTipCount);

  // 2. VU Meter Bar (0 to 20,000 range mapped to 380px)
  int barWidth = map(constrain(peakAmplitude, 0, 20000), 0, 20000, 0, 380);
  gfx->fillRect(43, 233, 380, 26, COLOR_BG); // Clear previous bar
  
  uint16_t barColor = COLOR_EMERALD;
  if (peakAmplitude >= dropThreshold) barColor = COLOR_AMBER;
  if (peakAmplitude >= tipThreshold)  barColor = COLOR_CRIMSON;
  
  if (barWidth > 0) {
    gfx->fillRoundRect(43, 233, barWidth, 26, 6, barColor);
  }
  
  // Draw threshold tick marks on VU meter
  int dropTickX = map(constrain(dropThreshold, 0, 20000), 0, 20000, 0, 380) + 43;
  int tipTickX  = map(constrain(tipThreshold,  0, 20000), 0, 20000, 0, 380) + 43;
  gfx->drawFastVLine(dropTickX, 226, 40, COLOR_WHITE);
  gfx->drawFastVLine(tipTickX,  226, 40, COLOR_CRIMSON);

  // 3. Numeric Readings
  gfx->fillRect(115, 280, 110, 20, COLOR_BG);
  gfx->fillRect(320, 280, 100, 20, COLOR_BG);
  gfx->fillRect(205, 312, 110, 20, COLOR_BG);
  gfx->fillRect(205, 342, 110, 20, COLOR_BG);

  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_WHITE);
  gfx->setCursor(115, 280);
  gfx->printf("%ld", peakAmplitude);

  gfx->setCursor(320, 280);
  gfx->printf("%d", (int)noiseFloor);

  gfx->setTextColor(COLOR_CYAN);
  gfx->setCursor(205, 312);
  gfx->printf("%ld", dropThreshold);

  gfx->setTextColor(COLOR_AMBER);
  gfx->setCursor(205, 342);
  gfx->printf("%ld", tipThreshold);

  // 4. Event Trigger Banner
  if (triggerVisualFlash) {
    if (millis() < flashExpireTime) {
      gfx->fillRoundRect(123, 372, 220, 28, 6, flashColor);
      gfx->setTextColor(COLOR_WHITE);
      gfx->setTextSize(2);
      gfx->setCursor(140, 378);
      gfx->println(flashText);
    } else {
      triggerVisualFlash = false;
      gfx->fillRect(123, 372, 220, 28, COLOR_BG);
    }
  }
}

// ==========================================
// 7. SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==============================================");
  Serial.println(" ESP32-S3 AMOLED: High-Gain Dual-Mic Calibrator");
  Serial.println("==============================================");

  // Turn on display
  gfx->begin();
  gfx->fillScreen(COLOR_BG);
  renderStaticUI();

  // Initialize Hardware
  initHardwarePower();
  initI2S();     // Starts MCLK & configures GPIO 10 as RX
  initES7210();  // Applies full power & +37.5 dB gain

  Serial.println("\n[READY] Calibration system listening!");
  Serial.println("-> Open Tools > Serial Plotter (Ctrl + Shift + L)");
  Serial.println("-> Format: RawSignal,DropThreshold,TipThreshold,DropHit,TipHit");
}

void loop() {
  size_t bytesRead = 0;
  
  // Pull raw PCM audio data from I2S
  esp_err_t result = i2s_read(I2S_PORT, audioBuffer, sizeof(audioBuffer), &bytesRead, 100 / portTICK_PERIOD_MS);

  int32_t currentPeak = 0;
  int samplesRead = bytesRead / sizeof(int16_t);

  if (result == ESP_OK && samplesRead > 0) {
    for (int i = 0; i < samplesRead; i++) {
      int16_t sample = audioBuffer[i];
      int32_t absVal = abs(sample);
      if (absVal > currentPeak) {
        currentPeak = absVal;
      }
    }
    peakAmplitude = currentPeak;
    
    // Slow rolling average for background noise floor
    noiseFloor = (noiseFloor * 0.95f) + ((float)peakAmplitude * 0.05f);
  }

  unsigned long now = millis();
  int dropPlotSpike = 0;
  int tipPlotSpike  = 0;

  // 1. Check for MECHANICAL BUCKET TIP (Very Loud Shockwave)
  if (peakAmplitude >= tipThreshold && (now - lastTipTime > TIP_DEBOUNCE_MS)) {
    lastTipTime = now;
    lastDropTime = now + 150; // Prevent tip reverberation from triggering a false drop
    physicalTipCount++;
    tipPlotSpike = 20000;
    
    triggerVisualFlash = true;
    strcpy(flashText, "BUCKET TIP! ***");
    flashColor = COLOR_AMBER;
    flashExpireTime = now + 350;

    Serial.printf("\n[***** MECHANICAL TIP #%lu *****] Peak: %ld (LOUD)\n", physicalTipCount, peakAmplitude);
  }
  // 2. Check for SINGLE WATER DROP IMPACT
  else if (peakAmplitude >= dropThreshold && (now - lastDropTime > DROP_DEBOUNCE_MS)) {
    lastDropTime = now;
    physicalDropCount++;
    dropPlotSpike = 8000;
    
    triggerVisualFlash = true;
    strcpy(flashText, "DROP HIT! *");
    flashColor = COLOR_EMERALD;
    flashExpireTime = now + 180;

    Serial.printf("[DROP HIT #%lu] Peak: %ld | Noise: %.1f\n", physicalDropCount, peakAmplitude, noiseFloor);
  }

  // Stream data to Arduino Serial Plotter
  Serial.printf("RawSignal:%ld,DropThresh:%ld,TipThresh:%ld,DropHit:%d,TipHit:%d\n",
                peakAmplitude, dropThreshold, tipThreshold, dropPlotSpike, tipPlotSpike);

  // Serial Monitor Keyboard Adjustments
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '+' || c == '=') {
      dropThreshold += 250;
      Serial.printf("[CONFIG] Drop Threshold raised to: %ld\n", dropThreshold);
    } else if (c == '-' || c == '_') {
      if (dropThreshold > 400) dropThreshold -= 250;
      Serial.printf("[CONFIG] Drop Threshold lowered to: %ld\n", dropThreshold);
    } else if (c == 'r' || c == 'R') {
      physicalDropCount = 0;
      physicalTipCount = 0;
      Serial.println("[CONFIG] Counters reset to 0.");
    }
  }

  // Refresh display at ~25 FPS
  if (now - lastUiUpdate >= UI_INTERVAL_MS) {
    lastUiUpdate = now;
    updateDynamicUI();
  }
}
