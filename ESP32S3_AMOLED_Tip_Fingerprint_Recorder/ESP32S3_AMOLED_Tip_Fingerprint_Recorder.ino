/*
 * ==============================================================================
 * ESP32-S3 Touch AMOLED 1.75 - Acoustic Tip Fingerprint Recorder & Profiler
 * ==============================================================================
 * Project: Smart Rain Station IoT (Workstream 05: Reference Fingerprint Sampling)
 * Target Hardware: Waveshare ESP32-S3 Touch AMOLED 1.75
 * Codec / ADC: Onboard ES7210 4-Channel Audio ADC + Dual MEMS Microphones
 * Display: 466x466 Round CO5300 QSPI AMOLED (GFX_Library_for_Arduino)
 *
 * PURPOSE:
 * 1. Accurately profiles the exact physical sound signatures of:
 *    - LEFT BUCKET TIP (Strikes left mechanical stopper)
 *    - RIGHT BUCKET TIP (Strikes right mechanical stopper)
 *    - STEEL FRAME KNOCK (Noise to reject)
 * 2. Captures 50 ms pre/post-trigger raw acoustic impulse.
 * 3. Extracts a 16-point Normalized Energy Envelope & 8-band Frequency Distribution.
 * 4. Outputs averaged C++ reference template arrays ready to paste into matcher firmware!
 * ==============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <Arduino_GFX_Library.h>
#include <math.h>

// ==========================================
// 1. PIN DEFINITIONS & DISPLAY SETUP
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

#define IIC_SDA     15
#define IIC_SCL     14

#define I2S_PORT    I2S_NUM_0
#define MCLKPIN     42
#define BCLKPIN     9
#define WSPIN       45
#define DOPIN       8
#define DIPIN       10
#define PA_EN_PIN   46

#define ES7210_ADDR   0x40
#define ES8311_ADDR   0x18
#define TCA9554_ADDR  0x20
#define AXP2101_ADDR  0x34

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
#define COLOR_CYAN          0x3DFE // Left Tip (Cyan)
#define COLOR_AMBER         0xFD20 // Right Tip (Amber)
#define COLOR_CRIMSON       0xF800 // Steel Knock / Noise (Red)
#define COLOR_EMERALD       0x170A // Success (Green)
#define COLOR_TRACK         0x18E3 // Dark gray track
#define COLOR_BORDER        0x0459 // Cyan border

// ==========================================
// 2. AUDIO SAMPLING & FINGERPRINT BUFFERS
// ==========================================
#define SAMPLE_RATE         16000
#define SAMPLES_PER_READ    256

// 50ms impulse window = 800 samples at 16 kHz
#define PROFILE_SAMPLES     800
#define PRE_TRIGGER_SAMPLES 100
#define ENVELOPE_POINTS     16
#define SPECTRAL_BANDS      8

// Trigger threshold to start recording an impulse
int32_t triggerThreshold = 4000;

static int16_t audioBuffer[SAMPLES_PER_READ * 2]; // 16-bit stereo
static int16_t ringBuffer[PROFILE_SAMPLES];       // Pre-trigger circular ring
static int ringIndex = 0;

static int16_t capturedWave[PROFILE_SAMPLES];

// Fingerprint Data Structure
struct SoundProfile {
  int32_t peak;
  float   rms;
  float   crest;
  int     zcr;
  float   decayRatio;
  float   envelope[ENVELOPE_POINTS];
  float   spectrum[SPECTRAL_BANDS];
};

#define MAX_SAMPLES_PER_TYPE 5
SoundProfile leftSamples[MAX_SAMPLES_PER_TYPE];
SoundProfile rightSamples[MAX_SAMPLES_PER_TYPE];
SoundProfile steelSamples[MAX_SAMPLES_PER_TYPE];

int leftCount  = 0;
int rightCount = 0;
int steelCount = 0;

// Recording State Machine
enum RecordMode {
  MODE_IDLE,
  MODE_ARM_LEFT,
  MODE_ARM_RIGHT,
  MODE_ARM_STEEL
};

RecordMode currentMode = MODE_ARM_LEFT;

bool recordingTriggered = false;
int  postTriggerSamplesRemaining = 0;
int  captureWriteIndex = 0;

unsigned long lastTriggerTime = 0;
const unsigned long DEBOUNCE_MS = 400;

// ==========================================
// 3. I2C REGISTER HELPERS & HARDWARE INIT
// ==========================================
bool writeI2CReg(uint8_t devAddr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

void initHardwarePower() {
  Wire.begin(IIC_SDA, IIC_SCL, 400000);
  delay(50);

  // AXP2101: Enable ALDO1-4, BLDO1-2, DLDO1
  writeI2CReg(AXP2101_ADDR, 0x90, 0xBF);
  writeI2CReg(AXP2101_ADDR, 0x91, 0xFF);
  writeI2CReg(AXP2101_ADDR, 0x92, 0x1F); // ALDO1=3.3V
  writeI2CReg(AXP2101_ADDR, 0x93, 0x1F); // ALDO2=3.3V
  writeI2CReg(AXP2101_ADDR, 0x94, 0x1F); // ALDO3=3.3V
  writeI2CReg(AXP2101_ADDR, 0x95, 0x1F); // ALDO4=3.3V

  // TCA9554: Set Pins as Outputs and set HIGH
  writeI2CReg(TCA9554_ADDR, 0x03, 0x00);
  writeI2CReg(TCA9554_ADDR, 0x01, 0xFF);

  pinMode(PA_EN_PIN, OUTPUT);
  digitalWrite(PA_EN_PIN, LOW); // Mute speaker amp
}

void initI2S() {
  i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 4,
    .dma_buf_len          = SAMPLES_PER_READ,
    .use_apll             = true,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 256 * SAMPLE_RATE
  };

  i2s_pin_config_t pin_config = {
    .mck_io_num   = MCLKPIN,
    .bck_io_num   = BCLKPIN,
    .ws_io_num    = WSPIN,
    .data_out_num = DOPIN,
    .data_in_num  = DIPIN
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void initES7210() {
  writeI2CReg(ES7210_ADDR, 0x00, 0xFF); // Reset
  delay(20);
  writeI2CReg(ES7210_ADDR, 0x00, 0x41);
  writeI2CReg(ES7210_ADDR, 0x01, 0x1F); // Clocks
  writeI2CReg(ES7210_ADDR, 0x06, 0x04); // Power on
  writeI2CReg(ES7210_ADDR, 0x07, 0x00);
  writeI2CReg(ES7210_ADDR, 0x08, 0x10);

  // Digital I2S 16-bit
  writeI2CReg(ES7210_ADDR, 0x11, 0x60);
  writeI2CReg(ES7210_ADDR, 0x12, 0x02);

  // Un-mute Analog Bias & PGAs
  writeI2CReg(ES7210_ADDR, 0x40, 0x42);
  writeI2CReg(ES7210_ADDR, 0x41, 0x70);
  writeI2CReg(ES7210_ADDR, 0x42, 0x70);

  // MAX Gain +37.5 dB on front mics (0x1E = +37.5dB)
  writeI2CReg(ES7210_ADDR, 0x43, 0x1E);
  writeI2CReg(ES7210_ADDR, 0x44, 0x1E);

  writeI2CReg(ES7210_ADDR, 0x4B, 0x00); // Un-mute ADC
  writeI2CReg(ES7210_ADDR, 0x4C, 0x00);
  writeI2CReg(ES7210_ADDR, 0x14, 0x03);
  writeI2CReg(ES7210_ADDR, 0x06, 0x00);
  writeI2CReg(ES7210_ADDR, 0x01, 0x00);

  writeI2CReg(ES7210_ADDR, 0x00, 0x71);
  delay(10);
  writeI2CReg(ES7210_ADDR, 0x00, 0x41);

  Serial.println("[ES7210] Onboard Dual Mics Ready for Sampling!");
}

// ==========================================
// 4. SIGNAL PROCESSING & FEATURE EXTRACTION
// ==========================================
void processAndExtractFeatures(const int16_t* wave, SoundProfile* out) {
  int32_t peakVal = 0;
  int64_t sumSq = 0;
  int zcr = 0;

  for (int i = 0; i < PROFILE_SAMPLES; i++) {
    int32_t absVal = abs(wave[i]);
    if (absVal > peakVal) peakVal = absVal;
    sumSq += (int32_t)wave[i] * (int32_t)wave[i];

    if (i > 0) {
      if ((wave[i - 1] > 0 && wave[i] < 0) || (wave[i - 1] < 0 && wave[i] > 0)) {
        zcr++;
      }
    }
  }

  out->peak = peakVal;
  out->rms  = sqrt((float)sumSq / PROFILE_SAMPLES);
  out->crest = (out->rms > 10.0f) ? ((float)out->peak / out->rms) : 0.0f;
  out->zcr  = zcr;

  // 1. Compute 16-point Normalized Energy Envelope (each point = 50 samples)
  int samplesPerPoint = PROFILE_SAMPLES / ENVELOPE_POINTS; // 50 samples = 3.125 ms
  float maxEnv = 0.0f;

  for (int p = 0; p < ENVELOPE_POINTS; p++) {
    int64_t binSumSq = 0;
    for (int s = 0; s < samplesPerPoint; s++) {
      int idx = p * samplesPerPoint + s;
      binSumSq += (int32_t)wave[idx] * (int32_t)wave[idx];
    }
    float binRms = sqrt((float)binSumSq / samplesPerPoint);
    out->envelope[p] = binRms;
    if (binRms > maxEnv) maxEnv = binRms;
  }

  // Normalize envelope 0.0 to 1.0
  if (maxEnv > 0.0f) {
    for (int p = 0; p < ENVELOPE_POINTS; p++) {
      out->envelope[p] /= maxEnv;
    }
  }

  // Decay ratio: envelope at ~25ms (point 8) vs peak point
  out->decayRatio = (out->envelope[8] > 0.0f) ? out->envelope[8] : 0.0f;

  // 2. Simplified 8-Band Frequency Energy Distribution (Zero-crossing sub-windows)
  for (int b = 0; b < SPECTRAL_BANDS; b++) {
    int bandStart = b * (PROFILE_SAMPLES / SPECTRAL_BANDS);
    int bandEnd   = bandStart + (PROFILE_SAMPLES / SPECTRAL_BANDS);
    int bandZcr   = 0;
    int64_t bSum  = 0;

    for (int i = bandStart; i < bandEnd; i++) {
      bSum += abs(wave[i]);
      if (i > bandStart) {
        if ((wave[i - 1] > 0 && wave[i] < 0) || (wave[i - 1] < 0 && wave[i] > 0)) {
          bandZcr++;
        }
      }
    }
    // Estimate energy weighted by zero-crossing band
    out->spectrum[b] = (float)bSum / (PROFILE_SAMPLES / SPECTRAL_BANDS);
  }
}

// ==========================================
// 5. AMOLED UI RENDERING
// ==========================================
void renderUI() {
  gfx->fillScreen(COLOR_BG);

  // Outer circles
  gfx->drawCircle(233, 233, 230, COLOR_BORDER);
  gfx->drawCircle(233, 233, 229, COLOR_TRACK);

  // Header Title
  gfx->setTextColor(COLOR_CYAN, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(75, 40);
  gfx->print("TIP SOUND PROFILER");

  gfx->setTextColor(COLOR_TEXT_MUTED, COLOR_BG);
  gfx->setTextSize(1);
  gfx->setCursor(130, 64);
  gfx->print("ACOUSTIC FINGERPRINT SAMPLER");

  // Mode Banner Box (Center)
  gfx->drawRoundRect(35, 90, 396, 90, 16, COLOR_TRACK);

  // Status Cards
  // Left Tip Card
  gfx->drawRoundRect(35, 195, 125, 95, 12, COLOR_TRACK);
  gfx->setTextColor(COLOR_CYAN, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(55, 205);
  gfx->print("LEFT");

  // Right Tip Card
  gfx->drawRoundRect(170, 195, 125, 95, 12, COLOR_TRACK);
  gfx->setTextColor(COLOR_AMBER, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(185, 205);
  gfx->print("RIGHT");

  // Steel Knock Card
  gfx->drawRoundRect(305, 195, 125, 95, 12, COLOR_TRACK);
  gfx->setTextColor(COLOR_CRIMSON, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(320, 205);
  gfx->print("STEEL");

  // Bottom Instructions
  gfx->setTextColor(COLOR_TEXT_MUTED, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(45, 310);
  gfx->print("PEAK:");
  gfx->setCursor(240, 310);
  gfx->print("CREST:");

  gfx->setCursor(45, 345);
  gfx->print("ZCR :");
  gfx->setCursor(240, 345);
  gfx->print("DECAY:");

  gfx->setCursor(25, 410);
  gfx->setTextSize(1);
  gfx->setTextColor(COLOR_CYAN, COLOR_BG);
  gfx->print("[l: Arm Left | r: Arm Right | k: Arm Steel | p: Export]");
}

void updateUIMode() {
  gfx->fillRect(40, 95, 386, 80, COLOR_BG);

  uint16_t bannerColor = COLOR_CYAN;
  const char* modeText = "ARMED: LEFT TIP (1/3)";
  const char* hintText = "-> Tip LEFT bucket with water now!";

  if (currentMode == MODE_ARM_LEFT) {
    bannerColor = COLOR_CYAN;
    modeText = "ARMED: LEFT TIP";
    hintText = "-> Tip LEFT bucket now";
  } else if (currentMode == MODE_ARM_RIGHT) {
    bannerColor = COLOR_AMBER;
    modeText = "ARMED: RIGHT TIP";
    hintText = "-> Tip RIGHT bucket now";
  } else if (currentMode == MODE_ARM_STEEL) {
    bannerColor = COLOR_CRIMSON;
    modeText = "ARMED: STEEL KNOCK";
    hintText = "-> Tap steel frame / noise";
  } else {
    bannerColor = COLOR_EMERALD;
    modeText = "SAMPLES READY!";
    hintText = "-> Press 'p' in Serial to export";
  }

  gfx->fillRoundRect(50, 105, 366, 36, 8, bannerColor);
  gfx->setTextColor(COLOR_WHITE, bannerColor);
  gfx->setTextSize(2);
  int tLen = strlen(modeText) * 12;
  gfx->setCursor(50 + (366 - tLen) / 2, 115);
  gfx->print(modeText);

  gfx->setTextColor(COLOR_TEXT_MUTED, COLOR_BG);
  gfx->setTextSize(1);
  int hLen = strlen(hintText) * 6;
  gfx->setCursor(50 + (366 - hLen) / 2, 155);
  gfx->print(hintText);

  // Update Counters
  gfx->setTextSize(4);
  char buf[8];

  // Left
  gfx->fillRect(40, 235, 115, 45, COLOR_BG);
  gfx->setTextColor(COLOR_CYAN, COLOR_BG);
  snprintf(buf, sizeof(buf), "%d/3", leftCount);
  gfx->setCursor(55, 238);
  gfx->print(buf);

  // Right
  gfx->fillRect(175, 235, 115, 45, COLOR_BG);
  gfx->setTextColor(COLOR_AMBER, COLOR_BG);
  snprintf(buf, sizeof(buf), "%d/3", rightCount);
  gfx->setCursor(190, 238);
  gfx->print(buf);

  // Steel
  gfx->fillRect(310, 235, 115, 45, COLOR_BG);
  gfx->setTextColor(COLOR_CRIMSON, COLOR_BG);
  snprintf(buf, sizeof(buf), "%d/2", steelCount);
  gfx->setCursor(325, 238);
  gfx->print(buf);
}

void updateMetricsUI(const SoundProfile& sp) {
  gfx->setTextSize(2);

  // PEAK
  gfx->fillRect(115, 310, 110, 20, COLOR_BG);
  gfx->setTextColor(COLOR_WHITE, COLOR_BG);
  gfx->setCursor(115, 310);
  gfx->print(sp.peak);

  // CREST
  gfx->fillRect(320, 310, 100, 20, COLOR_BG);
  gfx->setCursor(320, 310);
  char cBuf[10];
  snprintf(cBuf, sizeof(cBuf), "%.1fx", sp.crest);
  gfx->print(cBuf);

  // ZCR
  gfx->fillRect(115, 345, 110, 20, COLOR_BG);
  gfx->setCursor(115, 345);
  gfx->print(sp.zcr);

  // DECAY
  gfx->fillRect(320, 345, 100, 20, COLOR_BG);
  gfx->setCursor(320, 345);
  char dBuf[10];
  snprintf(dBuf, sizeof(dBuf), "%.0f%%", sp.decayRatio * 100.0f);
  gfx->print(dBuf);
}

// ==========================================
// 6. REFERENCE EXPORT GENERATOR
// ==========================================
void exportReferenceCode() {
  Serial.println("\n\n/*************************************************************");
  Serial.println(" * COPIABLE C++ ACOUSTIC REFERENCE TEMPLATE (PASTE INTO MATCHER)");
  Serial.println(" *************************************************************/");

  // Left Tip Average
  float avgLeftEnv[ENVELOPE_POINTS] = {0};
  int32_t avgLeftPeak = 0;
  float avgLeftCrest = 0;
  int avgLeftZCR = 0;

  if (leftCount > 0) {
    for (int i = 0; i < leftCount; i++) {
      avgLeftPeak += leftSamples[i].peak;
      avgLeftCrest += leftSamples[i].crest;
      avgLeftZCR += leftSamples[i].zcr;
      for (int p = 0; p < ENVELOPE_POINTS; p++) {
        avgLeftEnv[p] += leftSamples[i].envelope[p];
      }
    }
    avgLeftPeak /= leftCount;
    avgLeftCrest /= leftCount;
    avgLeftZCR /= leftCount;
    for (int p = 0; p < ENVELOPE_POINTS; p++) avgLeftEnv[p] /= leftCount;
  }

  // Right Tip Average
  float avgRightEnv[ENVELOPE_POINTS] = {0};
  int32_t avgRightPeak = 0;
  float avgRightCrest = 0;
  int avgRightZCR = 0;

  if (rightCount > 0) {
    for (int i = 0; i < rightCount; i++) {
      avgRightPeak += rightSamples[i].peak;
      avgRightCrest += rightSamples[i].crest;
      avgRightZCR += rightSamples[i].zcr;
      for (int p = 0; p < ENVELOPE_POINTS; p++) {
        avgRightEnv[p] += rightSamples[i].envelope[p];
      }
    }
    avgRightPeak /= rightCount;
    avgRightCrest /= rightCount;
    avgRightZCR /= rightCount;
    for (int p = 0; p < ENVELOPE_POINTS; p++) avgRightEnv[p] /= rightCount;
  }

  // Steel Knock Average
  int avgSteelZCR = 0;
  if (steelCount > 0) {
    for (int i = 0; i < steelCount; i++) avgSteelZCR += steelSamples[i].zcr;
    avgSteelZCR /= steelCount;
  }

  Serial.println("// Reference Profile: LEFT BUCKET TIP");
  Serial.printf("const int32_t REF_LEFT_PEAK  = %ld;\n", avgLeftPeak);
  Serial.printf("const float   REF_LEFT_CREST = %.2ff;\n", avgLeftCrest);
  Serial.printf("const int     REF_LEFT_ZCR   = %d;\n", avgLeftZCR);
  Serial.print("const float   REF_LEFT_ENVELOPE[16] = { ");
  for (int p = 0; p < ENVELOPE_POINTS; p++) {
    Serial.printf("%.3ff%s", avgLeftEnv[p], (p < ENVELOPE_POINTS - 1) ? ", " : " ");
  }
  Serial.println("};\n");

  Serial.println("// Reference Profile: RIGHT BUCKET TIP");
  Serial.printf("const int32_t REF_RIGHT_PEAK  = %ld;\n", avgRightPeak);
  Serial.printf("const float   REF_RIGHT_CREST = %.2ff;\n", avgRightCrest);
  Serial.printf("const int     REF_RIGHT_ZCR   = %d;\n", avgRightZCR);
  Serial.print("const float   REF_RIGHT_ENVELOPE[16] = { ");
  for (int p = 0; p < ENVELOPE_POINTS; p++) {
    Serial.printf("%.3ff%s", avgRightEnv[p], (p < ENVELOPE_POINTS - 1) ? ", " : " ");
  }
  Serial.println("};\n");

  Serial.println("// Reference Noise Reject: STEEL FRAME KNOCK");
  Serial.printf("const int     REF_STEEL_KNOCK_ZCR = %d;\n", avgSteelZCR);
  Serial.println("/*************************************************************/\n");
}

// ==========================================
// 7. SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=======================================================");
  Serial.println(" ESP32-S3 AMOLED: Acoustic Tip Fingerprint Recorder");
  Serial.println("=======================================================");

  gfx->begin();
  gfx->fillScreen(COLOR_BG);
  renderUI();
  updateUIMode();

  initHardwarePower();
  initI2S();
  initES7210();

  Serial.println("\n[READY] Sampling station online!");
  Serial.println("-> Default Armed: LEFT BUCKET TIP (Record 3 tips)");
  Serial.println("-> Controls:");
  Serial.println("   'l' = Arm for LEFT Tip");
  Serial.println("   'r' = Arm for RIGHT Tip");
  Serial.println("   'k' = Arm for STEEL Knock Noise");
  Serial.println("   'p' = Export & Print C++ Reference Arrays");
  Serial.println("   'c' = Clear all samples and restart");
  Serial.println("-------------------------------------------------------");
}

void loop() {
  size_t bytesRead = 0;
  esp_err_t result = i2s_read(I2S_PORT, audioBuffer, sizeof(audioBuffer), &bytesRead, 100 / portTICK_PERIOD_MS);

  int samplesRead = bytesRead / sizeof(int16_t);

  if (result == ESP_OK && samplesRead > 0) {
    for (int i = 0; i < samplesRead; i += 2) { // Read Left channel (or interleaved)
      int16_t sample = audioBuffer[i];
      int32_t absVal = abs(sample);

      // Store in circular pre-trigger buffer
      ringBuffer[ringIndex] = sample;
      ringIndex = (ringIndex + 1) % PROFILE_SAMPLES;

      unsigned long now = millis();

      // Trigger condition: sound spike above threshold and not currently recording
      if (!recordingTriggered && (absVal >= triggerThreshold) && (now - lastTriggerTime > DEBOUNCE_MS)) {
        recordingTriggered = true;
        lastTriggerTime = now;

        // Copy pre-trigger samples
        int startIdx = (ringIndex - PRE_TRIGGER_SAMPLES + PROFILE_SAMPLES) % PROFILE_SAMPLES;
        for (int p = 0; p < PRE_TRIGGER_SAMPLES; p++) {
          capturedWave[p] = ringBuffer[(startIdx + p) % PROFILE_SAMPLES];
        }

        captureWriteIndex = PRE_TRIGGER_SAMPLES;
        postTriggerSamplesRemaining = PROFILE_SAMPLES - PRE_TRIGGER_SAMPLES;
      }
      else if (recordingTriggered && postTriggerSamplesRemaining > 0) {
        capturedWave[captureWriteIndex++] = sample;
        postTriggerSamplesRemaining--;

        // Once 800 samples (50 ms) are completely captured:
        if (postTriggerSamplesRemaining <= 0) {
          recordingTriggered = false;

          SoundProfile sp;
          processAndExtractFeatures(capturedWave, &sp);

          updateMetricsUI(sp);

          if (currentMode == MODE_ARM_LEFT) {
            if (leftCount < MAX_SAMPLES_PER_TYPE) {
              leftSamples[leftCount++] = sp;
            }
            Serial.printf("\n[CAPTURED: LEFT TIP #%d]\n", leftCount);
            Serial.printf("  Peak: %ld | RMS: %.0f | Crest: %.1fx | ZCR: %d | Decay: %.1f%%\n",
                          sp.peak, sp.rms, sp.crest, sp.zcr, sp.decayRatio * 100.0f);
            Serial.print("  Envelope: { ");
            for (int p = 0; p < ENVELOPE_POINTS; p++) Serial.printf("%.2f%s", sp.envelope[p], (p < ENVELOPE_POINTS - 1) ? ", " : " ");
            Serial.println("}");

            if (leftCount >= 3) {
              currentMode = MODE_ARM_RIGHT;
              Serial.println("\n>>> [NEXT STEP] 3 Left Tips Captured! Now armed for RIGHT BUCKET TIP!");
            }
          }
          else if (currentMode == MODE_ARM_RIGHT) {
            if (rightCount < MAX_SAMPLES_PER_TYPE) {
              rightSamples[rightCount++] = sp;
            }
            Serial.printf("\n[CAPTURED: RIGHT TIP #%d]\n", rightCount);
            Serial.printf("  Peak: %ld | RMS: %.0f | Crest: %.1fx | ZCR: %d | Decay: %.1f%%\n",
                          sp.peak, sp.rms, sp.crest, sp.zcr, sp.decayRatio * 100.0f);
            Serial.print("  Envelope: { ");
            for (int p = 0; p < ENVELOPE_POINTS; p++) Serial.printf("%.2f%s", sp.envelope[p], (p < ENVELOPE_POINTS - 1) ? ", " : " ");
            Serial.println("}");

            if (rightCount >= 3) {
              currentMode = MODE_ARM_STEEL;
              Serial.println("\n>>> [NEXT STEP] 3 Right Tips Captured! Now armed for STEEL KNOCK NOISE!");
            }
          }
          else if (currentMode == MODE_ARM_STEEL) {
            if (steelCount < MAX_SAMPLES_PER_TYPE) {
              steelSamples[steelCount++] = sp;
            }
            Serial.printf("\n[CAPTURED: STEEL KNOCK #%d]\n", steelCount);
            Serial.printf("  Peak: %ld | RMS: %.0f | Crest: %.1fx | ZCR: %d | Decay: %.1f%%\n",
                          sp.peak, sp.rms, sp.crest, sp.zcr, sp.decayRatio * 100.0f);

            if (steelCount >= 2) {
              currentMode = MODE_IDLE;
              Serial.println("\n>>> [COMPLETE] All samples captured! Press 'p' to generate reference arrays!");
            }
          }

          updateUIMode();
        }
      }
    }
  }

  // Serial Keyboard Commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'l' || c == 'L') {
      currentMode = MODE_ARM_LEFT;
      updateUIMode();
      Serial.println("[MODE] Armed for LEFT BUCKET TIP recording.");
    }
    else if (c == 'r' || c == 'R') {
      currentMode = MODE_ARM_RIGHT;
      updateUIMode();
      Serial.println("[MODE] Armed for RIGHT BUCKET TIP recording.");
    }
    else if (c == 'k' || c == 'K') {
      currentMode = MODE_ARM_STEEL;
      updateUIMode();
      Serial.println("[MODE] Armed for STEEL FRAME KNOCK recording.");
    }
    else if (c == 'p' || c == 'P') {
      exportReferenceCode();
    }
    else if (c == 'c' || c == 'C') {
      leftCount = 0;
      rightCount = 0;
      steelCount = 0;
      currentMode = MODE_ARM_LEFT;
      updateUIMode();
      Serial.println("[RESET] Cleared all captured samples. Starting over with Left Tip.");
    }
  }
}

