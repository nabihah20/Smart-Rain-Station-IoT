/*
 * ==============================================================================
 * ESP32-S3 Touch AMOLED 1.75 - Acoustic Tip Fingerprint Recorder & Profiler
 * ==============================================================================
 * Project: Smart Rain Station IoT (Workstream 05: Reference Fingerprint Sampling)
 * Target Hardware: Waveshare ESP32-S3 Touch AMOLED 1.75
 * Codec / ADC: Onboard ES7210 4-Channel Audio ADC + Dual MEMS Microphones
 * Display: 466x466 Round CO5300 QSPI AMOLED (GFX_Library_for_Arduino)
 *
 * WORKFLOW:
 * 1. User starts each sample manually: Send 's' (or spacebar/Enter) in Serial.
 * 2. 10-Second Countdown Window opens:
 *    - The AMOLED displays a live 10-second timer and countdown bar.
 *    - User tips the bucket comfortably at their own pace.
 *    - The recorder captures the exact 50 ms impulse window.
 * 3. Review & Accept:
 *    - Displays exact Peak, Crest, ZCR, and Decay on the AMOLED and Serial Monitor.
 *    - Press 's' to ACCEPT and move to next sample.
 *    - Press 'r' to RETRY if there was an accidental noise or mistake.
 * 4. Export:
 *    - Automatically generates and prints C++ reference templates!
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
#define COLOR_EMERALD       0x170A // Success / Active (Green)
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

// Sensitivity threshold for capturing an impact during the 10s window (Mechanical bucket clacks peak at 8000-28000; default set to 4000 to reject water drops ~1500 and ambient noise ~500)
int32_t triggerThreshold = 4000;
int32_t liveWindowMaxPeak = 0; // Tracks the loudest peak heard during active 10s window
int32_t liveBatchPeak = 0;     // Instantaneous peak in current batch

static int16_t audioBuffer[SAMPLES_PER_READ * 2]; // 16-bit stereo
static int16_t ringBuffer[PROFILE_SAMPLES];       // Pre-trigger ring buffer
static int ringIndex = 0;
static int16_t capturedWave[PROFILE_SAMPLES];

struct SoundProfile {
  int32_t peak;
  float   rms;
  float   crest;
  int     zcr;
  float   decayRatio;
  float   envelope[ENVELOPE_POINTS];
  float   spectrum[SPECTRAL_BANDS];
};

#define MAX_SAMPLES_PER_TYPE 3
SoundProfile leftSamples[MAX_SAMPLES_PER_TYPE];
SoundProfile rightSamples[MAX_SAMPLES_PER_TYPE];
SoundProfile steelSamples[2];

int leftCount  = 0;
int rightCount = 0;
int steelCount = 0;

SoundProfile tempProfile; // Holds the most recently captured profile awaiting confirmation

// Target categories
enum TargetType {
  TARGET_LEFT_TIP,
  TARGET_RIGHT_TIP,
  TARGET_STEEL_KNOCK,
  TARGET_ALL_COMPLETE
};

TargetType currentTarget = TARGET_LEFT_TIP;

// Interactive 10-Second Window State Machine
enum SessionState {
  STATE_WAIT_FOR_START, // Waiting for user to send 's' to start countdown
  STATE_LISTENING_10S,  // Active 10-second countdown listening for the impact
  STATE_SAMPLE_REVIEW   // Impact captured! Showing metrics, waiting for 's' (accept) or 'r' (retry)
};

SessionState sessionState = STATE_WAIT_FOR_START;
unsigned long windowStartTime = 0;
const unsigned long WINDOW_MS = 10000; // 10 seconds
int lastCountdownSec = -1;

bool recordingTriggered = false;
int  postTriggerSamplesRemaining = 0;
int  captureWriteIndex = 0;

// UI Dirty flags
bool needUiFullRedraw = true;

// ==========================================
// 3. I2C & HARDWARE INITIALIZATION
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

  // AXP2101 power rails
  writeI2CReg(AXP2101_ADDR, 0x90, 0xBF);
  writeI2CReg(AXP2101_ADDR, 0x91, 0xFF);
  writeI2CReg(AXP2101_ADDR, 0x92, 0x1F); // ALDO1=3.3V
  writeI2CReg(AXP2101_ADDR, 0x93, 0x1F); // ALDO2=3.3V
  writeI2CReg(AXP2101_ADDR, 0x94, 0x1F); // ALDO3=3.3V
  writeI2CReg(AXP2101_ADDR, 0x95, 0x1F); // ALDO4=3.3V

  // TCA9554
  writeI2CReg(TCA9554_ADDR, 0x03, 0x00);
  writeI2CReg(TCA9554_ADDR, 0x01, 0xFF);

  pinMode(PA_EN_PIN, OUTPUT);
  digitalWrite(PA_EN_PIN, LOW); // Mute speaker
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
  writeI2CReg(ES7210_ADDR, 0x00, 0xFF);
  delay(20);
  writeI2CReg(ES7210_ADDR, 0x00, 0x41);
  writeI2CReg(ES7210_ADDR, 0x01, 0x1F);
  writeI2CReg(ES7210_ADDR, 0x06, 0x04);
  writeI2CReg(ES7210_ADDR, 0x07, 0x00);
  writeI2CReg(ES7210_ADDR, 0x08, 0x10);

  // Digital I2S 16-bit
  writeI2CReg(ES7210_ADDR, 0x11, 0x60);
  writeI2CReg(ES7210_ADDR, 0x12, 0x02);

  // Un-mute Analog Bias & PGAs
  writeI2CReg(ES7210_ADDR, 0x40, 0x42);
  writeI2CReg(ES7210_ADDR, 0x41, 0x70);
  writeI2CReg(ES7210_ADDR, 0x42, 0x70);

  // Maximum gain (+37.5 dB) on both front microphones
  writeI2CReg(ES7210_ADDR, 0x43, 0x1E);
  writeI2CReg(ES7210_ADDR, 0x44, 0x1E);

  writeI2CReg(ES7210_ADDR, 0x4B, 0x00);
  writeI2CReg(ES7210_ADDR, 0x4C, 0x00);
  writeI2CReg(ES7210_ADDR, 0x14, 0x03);
  writeI2CReg(ES7210_ADDR, 0x06, 0x00);
  writeI2CReg(ES7210_ADDR, 0x01, 0x00);

  writeI2CReg(ES7210_ADDR, 0x00, 0x71);
  delay(10);
  writeI2CReg(ES7210_ADDR, 0x00, 0x41);

  Serial.println("[ES7210] High-Gain Microphones Configured!");
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

  out->peak  = peakVal;
  out->rms   = sqrt((float)sumSq / PROFILE_SAMPLES);
  out->crest = (out->rms > 10.0f) ? ((float)out->peak / out->rms) : 0.0f;
  out->zcr   = zcr;

  // 16-point Normalized Energy Envelope (50 samples per point)
  int samplesPerPoint = PROFILE_SAMPLES / ENVELOPE_POINTS;
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

  if (maxEnv > 0.0f) {
    for (int p = 0; p < ENVELOPE_POINTS; p++) {
      out->envelope[p] /= maxEnv;
    }
  }

  out->decayRatio = (out->envelope[8] > 0.0f) ? out->envelope[8] : 0.0f;
}

// ==========================================
// 5. AMOLED UI RENDERING
// ==========================================
void renderBaseUI() {
  gfx->fillScreen(COLOR_BG);

  // Outer circles
  gfx->drawCircle(233, 233, 230, COLOR_BORDER);
  gfx->drawCircle(233, 233, 229, COLOR_TRACK);

  // Header Title
  gfx->setTextColor(COLOR_CYAN, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(75, 38);
  gfx->print("TIP SOUND PROFILER");

  gfx->setTextColor(COLOR_TEXT_MUTED, COLOR_BG);
  gfx->setTextSize(1);
  gfx->setCursor(140, 62);
  gfx->print("10-SECOND CONTROLLED SAMPLER");

  // Center Interactive Card Box
  gfx->drawRoundRect(35, 85, 396, 115, 16, COLOR_TRACK);

  // Sample Progress Counters
  // Left
  gfx->drawRoundRect(35, 215, 125, 80, 12, COLOR_TRACK);
  gfx->setTextColor(COLOR_CYAN, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(55, 225);
  gfx->print("LEFT");

  // Right
  gfx->drawRoundRect(170, 215, 125, 80, 12, COLOR_TRACK);
  gfx->setTextColor(COLOR_AMBER, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(185, 225);
  gfx->print("RIGHT");

  // Steel
  gfx->drawRoundRect(305, 215, 125, 80, 12, COLOR_TRACK);
  gfx->setTextColor(COLOR_CRIMSON, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(320, 225);
  gfx->print("STEEL");

  // Metrics Labels
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

  // Footer Instructions
  gfx->setCursor(35, 410);
  gfx->setTextSize(1);
  gfx->setTextColor(COLOR_CYAN, COLOR_BG);
  gfx->print("[s: START / ACCEPT | r: RETRY | p: EXPORT]");
}

void updateStatusCards() {
  gfx->setTextSize(3);
  char buf[8];

  // Left (x=55, y=252)
  gfx->fillRect(40, 250, 115, 36, COLOR_BG);
  gfx->setTextColor(COLOR_CYAN, COLOR_BG);
  snprintf(buf, sizeof(buf), "%d/3", leftCount);
  gfx->setCursor(65, 252);
  gfx->print(buf);

  // Right (x=190, y=252)
  gfx->fillRect(175, 250, 115, 36, COLOR_BG);
  gfx->setTextColor(COLOR_AMBER, COLOR_BG);
  snprintf(buf, sizeof(buf), "%d/3", rightCount);
  gfx->setCursor(200, 252);
  gfx->print(buf);

  // Steel (x=325, y=252)
  gfx->fillRect(310, 250, 115, 36, COLOR_BG);
  gfx->setTextColor(COLOR_CRIMSON, COLOR_BG);
  snprintf(buf, sizeof(buf), "%d/2", steelCount);
  gfx->setCursor(335, 252);
  gfx->print(buf);
}

void updateMetricsDisplay(const SoundProfile& sp) {
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

// Center Interactive Box Display
void updateInteractiveBox() {
  gfx->fillRect(40, 90, 386, 105, COLOR_BG);

  if (sessionState == STATE_WAIT_FOR_START) {
    uint16_t targetColor = COLOR_CYAN;
    const char* targetName = "LEFT TIP";
    int currentSampleNum = leftCount + 1;
    int totalSampleNum   = 3;

    if (currentTarget == TARGET_LEFT_TIP) {
      targetColor = COLOR_CYAN;
      targetName  = "LEFT TIP";
      currentSampleNum = leftCount + 1;
      totalSampleNum   = 3;
    } else if (currentTarget == TARGET_RIGHT_TIP) {
      targetColor = COLOR_AMBER;
      targetName  = "RIGHT TIP";
      currentSampleNum = rightCount + 1;
      totalSampleNum   = 3;
    } else if (currentTarget == TARGET_STEEL_KNOCK) {
      targetColor = COLOR_CRIMSON;
      targetName  = "STEEL KNOCK";
      currentSampleNum = steelCount + 1;
      totalSampleNum   = 2;
    } else {
      // All complete
      gfx->fillRoundRect(48, 96, 370, 40, 8, COLOR_EMERALD);
      gfx->setTextColor(COLOR_WHITE, COLOR_EMERALD);
      gfx->setTextSize(2);
      gfx->setCursor(110, 108);
      gfx->print("ALL SAMPLES READY!");

      gfx->setTextColor(COLOR_WHITE, COLOR_BG);
      gfx->setTextSize(2);
      gfx->setCursor(65, 150);
      gfx->print("Press 'p' in Serial to export");
      return;
    }

    // Top pill: Ready to start
    gfx->fillRoundRect(48, 96, 370, 38, 8, targetColor);
    gfx->setTextColor(COLOR_WHITE, targetColor);
    gfx->setTextSize(2);
    char titleBuf[32];
    snprintf(titleBuf, sizeof(titleBuf), "%s (Sample %d/%d)", targetName, currentSampleNum, totalSampleNum);
    int tLen = strlen(titleBuf) * 12;
    gfx->setCursor(48 + (370 - tLen) / 2, 106);
    gfx->print(titleBuf);

    // Call to action
    gfx->setTextColor(COLOR_WHITE, COLOR_BG);
    gfx->setTextSize(2);
    gfx->setCursor(80, 145);
    gfx->print("SEND 's' TO START");

    gfx->setTextColor(COLOR_TEXT_MUTED, COLOR_BG);
    gfx->setTextSize(1);
    gfx->setCursor(120, 172);
    gfx->print("Opens 10-second capture window");
  }
  else if (sessionState == STATE_LISTENING_10S) {
    // 10-Second Listening Active
    gfx->fillRoundRect(48, 96, 370, 38, 8, COLOR_EMERALD);
    gfx->setTextColor(COLOR_WHITE, COLOR_EMERALD);
    gfx->setTextSize(2);
    gfx->setCursor(75, 106);
    gfx->print("RECORDING... TIP NOW!");

    // Big digital seconds countdown
    unsigned long elapsed = millis() - windowStartTime;
    int secLeft = 10 - (elapsed / 1000);
    if (secLeft < 0) secLeft = 0;

    gfx->setTextColor(COLOR_WHITE, COLOR_BG);
    gfx->setTextSize(3);
    char secBuf[16];
    snprintf(secBuf, sizeof(secBuf), "%ds REMAINING", secLeft);
    int sLen = strlen(secBuf) * 18;
    gfx->setCursor(48 + (370 - sLen) / 2, 145);
    gfx->print(secBuf);

    // Shrinking Countdown Progress Bar
    int barW = map(constrain(10000 - elapsed, 0, 10000), 0, 10000, 0, 350);
    gfx->drawRoundRect(55, 175, 356, 12, 4, COLOR_TRACK);
    gfx->fillRect(58, 178, barW, 6, COLOR_EMERALD);
  }
  else if (sessionState == STATE_SAMPLE_REVIEW) {
    // Sample Captured! Showing review options
    gfx->fillRoundRect(48, 96, 370, 38, 8, COLOR_EMERALD);
    gfx->setTextColor(COLOR_WHITE, COLOR_EMERALD);
    gfx->setTextSize(2);
    gfx->setCursor(105, 106);
    gfx->print("SAMPLE CAPTURED!");

    // Review Actions
    gfx->setTextColor(COLOR_CYAN, COLOR_BG);
    gfx->setTextSize(2);
    gfx->setCursor(65, 142);
    gfx->print("[s] ACCEPT & NEXT");

    gfx->setTextColor(COLOR_AMBER, COLOR_BG);
    gfx->setCursor(65, 168);
    gfx->print("[r] RETRY THIS SAMPLE");
  }
}

// ==========================================
// 6. EXPORT C++ REFERENCE CODE
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
  int32_t avgSteelPeak = 0;
  if (steelCount > 0) {
    for (int i = 0; i < steelCount; i++) {
      avgSteelZCR += steelSamples[i].zcr;
      avgSteelPeak += steelSamples[i].peak;
    }
    avgSteelZCR /= steelCount;
    avgSteelPeak /= steelCount;
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

  Serial.println("// Reference Noise: STEEL FRAME KNOCK (TO REJECT)");
  Serial.printf("const int     REF_STEEL_KNOCK_ZCR  = %d;\n", avgSteelZCR);
  Serial.printf("const int32_t REF_STEEL_KNOCK_PEAK = %ld;\n", avgSteelPeak);
  Serial.println("/*************************************************************/\n");
}

// ==========================================
// 7. SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=======================================================");
  Serial.println(" ESP32-S3 AMOLED: 10-Second Controlled Tip Profiler");
  Serial.println("=======================================================");

  gfx->begin();
  renderBaseUI();
  updateStatusCards();
  updateInteractiveBox();

  initHardwarePower();
  initI2S();
  initES7210();

  Serial.println("\n[READY] Interactive Sampling Station Online!");
  Serial.println("-------------------------------------------------------");
  Serial.println("HOW TO USE:");
  Serial.println("1. When ready, type 's' and press Enter in Serial.");
  Serial.println("2. You will have 10 SECONDS to tip the bucket.");
  Serial.println("3. The system captures the sound and pauses.");
  Serial.println("4. Type 's' to ACCEPT or 'r' to RETRY that sample.");
  Serial.println("-------------------------------------------------------");
  Serial.println("[START] Target 1: LEFT BUCKET TIP (Sample 1 of 3)");
  Serial.println("-> Type 's' and press Enter whenever you are ready to tip!");
}

void loop() {
  size_t bytesRead = 0;
  esp_err_t result = i2s_read(I2S_PORT, audioBuffer, sizeof(audioBuffer), &bytesRead, 100 / portTICK_PERIOD_MS);
  int samplesRead = bytesRead / sizeof(int16_t);

  unsigned long now = millis();

  // ========================================================
  // 10-SECOND LISTENING WINDOW HANDLER
  // ========================================================
  if (sessionState == STATE_LISTENING_10S) {
    unsigned long elapsed = now - windowStartTime;

    // Countdown timer display update (every 250ms)
    static unsigned long lastUiTick = 0;
    if (now - lastUiTick >= 200) {
      lastUiTick = now;
      updateInteractiveBox();
    }

    // Print countdown second tick in Serial Monitor with live sound level telemetry
    int secLeft = 10 - (elapsed / 1000);
    if (secLeft != lastCountdownSec && secLeft >= 0) {
      lastCountdownSec = secLeft;
      Serial.printf("[LISTENING] %2d sec left | Live Peak: %5ld | Max Heard: %5ld | Thresh: %5ld\n",
                    secLeft, liveBatchPeak, liveWindowMaxPeak, triggerThreshold);
    }

    // Check for Timeout
    if (elapsed >= WINDOW_MS && !recordingTriggered) {
      sessionState = STATE_WAIT_FOR_START;
      lastCountdownSec = -1;
      updateInteractiveBox();
      Serial.println("\n[TIMEOUT] 10 seconds expired with no trigger reached.");
      Serial.printf("  Loudest sound heard in window was Peak: %ld (Threshold is %ld)\n", liveWindowMaxPeak, triggerThreshold);
      if (liveWindowMaxPeak > 300 && liveWindowMaxPeak < triggerThreshold) {
        Serial.printf("  -> TIP: The sound reached %ld! You can lower threshold by pressing '-' or retry.\n", liveWindowMaxPeak);
      }
      Serial.println("-> Type 's' and press Enter to try this sample again.");
    }
  }

  // ========================================================
  // AUDIO CAPTURE ENGINE (DUAL-MIC COMBINED STEREO)
  // ========================================================
  if (result == ESP_OK && samplesRead > 0) {
    liveBatchPeak = 0;
    for (int i = 0; i < samplesRead; i += 2) {
      int16_t mic1 = audioBuffer[i];     // Channel 0 (MIC1)
      int16_t mic2 = audioBuffer[i + 1]; // Channel 1 (MIC2)

      // Take the louder channel to guarantee both left and right bucket impacts are heard
      int16_t sample = (abs(mic1) >= abs(mic2)) ? mic1 : mic2;
      int32_t absVal = abs(sample);

      if (absVal > liveBatchPeak) liveBatchPeak = absVal;

      // Track peak during 10s listening window
      if (sessionState == STATE_LISTENING_10S) {
        if (absVal > liveWindowMaxPeak) liveWindowMaxPeak = absVal;
      }

      // Store in circular ring buffer
      ringBuffer[ringIndex] = sample;
      ringIndex = (ringIndex + 1) % PROFILE_SAMPLES;

      // Only trigger if we are actively in the 10-second listening window
      if (sessionState == STATE_LISTENING_10S) {
        if (!recordingTriggered && (absVal >= triggerThreshold)) {
          recordingTriggered = true;

          // Copy 100 samples before trigger point
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

          // Complete 50 ms (800 samples) captured!
          if (postTriggerSamplesRemaining <= 0) {
            recordingTriggered = false;

            // Extract features
            processAndExtractFeatures(capturedWave, &tempProfile);

            sessionState = STATE_SAMPLE_REVIEW;
            lastCountdownSec = -1;

            updateMetricsDisplay(tempProfile);
            updateInteractiveBox();

            // Print detailed review in Serial Monitor
            const char* targetName = (currentTarget == TARGET_LEFT_TIP) ? "LEFT TIP" :
                                     (currentTarget == TARGET_RIGHT_TIP) ? "RIGHT TIP" : "STEEL KNOCK";
            int currentNum = (currentTarget == TARGET_LEFT_TIP) ? (leftCount + 1) :
                             (currentTarget == TARGET_RIGHT_TIP) ? (rightCount + 1) : (steelCount + 1);

            Serial.println("\n-------------------------------------------------------");
            Serial.printf("[CAPTURED: %s (Sample %d)]\n", targetName, currentNum);
            Serial.printf("  Peak: %ld | RMS: %.0f | Crest: %.1fx | ZCR: %d | Decay: %.1f%%\n",
                          tempProfile.peak, tempProfile.rms, tempProfile.crest, tempProfile.zcr, tempProfile.decayRatio * 100.0f);
            Serial.print("  Envelope: { ");
            for (int p = 0; p < ENVELOPE_POINTS; p++) Serial.printf("%.2f%s", tempProfile.envelope[p], (p < ENVELOPE_POINTS - 1) ? ", " : " ");
            Serial.println("}");
            Serial.println("-------------------------------------------------------");
            Serial.println("OPTIONS:");
            Serial.println("-> Type 's' to ACCEPT this sample and proceed to next.");
            Serial.println("-> Type 'r' to RETRY this sample if it was a mistake.");
          }
        }
      }
    }
  }

  // ========================================================
  // SERIAL KEYBOARD COMMANDS
  // ========================================================
  while (Serial.available()) {
    char c = Serial.read();

    // Ignore newline and carriage return characters from terminal enter key
    if (c == '\r' || c == '\n') continue;

    // 's' or 'S' to START countdown or ACCEPT sample
    if (c == 's' || c == 'S') {
      if (sessionState == STATE_WAIT_FOR_START) {
        if (currentTarget != TARGET_ALL_COMPLETE) {
          sessionState = STATE_LISTENING_10S;
          windowStartTime = millis();
          recordingTriggered = false;
          liveWindowMaxPeak = 0;
          lastCountdownSec = -1;
          updateInteractiveBox();

          const char* targetName = (currentTarget == TARGET_LEFT_TIP) ? "LEFT TIP" :
                                   (currentTarget == TARGET_RIGHT_TIP) ? "RIGHT TIP" : "STEEL KNOCK";
          int num = (currentTarget == TARGET_LEFT_TIP) ? (leftCount + 1) :
                    (currentTarget == TARGET_RIGHT_TIP) ? (rightCount + 1) : (steelCount + 1);

          Serial.println("\n>>> [10-SECOND WINDOW STARTED!] <<<");
          Serial.printf(">>> TIP %s NOW! (Sample %d) <<<\n", targetName, num);
        }
      }
      else if (sessionState == STATE_SAMPLE_REVIEW) {
        // Accept sample and commit it
        if (currentTarget == TARGET_LEFT_TIP) {
          leftSamples[leftCount++] = tempProfile;
          Serial.printf("[ACCEPTED] Left Tip Sample %d of 3 saved.\n", leftCount);
          if (leftCount >= 3) {
            currentTarget = TARGET_RIGHT_TIP;
            Serial.println("\n=======================================================");
            Serial.println("[COMPLETE] All 3 Left Tip samples recorded!");
            Serial.println("Next target: RIGHT BUCKET TIP (Sample 1 of 3)");
            Serial.println("-> Type 's' when ready to tip the RIGHT bucket.");
            Serial.println("=======================================================");
          }
        }
        else if (currentTarget == TARGET_RIGHT_TIP) {
          rightSamples[rightCount++] = tempProfile;
          Serial.printf("[ACCEPTED] Right Tip Sample %d of 3 saved.\n", rightCount);
          if (rightCount >= 3) {
            currentTarget = TARGET_STEEL_KNOCK;
            Serial.println("\n=======================================================");
            Serial.println("[COMPLETE] All 3 Right Tip samples recorded!");
            Serial.println("Next target: STEEL FRAME KNOCK (Sample 1 of 2)");
            Serial.println("-> Type 's' when ready to tap the steel frame.");
            Serial.println("=======================================================");
          }
        }
        else if (currentTarget == TARGET_STEEL_KNOCK) {
          steelSamples[steelCount++] = tempProfile;
          Serial.printf("[ACCEPTED] Steel Knock Sample %d of 2 saved.\n", steelCount);
          if (steelCount >= 2) {
            currentTarget = TARGET_ALL_COMPLETE;
            Serial.println("\n=======================================================");
            Serial.println("[ALL RECORDINGS COMPLETE!] Generating Reference Template...");
            Serial.println("=======================================================");
            exportReferenceCode();
          }
        }

        updateStatusCards();
        sessionState = STATE_WAIT_FOR_START;
        updateInteractiveBox();
      }
    }
    // '+' or '=' to INCREASE sensitivity threshold
    else if (c == '+' || c == '=') {
      triggerThreshold += 250;
      Serial.printf("[CONFIG] Trigger threshold increased to: %ld\n", triggerThreshold);
    }
    // '-' or '_' to DECREASE sensitivity threshold
    else if (c == '-' || c == '_') {
      if (triggerThreshold > 300) triggerThreshold -= 250;
      Serial.printf("[CONFIG] Trigger threshold decreased to: %ld\n", triggerThreshold);
    }
    // 'r' or 'R' to RETRY the current sample
    else if (c == 'r' || c == 'R') {
      if (sessionState == STATE_SAMPLE_REVIEW) {
        sessionState = STATE_WAIT_FOR_START;
        updateInteractiveBox();
        Serial.println("[RETRY] Discarded last sample. Ready to re-take.");
        Serial.println("-> Type 's' whenever you are ready to start 10s window!");
      }
    }
    // 'p' or 'P' to EXPORT reference templates at any time
    else if (c == 'p' || c == 'P') {
      exportReferenceCode();
    }
    // 'c' or 'C' to RESET and start completely over
    else if (c == 'c' || c == 'C') {
      leftCount = 0;
      rightCount = 0;
      steelCount = 0;
      currentTarget = TARGET_LEFT_TIP;
      sessionState = STATE_WAIT_FOR_START;
      renderBaseUI();
      updateStatusCards();
      updateInteractiveBox();
      Serial.println("\n[RESET] Cleared all samples! Starting over with Left Tip 1/3.");
      Serial.println("-> Type 's' whenever you are ready!");
    }
  }
}
