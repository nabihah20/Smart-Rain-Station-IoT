/*
 * Project: ESP32-S3 AMOLED - Edge Impulse Audio Data Collector
 * Board: Waveshare ESP32-S3 Touch AMOLED 1.75
 * Purpose: Stream continuous 16 kHz 16-bit mono PCM audio over USB Serial
 *          to PC Python script for automatic .WAV dataset generation.
 */

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <Arduino_GFX_Library.h>

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

#define IIC_SDA     15
#define IIC_SCL     14

// ES7210 Dual Microphone
#define ES7210_ADDR 0x41
#define MIC_I2S_PORT I2S_NUM_1
#define MIC_MCLK    16
#define MIC_BCLK    17
#define MIC_WS      47
#define MIC_DIN     48

// Display Bus
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3
);
Arduino_CO5300 *gfx = new Arduino_CO5300(
  bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0
);

// Colors
#define COLOR_BG      0x0000
#define COLOR_TEXT    0xFFFF
#define COLOR_CYAN    0x3DFE
#define COLOR_AMBER   0xFD20
#define COLOR_EMERALD 0x170A

// Audio Parameters
#define SAMPLE_RATE      16000
#define SAMPLES_PER_READ 256

static int16_t audioBuffer[SAMPLES_PER_READ * 2]; // 16-bit stereo from ES7210
static int16_t monoBuffer[SAMPLES_PER_READ];      // 16-bit merged mono for Edge Impulse

bool streamingActive = false;

// ==========================================
// 2. ES7210 & I2S INITIALIZATION
// ==========================================
bool writeI2CReg(uint8_t devAddr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

void initES7210() {
  writeI2CReg(ES7210_ADDR, 0x01, 0x3F); delay(10);
  writeI2CReg(ES7210_ADDR, 0x02, 0x00);
  writeI2CReg(ES7210_ADDR, 0x03, 0x04);
  writeI2CReg(ES7210_ADDR, 0x04, 0x03);
  writeI2CReg(ES7210_ADDR, 0x05, 0x00);
  writeI2CReg(ES7210_ADDR, 0x06, 0x00);
  writeI2CReg(ES7210_ADDR, 0x07, 0x20);
  writeI2CReg(ES7210_ADDR, 0x08, 0x10);

  // Digital Volume
  writeI2CReg(ES7210_ADDR, 0x09, 0x00);
  writeI2CReg(ES7210_ADDR, 0x0A, 0x00);

  // Microphones 1 & 2 Gain: +24 dB (Optimal for mechanical tipping & knock distinction)
  writeI2CReg(ES7210_ADDR, 0x43, 0x18);
  writeI2CReg(ES7210_ADDR, 0x44, 0x18);
  writeI2CReg(ES7210_ADDR, 0x45, 0x00);
  writeI2CReg(ES7210_ADDR, 0x46, 0x00);
  writeI2CReg(ES7210_ADDR, 0x4B, 0x00);
  writeI2CReg(ES7210_ADDR, 0x4C, 0x00);

  writeI2CReg(ES7210_ADDR, 0x00, 0x01);
  writeI2CReg(ES7210_ADDR, 0x00, 0x41); delay(10);
}

void initI2SMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = SAMPLES_PER_READ,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .mck_io_num = MIC_MCLK,
    .bck_io_num = MIC_BCLK,
    .ws_io_num = MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = MIC_DIN
  };

  i2s_driver_install(MIC_I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(MIC_I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(MIC_I2S_PORT);
}

// ==========================================
// 3. UI RENDERING
// ==========================================
void renderUI(bool active) {
  gfx->fillScreen(COLOR_BG);

  gfx->setTextColor(COLOR_AMBER, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(85, 90);
  gfx->print("EDGE IMPULSE TINYML");

  gfx->setTextColor(COLOR_CYAN, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(75, 125);
  gfx->print("AUDIO DATA COLLECTOR");

  // Center status badge
  uint16_t badgeColor = active ? COLOR_EMERALD : 0x4208;
  gfx->fillRoundRect(70, 200, 326, 90, 16, badgeColor);

  gfx->setTextColor(COLOR_TEXT, badgeColor);
  gfx->setTextSize(3);
  if (active) {
    gfx->setCursor(115, 220);
    gfx->print("STREAMING");
    gfx->setTextSize(2);
    gfx->setCursor(130, 255);
    gfx->print("16kHz 16-bit");
  } else {
    gfx->setCursor(140, 220);
    gfx->print("STANDBY");
    gfx->setTextSize(2);
    gfx->setCursor(120, 255);
    gfx->print("Run Python Tool");
  }

  // Footer instructions
  gfx->setTextColor(0x8C71, COLOR_BG);
  gfx->setTextSize(2);
  gfx->setCursor(100, 360);
  gfx->print("Port: COM8 @ 921600");
  gfx->setCursor(85, 395);
  gfx->print("Ready for Tip & Knock");
}

// ==========================================
// SETUP & LOOP
// ==========================================
void setup() {
  // Use high baudrate for smooth binary audio streaming
  Serial.begin(921600);

  Wire.begin(IIC_SDA, IIC_SCL, 400000);
  delay(50);
  initES7210();
  initI2SMic();

  gfx->begin();
  renderUI(false);
}

void loop() {
  // Check for commands from Python logger tool
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'S') { // START STREAM
      streamingActive = true;
      renderUI(true);
      Serial.println("\n[ESP32_STREAM_START]");
    } else if (c == 'X') { // STOP STREAM
      streamingActive = false;
      renderUI(false);
      Serial.println("\n[ESP32_STREAM_STOP]");
    }
  }

  if (streamingActive) {
    size_t bytesRead = 0;
    esp_err_t res = i2s_read(MIC_I2S_PORT, audioBuffer, sizeof(audioBuffer), &bytesRead, 20 / portTICK_PERIOD_MS);

    if (res == ESP_OK && bytesRead > 0) {
      int samples = bytesRead / 4; // 4 bytes per stereo sample (2x 16-bit)

      for (int i = 0; i < samples; i++) {
        int16_t mic1 = audioBuffer[i * 2];
        int16_t mic2 = audioBuffer[i * 2 + 1];

        // Merge dual mics: take the louder channel so both left and right bucket impacts are captured faithfully
        monoBuffer[i] = (abs(mic1) >= abs(mic2)) ? mic1 : mic2;
      }

      // Stream raw 16-bit mono PCM binary to PC
      Serial.write((const uint8_t*)monoBuffer, samples * sizeof(int16_t));
    }
  } else {
    delay(10);
  }
}
