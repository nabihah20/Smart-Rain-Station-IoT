# 🌧️ Smart Rain Station IoT
### Intelligent Meteorological Telemetry, AMOLED Field Gateway & Cloud Forecasting

An end-to-end IoT meteorological telemetry, remote control, and AI-predictive rainfall system integrating the **Arduino Nano 33 BLE** (Physical Tipping-Bucket Simulator), the **Waveshare ESP32-S3 Touch AMOLED 1.75** (Field Station Gateway & Local Visual/Acoustic Display), and a **Real-Time Cloud Web Dashboard** powered by **Google Firebase Realtime Database**.

---

## 🏛️ System Architecture

```mermaid
flowchart TD
    subgraph Nano ["1. Arduino Nano 33 BLE (Rain Simulator)"]
        RainSim["Tipping-Bucket Simulator<br/>(126 drops = 1 tip = 0.20 mm)"]
        BLE_Tx["BLE Peripheral<br/>UUID: 19B10000..."]
        BLE_Rx["BLE Control Characteristic<br/>(Run / Stop / Reset / Fast-Tip)"]
        RainSim --> BLE_Tx
        BLE_Rx --> RainSim
    end

    subgraph ESP32 ["2. ESP32-S3 Touch AMOLED 1.75 (Field Node)"]
        BLE_Client["BLE Central Client"]
        Disp["466x466 Round AMOLED Display<br/>+ ES8311 I2S Audio Chimes"]
        WiFi_Mod["Wi-Fi Gateway Client"]
        FB_Client["Firebase Realtime Sync Engine"]
        
        BLE_Client --> Disp
        BLE_Client --> FB_Client
        WiFi_Mod --> FB_Client
        FB_Client -.->|"Forward Run/Stop Cmd"| BLE_Client
    end

    subgraph Cloud ["3. Google Firebase Realtime Database"]
        DB_State["/rain_station/telemetry<br/>• tips_count, drop_count<br/>• depth_mm, rain_rate_mmh<br/>• weather_status, timestamp"]
        DB_Cmd["/rain_station/control<br/>• state: RUN / STOP<br/>• reset_trigger<br/>• tip_trigger"]
        DB_AI["/rain_station/ai_prediction<br/>• storm_risk_score<br/>• 30min_forecast<br/>• anomaly_detected"]
    end

    subgraph Web ["4. Meteorological Web Dashboard & AI Hub"]
        UI["Real-Time Web Dashboard<br/>(HTML5 / CSS Glassmorphism / JS)"]
        Chart["Precipitation & Rain Rate Trend Chart"]
        Ctrl["Remote Station Control<br/>[RUN] [STOP] [RESET] [FAST-TIP]"]
        AI_Engine["AI Rainfall Predictor & Anomaly Detector<br/>(Ready for ML / TensorFlow.js / Vertex AI)"]
        
        UI --> Ctrl
        Ctrl -->|"Write Command"| DB_Cmd
        DB_State -->|"Real-time Listen"| UI
        DB_State --> Chart
        DB_State --> AI_Engine
        AI_Engine -->|"Publish Forecast"| DB_AI
    end

    BLE_Tx -->|"BLE Notify (Rain Drops/Tips)"| BLE_Client
    BLE_Client -->|"BLE Write (Cmd: Run/Stop)"| BLE_Rx
    FB_Client -->|"HTTPS REST / WebSocket Patch"| DB_State
    DB_Cmd -->|"Real-time Stream"| FB_Client
```

---

## 📁 Project Structure

```
firebase-rain-ai-monitoring-system/
├── AMOLED_Rain_Receiver_Firebase/
│   └── AMOLED_Rain_Receiver_Firebase.ino    # ESP32-S3 AMOLED Firmware (BLE + Wi-Fi + Firebase + UI + Audio)
├── Nano33BLE_Rain_Transmitter_BiDir/
│   └── Nano33BLE_Rain_Transmitter_BiDir.ino # Nano 33 BLE Simulator (Drop simulation + Bidirectional BLE)
├── web-dashboard/
│   ├── index.html                           # Modern meteorological dashboard & AI forecasting UI
│   ├── style.css                            # Glassmorphism dark aesthetic matching AMOLED UI
│   ├── app.js                               # Real-time telemetry, remote commands, status classifier & AI stub
│   └── firebase-config.js                   # Firebase configuration with seamless Mock/Demo mode fallback
└── README.md                                # Full documentation and setup instructions
```

---

## ⚡ Key Highlights

1. **Bi-Directional Remote Control (RUN / STOP / RESET / FAST-TIP)**:
   - Command triggers on the web dashboard update Firebase `/rain_station/control`.
   - The ESP32-S3 AMOLED station receives the event instantly and writes over BLE to the Nano 33 BLE peripheral.
   - The Nano 33 BLE immediately starts or halts rainfall generation.

2. **Meteorological Classification Engine**:
   - Analyzes real-time tipping frequency ($1\text{ tip} = 0.20\text{ mm}$):
     - **Clear / Sunny**: $0.0\text{ mm/h}$
     - **Light Drizzle**: $< 2.5\text{ mm/h}$
     - **Moderate Rain**: $2.5 - 10.0\text{ mm/h}$
     - **Heavy Rain**: $10.0 - 50.0\text{ mm/h}$
     - **Thunderstorm / Torrential Alert**: $> 50.0\text{ mm/h}$

3. **Future AI-Model Readiness**:
   - Ready data structure for edge AI / Cloud inference (rolling tipping velocity, rainfall rate acceleration, pattern anomalies).
   - Interactive AI prediction widget on the dashboard predicting 30-minute flash-flood/storm risk and anomaly status.
