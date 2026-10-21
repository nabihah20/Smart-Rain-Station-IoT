/**
 * Firebase Realtime Database Configuration & Cloud Sync Module
 * Project: Firebase & AI-Ready Meteorological Rain Station
 */

const FIREBASE_CONFIG = {
  // Configured with your Firebase project
  apiKey: "",
  authDomain: "rain-station-iot.firebaseapp.com",
  databaseURL: "https://rain-station-iot-default-rtdb.asia-southeast1.firebasedatabase.app",
  projectId: "rain-station-iot",
  storageBucket: "rain-station-iot.appspot.com"
};

// Application Database Endpoints
const DB_PATHS = {
  TELEMETRY: 'rain_station/telemetry',
  CONTROL: 'rain_station/control',
  AI_PREDICTION: 'rain_station/ai_prediction',
  HISTORY: 'rain_station/history'
};

// Export configuration
window.FIREBASE_CONFIG = FIREBASE_CONFIG;
window.DB_PATHS = DB_PATHS;
