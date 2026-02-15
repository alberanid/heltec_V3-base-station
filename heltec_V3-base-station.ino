/*
  Heltec Wireless Stick Lite V3 - LoRa to MQTT Base Station
  
  Receives LoRa packets from mailbox device (868 MHz, SF7)
  Parses packet: [Node ID, Message Type, Hatch State, Door State, Battery %]
  Publishes as JSON to MQTT broker over WiFi with retained flag
  
  Hardware: Heltec Wireless Stick Lite V3 (ESP32 + SX1262)
  
  Configuration: Edit the constants below for your WiFi and MQTT setup
*/

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ========== CONFIGURATION ==========

// WiFi Credentials
const char* WIFI_SSID = "YOUR_SSID_HERE";
const char* WIFI_PASSWORD = "YOUR_PASSWORD_HERE";

// MQTT Broker Configuration
const char* MQTT_BROKER = "192.168.1.100";  // IP or hostname
const int MQTT_PORT = 1883;
const char* MQTT_USER = "your_username";
const char* MQTT_PASSWORD = "your_password";
const char* MQTT_TOPIC = "home/mailbox/status";

// LoRa Configuration (must match mailbox device)
const long LORA_FREQUENCY = 868E6;      // 868 MHz
const int LORA_SPREADING_FACTOR = 7;    // SF7
const int LORA_BANDWIDTH = 125000;      // 125 kHz
const int LORA_CODING_RATE = 5;         // 4/5
const byte LORA_SYNC_WORD = 0x14;       // First byte of 0x1424 sync word

// Heltec Wireless Stick Lite V3 LoRa Pins (verify these for your board version)
const int LORA_CS = 8;       // NSS/Chip Select (GPIO8)
const int LORA_RST = 12;     // Reset (GPIO12)
const int LORA_DIO0 = 14;    // DIO0/IRQ (GPIO14)

// Serial Debug (comment out to disable)
#define ENABLE_SERIAL_DEBUG

#ifdef ENABLE_SERIAL_DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// ========== GLOBAL VARIABLES ==========

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastWiFiCheck = 0;
unsigned long lastMQTTCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 5000;  // Check WiFi every 5s
const unsigned long MQTT_CHECK_INTERVAL = 2000;  // Check MQTT every 2s

// LoRa packet structure
struct LoRaPacket {
  uint8_t nodeID;
  uint8_t messageType;
  uint8_t hatchOpen;      // 1 = open, 0 = closed
  uint8_t doorOpen;       // 1 = open, 0 = closed
  uint8_t batteryPercent; // 0-100%
};

// ========== SETUP ==========

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(100);
  
  DEBUG_PRINTLN("\n\n=== Heltec V3 LoRa to MQTT Base Station ===");
  DEBUG_PRINTLN("Startup...");
  
  // Initialize LoRa
  DEBUG_PRINTLN("Initializing LoRa...");
  if (!initializeLoRa()) {
    DEBUG_PRINTLN("ERROR: LoRa initialization failed!");
    while (1) {
      delay(1000);
    }
  }
  DEBUG_PRINTLN("LoRa initialized successfully");
  
  // Set up MQTT callbacks
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  
  // Connect to WiFi and MQTT
  connectToWiFi();
  connectToMQTT();
  
  DEBUG_PRINTLN("Setup complete. Listening for LoRa packets...");
}

// ========== MAIN LOOP ==========

void loop() {
  // Check WiFi connectivity
  unsigned long now = millis();
  if (now - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      DEBUG_PRINTLN("WiFi disconnected. Reconnecting...");
      connectToWiFi();
    }
  }
  
  // Check MQTT connectivity
  if (now - lastMQTTCheck >= MQTT_CHECK_INTERVAL) {
    lastMQTTCheck = now;
    if (!mqttClient.connected()) {
      DEBUG_PRINTLN("MQTT disconnected. Reconnecting...");
      connectToMQTT();
    }
  }
  
  // Keep MQTT connection alive
  mqttClient.loop();
  
  // Check for incoming LoRa packet
  int packetSize = LoRa.parsePacket();
  if (packetSize == 5) {  // Expected packet size from mailbox
    DEBUG_PRINTLN("LoRa packet received!");
    
    // Read packet bytes
    uint8_t packetData[5] = {0};
    for (int i = 0; i < 5; i++) {
      packetData[i] = LoRa.read();
    }
    
    // Get RSSI for diagnostics
    int rssi = LoRa.packetRssi();
    
    // Parse packet
    LoRaPacket packet = parseLoRaPacket(packetData);
    
    // Validate packet
    if (packet.nodeID == 0x01 && packet.messageType == 0x01) {
      DEBUG_PRINTLN("Packet validation passed");
      
      // Publish to MQTT
      publishToMQTT(packet, rssi);
    } else {
      DEBUG_PRINT("Invalid packet: Node ID=0x");
      //DEBUG_PRINT(packet.nodeID, HEX);
      //DEBUG_PRINT(" Type=0x");
      //DEBUG_PRINTLN(packet.messageType, HEX);
    }
  }
  
  delay(10);  // Small delay to prevent hogging CPU
}

// ========== LoRa FUNCTIONS ==========

bool initializeLoRa() {
  // Initialize SPI for Heltec V3 (SCK=9, MISO=11, MOSI=10)
  SPI.begin(9, 11, 10, LORA_CS);
  
  // Set LoRa CS, RESET, and DIO0 pins
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  
  // Initialize LoRa radio
  if (!LoRa.begin(LORA_FREQUENCY)) {
    return false;
  }
  
  // Configure LoRa parameters
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  
  // Enable CRC
  LoRa.enableCrc();
  
  // Set to receiver mode (continuous RX)
  LoRa.receive();
  
  return true;
}

LoRaPacket parseLoRaPacket(uint8_t* data) {
  LoRaPacket packet;
  packet.nodeID = data[0];
  packet.messageType = data[1];
  packet.hatchOpen = data[2];
  packet.doorOpen = data[3];
  packet.batteryPercent = data[4];
  return packet;
}

// ========== WiFi FUNCTIONS ==========

void connectToWiFi() {
  int attempts = 0;
  const int MAX_ATTEMPTS = 20;  // 10 seconds with 500ms delay
  
  DEBUG_PRINT("Connecting to WiFi: ");
  DEBUG_PRINTLN(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED && attempts < MAX_ATTEMPTS) {
    delay(500);
    DEBUG_PRINT(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    DEBUG_PRINTLN();
    DEBUG_PRINT("WiFi connected! IP: ");
    DEBUG_PRINTLN(WiFi.localIP());
  } else {
    DEBUG_PRINTLN();
    DEBUG_PRINTLN("ERROR: WiFi connection failed");
  }
}

// ========== MQTT FUNCTIONS ==========

void connectToMQTT() {
  int attempts = 0;
  const int MAX_ATTEMPTS = 5;
  
  DEBUG_PRINT("Connecting to MQTT broker: ");
  DEBUG_PRINT(MQTT_BROKER);
  DEBUG_PRINT(":");
  DEBUG_PRINTLN(MQTT_PORT);
  
  while (!mqttClient.connected() && attempts < MAX_ATTEMPTS) {
    DEBUG_PRINT("MQTT connection attempt ");
    DEBUG_PRINT(attempts + 1);
    DEBUG_PRINT("/");
    DEBUG_PRINTLN(MAX_ATTEMPTS);
    
    if (mqttClient.connect("heltec-v3-base-station", MQTT_USER, MQTT_PASSWORD)) {
      DEBUG_PRINTLN("MQTT connected!");
      return;
    } else {
      DEBUG_PRINT("MQTT connection failed. Error code: ");
      DEBUG_PRINTLN(mqttClient.state());
      delay(1000);
      attempts++;
    }
  }
  
  if (!mqttClient.connected()) {
    DEBUG_PRINTLN("ERROR: Could not connect to MQTT broker");
  }
}

void publishToMQTT(LoRaPacket packet, int rssi) {
  // Create JSON payload
  StaticJsonDocument<256> doc;
  doc["node_id"] = packet.nodeID;
  doc["hatch_open"] = packet.hatchOpen == 1 ? true : false;
  doc["door_open"] = packet.doorOpen == 1 ? true : false;
  doc["battery_percent"] = packet.batteryPercent;
  doc["rssi"] = rssi;
  doc["timestamp"] = millis();
  
  // Serialize to string
  char jsonBuffer[256];
  size_t n = serializeJson(doc, jsonBuffer);
  
  // Publish with retain flag
  bool success = mqttClient.publish(MQTT_TOPIC, jsonBuffer, true);
  
  if (success) {
    DEBUG_PRINT("Published to MQTT: ");
    DEBUG_PRINTLN(jsonBuffer);
  } else {
    DEBUG_PRINTLN("ERROR: MQTT publish failed");
  }
}

// ========== END OF CODE ==========
