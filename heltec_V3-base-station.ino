/*
  SPDX-License-Identifier: Apache-2.0

  Heltec Wireless Stick Lite V3 - LoRa to MQTT Base Station
  
  Receives LoRa packets from mailbox device (868 MHz, SF8)
  Parses packet: [Node ID, Message Type, Hatch State, Door State, Battery %]
  Publishes as JSON to MQTT broker over WiFi with retained flag
  
  Hardware: Heltec Wireless Stick Lite V3 (ESP32 + SX1262)
  
  Configuration: Edit the constants below for your WiFi and MQTT setup
*/

#include <Arduino.h>
#include <SPI.h>
#include <LoRaWan_APP.h> // Heltec LoRaWan_APP (provides Radio API for raw LoRa)
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

// LoRa configuration (Italy 868 MHz)
#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             21        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       8         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

// LoRa pins are handled by `LoRaWan_APP` / board drivers — no manual pin setup required here.

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

// Radio event structure (LoRaWan_APP)
static RadioEvents_t RadioEvents;
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxError(void);

// Packet diagnostics (enable for verbose packet logging)
const bool ENABLE_PACKET_DIAGNOSTICS = true;
unsigned long packetsReceived = 0;
unsigned long packetsValid = 0;
unsigned long packetsInvalid = 0;
unsigned long lastStatsPrint = 0;
const unsigned long STATS_INTERVAL = 60000; // print stats every 60s

unsigned long lastWiFiCheck = 0;
unsigned long lastMQTTCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 30000;  // Check WiFi every 30s
const unsigned long MQTT_CHECK_INTERVAL = 15000;  // Check MQTT every 15s

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
  // Print radio configuration for diagnostics
  if (ENABLE_PACKET_DIAGNOSTICS) {
    Serial.printf("LoRa config: freq=%lu SF=%u BW_idx=%u CR=%u preamble=%u\n", RF_FREQUENCY, LORA_SPREADING_FACTOR, LORA_BANDWIDTH, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH);
  }
  
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
  // Process LoRa radio IRQs — incoming packets handled in OnRxDone()
  Radio.IrqProcess();

  // Periodic diagnostics even when no packets arrive
  if (ENABLE_PACKET_DIAGNOSTICS && (millis() - lastStatsPrint >= STATS_INTERVAL)) {
    lastStatsPrint = millis();
    Serial.printf("[LoRa Stats] received=%lu valid=%lu invalid=%lu\n", packetsReceived, packetsValid, packetsInvalid);
  }
  
  delay(10);  // Small delay to prevent hogging CPU
}

// ========== LoRa FUNCTIONS ==========

bool initializeLoRa() {
  // Initialize MCU / board for radio (handled by LoRaWan_APP)
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // Register radio callbacks and initialize radio
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxDone = NULL;
  RadioEvents.TxTimeout = NULL;
  RadioEvents.RxError = OnRxError;
  RadioEvents.RxTimeout = NULL;

  Radio.Init(&RadioEvents);
  // Clear any lingering SX126x errors/IRQs and show current status (diagnostic)
  SX126xClearDeviceErrors();
  SX126xClearIrqStatus(IRQ_RADIO_ALL);
  if (ENABLE_PACKET_DIAGNOSTICS) {
    RadioError_t _devErr = SX126xGetDeviceErrors();
    Serial.printf("SX126x deviceErrors (post-clear)=0x%04X\n", _devErr.Value);
  }
  // Use private sync word to match CubeCell transmitter (Radio.SetPublicNetwork(false) sets sync word 0x1424)
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetRxConfig(MODEM_LORA,
                    LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE,
                    0,                   // bandwidthAFC
                    LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT,
                    LORA_FIX_LENGTH_PAYLOAD_ON,
                    0,                   // payloadLength
                    true,                // crcOn
                    0, 0,
                    false,
                    true);               // rxContinuous

  // Start continuous receive
  Radio.Rx(0);

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

// Helper: print buffer as hex (compact)
static void printHex(const uint8_t *data, uint16_t len) {
  for (uint16_t i = 0; i < len; ++i) {
    Serial.printf("%02X", data[i]);
    if (i < len - 1) Serial.print(" ");
  }
  Serial.println();
}

// Radio callback: called by LoRaWan_APP Radio when an RF packet is received
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  packetsReceived++;
  DEBUG_PRINTLN("OnRxDone: LoRa packet received");

  // Diagnostics: header with size/RSSI/SNR/timestamp
  if (ENABLE_PACKET_DIAGNOSTICS) {
    Serial.printf("[LoRa RX] size=%u rssi=%d snr=%d time=%lu\n", size, rssi, snr, millis());
    Serial.print("[LoRa RX] payload=");
    printHex(payload, size);

    uint16_t irq = SX126xGetIrqStatus();
    Serial.printf("IRQ status: 0x%04X", irq);
    if (irq & IRQ_HEADER_ERROR) Serial.print(" HDR_ERR");
    if (irq & IRQ_CRC_ERROR) Serial.print(" CRC_ERR");
    if (irq & IRQ_SYNCWORD_VALID) Serial.print(" SYNC_OK");
    if (irq & IRQ_HEADER_VALID) Serial.print(" HDR_OK");
    if (irq & IRQ_RX_DONE) Serial.print(" RX_DONE");
    Serial.println();
    SX126xClearIrqStatus(irq);
  }

  if (size == 5) { // expected 5-byte mailbox packet
    LoRaPacket packet = parseLoRaPacket(payload);

    // Log parsed fields
    if (ENABLE_PACKET_DIAGNOSTICS) {
      Serial.printf("[Parsed] node=0x%02X type=0x%02X hatch=%u door=%u batt=%u\n",
                    packet.nodeID, packet.messageType, packet.hatchOpen, packet.doorOpen, packet.batteryPercent);
    }

    // Validate packet (same rules as original)
    if (packet.nodeID == 0x01 && packet.messageType == 0x01) {
      packetsValid++;
      DEBUG_PRINTLN("Packet validation passed (OnRxDone)");
      publishToMQTT(packet, rssi);
    } else {
      packetsInvalid++;
      DEBUG_PRINT("[Invalid packet] node=0x");
      Serial.printf("%02X", packet.nodeID);
      DEBUG_PRINT(" type=0x");
      Serial.printf("%02X\n", packet.messageType);
    }
  } else {
    packetsInvalid++;
    DEBUG_PRINT("[Unexpected payload size] ");
    Serial.println(size);

    // Extra diagnostics when payload size is unexpected (especially size == 0)
    if (size == 0) {
      uint8_t hwPayloadLength = 0;
      uint8_t rxStartBuffer = 0;
      SX126xGetRxBufferStatus(&hwPayloadLength, &rxStartBuffer);
      Serial.printf("HW RxBufferStatus: payloadLength=%u rxStartBuffer=%u\n", hwPayloadLength, rxStartBuffer);

      PacketStatus_t pktStatus;
      SX126xGetPacketStatus(&pktStatus);
      Serial.printf("HW PacketStatus: packetType=%u rssiPkt=%d snrPkt=%d signalRssi=%d freqError=%lu\n",
                    pktStatus.packetType,
                    pktStatus.Params.LoRa.RssiPkt,
                    pktStatus.Params.LoRa.SnrPkt,
                    pktStatus.Params.LoRa.SignalRssiPkt,
                    pktStatus.Params.LoRa.FreqError);

      RadioError_t devErr = SX126xGetDeviceErrors();
      Serial.printf("HW DeviceErrors: 0x%04X\n", devErr.Value);
    }
  }

  // Periodic inline stats (every STATS_INTERVAL)
  if (ENABLE_PACKET_DIAGNOSTICS && (millis() - lastStatsPrint >= STATS_INTERVAL)) {
    lastStatsPrint = millis();
    Serial.printf("[LoRa Stats] received=%lu valid=%lu invalid=%lu\n", packetsReceived, packetsValid, packetsInvalid);
  }

  // Re-enter RX (continuous)
  Radio.Rx(0);
} 

void OnRxError(void) {
  packetsInvalid++;
  Serial.println("[LoRa RX] RxError (header/CRC).");

  uint16_t irq = SX126xGetIrqStatus();
  Serial.printf("IRQ status: 0x%04X\n", irq);
  SX126xClearIrqStatus(irq);

  uint8_t hwPayloadLength = 0;
  uint8_t rxStartBuffer = 0;
  SX126xGetRxBufferStatus(&hwPayloadLength, &rxStartBuffer);
  Serial.printf("HW RxBufferStatus: payloadLength=%u rxStartBuffer=%u\n", hwPayloadLength, rxStartBuffer);

  PacketStatus_t pktStatus;
  SX126xGetPacketStatus(&pktStatus);
  Serial.printf("HW PacketStatus: packetType=%u rssiPkt=%d snrPkt=%d signalRssi=%d freqError=%lu\n",
                pktStatus.packetType,
                pktStatus.Params.LoRa.RssiPkt,
                pktStatus.Params.LoRa.SnrPkt,
                pktStatus.Params.LoRa.SignalRssiPkt,
                pktStatus.Params.LoRa.FreqError);

  // Try clearing device errors (XOSC start flag)
  SX126xClearDeviceErrors();
  RadioError_t devErr = SX126xGetDeviceErrors();
  Serial.printf("HW DeviceErrors: 0x%04X\n", devErr.Value);

  // Re-enter RX
  Radio.Rx(0);
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
