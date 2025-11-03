#include <Arduino.h>
#include <mcp2515.h>
#include <TinyGPS++.h>
#include <pb_encode.h>
#include <optional>

// Include the DBC-generated header
#include "haltech.h"

// Include Protobuf header
#include "./telemetry.pb.h"

// Control modules for reading & sending data
#include "can_processor.h"
#include "telemetry_encoder.h"

// Can bus
std::optional<MCP2515> can;

// GPS setup
TinyGPSPlus gps;
#define GPS_SERIAL Serial2

// Nomad Radio (transmit only) setup
#define RADIO_SERIAL Serial1

// Voltage Regulator Status (monitored via analog input)
const int VOLTAGE_REG_PIN = A0;

// Buffer for protobuf encoding
uint8_t protobuf_buffer[TelemetryPacket_size];

// Timing variables
unsigned long lastCanUpdateTime = 0;
unsigned long lastGpsUpdateTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastDebugPrintTime = 0;

// Update intervals (milliseconds)
const unsigned long CAN_UPDATE_INTERVAL = 10;    // 100Hz
const unsigned long GPS_UPDATE_INTERVAL = 100;   // 10Hz
const unsigned long TELEMETRY_INTERVAL = 500;    // 2Hz
const unsigned long DEBUG_PRINT_INTERVAL = 1000; // 1Hz for debugging

// Haltech data buffer
HaltechData haltechData;

// Debug counters
unsigned long canMessageCount = 0;
unsigned long telemetrySentCount = 0;
boolean canConnected = false;

void setupCAN();
void processGpsData();
void debugStatus();

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== Telemetry Data Logger Starting ===");

  // Initialize GPS serial
  GPS_SERIAL.begin(115200);
  Serial.println("GPS initialized on Serial2");

  // Initialize radio serial (transmit only)
  RADIO_SERIAL.begin(38400);
  Serial.println("Radio initialized on Serial1 - Transmit Only");

  // Configure voltage regulator monitoring pin
  pinMode(VOLTAGE_REG_PIN, INPUT);
  Serial.println("Voltage regulator pin configured");

  // Set SPI settings
  SPI1.setSCK(10);
  SPI1.setMISO(12);
  SPI1.setMOSI(11);
  SPI1.begin();

  // Initialize CAN1 connection
  can.emplace(MCP2515(9, 10000000, &SPI1));
  can->setBitrate(CAN_1000KBPS, MCP_8MHZ);
  can->setNormalMode();

  // Initialize data structures to zero
  haltechData.init();

  Serial.println("=== System initialized and ready ===\n");

  // Initial debug status
  debugStatus();
}

void setupCAN()
{
  // NOTE: Since we don't have enough filters to cover each group, we need to split them up.
  // Since all of the groups we want live between 0x360u and 0x476u, we can just filter for
  // everything that starts with 0x3 & 0x4.

  // Start config mode
  can->setConfigMode();

  // One mask for both receive buffers
  can->setFilterMask(MCP2515::MASK0, false, 0x700);
  can->setFilterMask(MCP2515::MASK1, false, 0x700);

  // Filter 0 → 0x300–0x3FF
  can->setFilter(MCP2515::RXF0, false, 0x300);

  // Filter 1 → 0x400–0x4FF
  can->setFilter(MCP2515::RXF1, false, 0x400);

  // You can reuse the same mask for the rest if you want:
  can->setFilter(MCP2515::RXF2, false, 0x300);
  can->setFilter(MCP2515::RXF3, false, 0x400);
  can->setFilter(MCP2515::RXF4, false, 0x300);
  can->setFilter(MCP2515::RXF5, false, 0x400);

  // Return to normal mode
  can->setNormalMode();
}

void loop()
{
  unsigned long currentMillis = millis();

  // Check for CAN messages (highest priority)
  if (currentMillis - lastCanUpdateTime >= CAN_UPDATE_INTERVAL)
  {
    readCanMessages();
    lastCanUpdateTime = currentMillis;
  }

  // Process GPS data
  if (currentMillis - lastGpsUpdateTime >= GPS_UPDATE_INTERVAL)
  {
    processGpsData();
    lastGpsUpdateTime = currentMillis;
  }

  // Send telemetry data
  if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL)
  {
    sendTelemetry();
    lastTelemetryTime = currentMillis;
  }

  // Print debug status
  if (currentMillis - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL)
  {
    debugStatus();
    lastDebugPrintTime = currentMillis;
  }
}

/**
 * @brief Wrapper that calls the extracted readCanMessages function.
 *
 * This keeps the same interface for the main loop while using the testable
 * extracted function from can_processor.cpp.
 */
void readCanMessages()
{
  ::readCanMessages(&*can, haltechData, canMessageCount, canConnected);
}

/**
 * @brief Pulls GPS data and encodes it into the GPS buffer.
 */
void processGpsData()
{
  // Read all available GPS data
  int bytesRead = 0;
  while (GPS_SERIAL.available() &&
         bytesRead < 50)
  { // Limit to prevent blocking
    gps.encode(GPS_SERIAL.read());
    bytesRead++;
  }
}

/**
 * @brief Wrapper that calls the extracted sendTelemetry function.
 *
 * This keeps the same interface for the main loop while using the testable
 * extracted function from telemetry_encoder.cpp.
 */
void sendTelemetry()
{
  ::sendTelemetry(haltechData, gps, canConnected, canMessageCount,
                 telemetrySentCount, RADIO_SERIAL, Serial, millis(),
                 VOLTAGE_REG_PIN, protobuf_buffer, sizeof(protobuf_buffer));
}

void debugStatus()
{
  Serial.println("\n=== System Status ===");
  Serial.printf("Uptime: %d seconds\n", millis() / 1000);
  Serial.printf("CAN Status: %s\n", canConnected ? "Connected" : "Disconnected");
  Serial.printf("CAN Messages Received: %d\n", canMessageCount);
  Serial.printf("Telemetry Packets Sent: %d\n", telemetrySentCount);
  Serial.printf("System Voltage: %fV\n", readRegulatorVoltage(VOLTAGE_REG_PIN));
  Serial.printf("GPS Status: %s\n", GPS_SERIAL.available() ? "Interface unavailable" : gps.location.isValid() ? "Valid" : "Searching...");
  Serial.println("=======================\n");
}
