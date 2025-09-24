#include <Arduino.h>
#include <ArduinoJson.h>
#include <FlexCAN_T4.h>
#include <TinyGPS++.h>

// Include the DBC-generated header
#include "haltech.h"

// CAN bus setup for Teensy 4.1
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // Connected to Haltect R6

// GPS setup
TinyGPSPlus gps;
#define GPS_SERIAL Serial2

// Nomad Radio (transmit only) setup
#define RADIO_SERIAL Serial1

// Voltage Regulator Status (monitored via analog input)
const int VOLTAGE_REG_PIN = A0;

// JSON document for telemetry serialization
StaticJsonDocument<2048> telemetryDoc;

// Timing variables
unsigned long lastCanUpdateTime = 0;
unsigned long lastGpsUpdateTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastDebugPrintTime = 0;

// Update intervals (milliseconds)
const unsigned long CAN_UPDATE_INTERVAL = 10;    // 100Hz
const unsigned long GPS_UPDATE_INTERVAL = 100;   // 10Hz
const unsigned long TELEMETRY_INTERVAL = 500;    // 10Hz
const unsigned long DEBUG_PRINT_INTERVAL = 1000; // 1Hz for debugging

// Storage for CAN message structures
struct haltech_group00_t group0;
struct haltech_group01_t group1;
struct haltech_group05_t group5;
struct haltech_group13_t group13;
struct haltech_group15_t group15;
struct haltech_group20_t group20;
struct haltech_group24_t group24;
struct haltech_group25_t group25;
struct haltech_group37_t group37;
struct haltech_group39_t group39;

// Debug counters
unsigned long canMessageCount = 0;
unsigned long telemetrySentCount = 0;
boolean canConnected = false;

void setupCAN();
void readCanMessages();
void processGpsData();
void sendTelemetry();
float readRegulatorVoltage();
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

  setupCAN();

  // Initialize data structures to zero
  memset(&group0, 0, sizeof(group0));
  memset(&group1, 0, sizeof(group1));
  memset(&group5, 0, sizeof(group5));
  memset(&group13, 0, sizeof(group13));
  memset(&group15, 0, sizeof(group15));
  memset(&group20, 0, sizeof(group20));
  memset(&group24, 0, sizeof(group24));
  memset(&group25, 0, sizeof(group25));
  memset(&group37, 0, sizeof(group37));
  memset(&group39, 0, sizeof(group39));

  Serial.println("=== System initialized and ready ===\n");

  // Initial debug status
  debugStatus();
}

void setupCAN()
{
  Serial.println("Initializing CAN bus...");

  // Initialize CAN1 - Connection to Haltect R6
  can1.begin();
  can1.setBaudRate(1000000);
  Serial.println("CAN bus set to 1Mb");

  // Set up basic filters for important message IDs
  can1.setMB(MB0, RX, STD);
  can1.setMB(MB1, RX, STD);
  can1.setMB(MB2, RX, STD);
  can1.setMB(MB3, RX, STD);
  can1.setMB(MB4, RX, STD);
  can1.setMB(MB5, RX, STD);
  can1.setMB(MB6, RX, STD);
  can1.setMB(MB7, RX, STD);
  can1.setMB(MB8, RX, STD);
  can1.setMB(MB9, RX, STD);

  can1.setMBFilter(REJECT_ALL);
  can1.setMBFilterRange(MB0, HALTECH_GROUP00_FRAME_ID,
                        HALTECH_GROUP00_FRAME_ID);
  can1.setMBFilterRange(MB1, HALTECH_GROUP01_FRAME_ID,
                        HALTECH_GROUP01_FRAME_ID);
  can1.setMBFilterRange(MB2, HALTECH_GROUP05_FRAME_ID,
                        HALTECH_GROUP05_FRAME_ID);
  can1.setMBFilterRange(MB3, HALTECH_GROUP13_FRAME_ID,
                        HALTECH_GROUP13_FRAME_ID);
  can1.setMBFilterRange(MB4, HALTECH_GROUP15_FRAME_ID,
                        HALTECH_GROUP15_FRAME_ID);
  can1.setMBFilterRange(MB5, HALTECH_GROUP20_FRAME_ID,
                        HALTECH_GROUP20_FRAME_ID);
  can1.setMBFilterRange(MB6, HALTECH_GROUP24_FRAME_ID,
                        HALTECH_GROUP24_FRAME_ID);
  can1.setMBFilterRange(MB7, HALTECH_GROUP25_FRAME_ID,
                        HALTECH_GROUP25_FRAME_ID);
  can1.setMBFilterRange(MB8, HALTECH_GROUP37_FRAME_ID,
                        HALTECH_GROUP37_FRAME_ID);
  can1.setMBFilterRange(MB9, HALTECH_GROUP39_FRAME_ID,
                        HALTECH_GROUP39_FRAME_ID);

  Serial.println(
      "CAN filters configured for groups: 0, 1, 5, 13, 15, 20, 24, 25, 37, 39");
  Serial.println("CAN bus initialization complete");
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

void readCanMessages()
{
  CAN_message_t msg;

  // Check for messages on CAN1 (Haltect R6)
  while (can1.read(msg))
  {
    canMessageCount++;
    canConnected = true;

    switch (msg.id)
    {
    case HALTECH_GROUP00_FRAME_ID:
      haltech_group00_unpack(&group0, msg.buf, msg.len);
      break;

    case HALTECH_GROUP01_FRAME_ID:
      haltech_group01_unpack(&group1, msg.buf, msg.len);
      break;

    case HALTECH_GROUP05_FRAME_ID:
      haltech_group05_unpack(&group5, msg.buf, msg.len);
      break;

    case HALTECH_GROUP13_FRAME_ID:
      haltech_group13_unpack(&group13, msg.buf, msg.len);
      break;

    case HALTECH_GROUP15_FRAME_ID:
      haltech_group15_unpack(&group15, msg.buf, msg.len);
      break;

    case HALTECH_GROUP20_FRAME_ID:
      haltech_group20_unpack(&group20, msg.buf, msg.len);
      break;

    case HALTECH_GROUP24_FRAME_ID:
      haltech_group24_unpack(&group24, msg.buf, msg.len);
      break;

    case HALTECH_GROUP25_FRAME_ID:
      haltech_group25_unpack(&group25, msg.buf, msg.len);
      break;

    case HALTECH_GROUP37_FRAME_ID:
      haltech_group37_unpack(&group37, msg.buf, msg.len);
      break;

    case HALTECH_GROUP39_FRAME_ID:
      haltech_group39_unpack(&group39, msg.buf, msg.len);
      break;
    }
  }
}

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

void sendTelemetry()
{
  // Clear previous data
  telemetryDoc.clear();

  // Add timestamp
  telemetryDoc["timestamp"] = millis();

  // Add system voltage
  telemetryDoc["voltage"] = readRegulatorVoltage();

  // Add CAN connection status
  telemetryDoc["can_connected"] = canConnected;
  telemetryDoc["can_messages"] = canMessageCount;

  // Add GPS data if valid
  telemetryDoc["gps_valid"] = gps.location.isValid();
  if (gps.location.isValid())
  {
    JsonObject gpsObj = telemetryDoc.createNestedObject("gps");
    gpsObj["lat"] = gps.location.lat();
    gpsObj["lng"] = gps.location.lng();
    gpsObj["alt"] = gps.altitude.meters();
    gpsObj["speed"] = gps.speed.kmph();
    gpsObj["course"] = gps.course.deg();
    gpsObj["satellites"] = gps.satellites.value();

    if (gps.date.isValid() && gps.time.isValid())
    {
      char dateTime[30];
      sprintf(dateTime, "20%02d-%02d-%02dT%02d:%02d:%02dZ", gps.date.year(),
              gps.date.month(), gps.date.day(), gps.time.hour(),
              gps.time.minute(), gps.time.second());
      gpsObj["datetime"] = dateTime;
    }
  }

  // Add engine data only if we have valid CAN data
  if (canConnected)
  {
    JsonObject engineObj = telemetryDoc.createNestedObject("engine");

    // Group 0 (Engine basics)
    engineObj["rpm"] = haltech_group00_rpm_decode(group0.rpm);
    engineObj["manifold_pressure"] =
        haltech_group00_manifold_pressure_decode(group0.manifold_pressure);
    engineObj["throttle_position"] =
        haltech_group00_throttle_position_decode(group0.throttle_position);
    engineObj["coolant_pressure"] =
        haltech_group00_coolant_pressure_decode(group0.coolant_pressure);

    // Group 1 (Pressures)
    engineObj["fuel_pressure"] =
        haltech_group01_fuel_pressure_decode(group1.fuel_pressure);
    engineObj["oil_pressure"] =
        haltech_group01_oil_pressure_decode(group1.oil_pressure);
    engineObj["engine_demand"] =
        haltech_group01_engine_demand_decode(group1.engine_demand);
    engineObj["wastegate_pressure"] =
        haltech_group01_wastegate_pressure_decode(group1.wastegate_pressure);

    // Group 5 & 39 (Wideband sensors)
    JsonObject wideband = engineObj.createNestedObject("wideband");
    wideband["sensor_1"] =
        haltech_group05_wideband_sensor_1_decode(group5.wideband_sensor_1);
    wideband["sensor_2"] =
        haltech_group05_wideband_sensor_2_decode(group5.wideband_sensor_2);
    wideband["sensor_3"] =
        haltech_group05_wideband_sensor_3_decode(group5.wideband_sensor_3);
    wideband["sensor_4"] =
        haltech_group05_wideband_sensor_4_decode(group5.wideband_sensor_4);
    wideband["overall"] =
        haltech_group39_wideband_overall_decode(group39.wideband_overall);
    wideband["bank_1"] =
        haltech_group39_wideband_bank_1_decode(group39.wideband_bank_1);
    wideband["bank_2"] =
        haltech_group39_wideband_bank_2_decode(group39.wideband_bank_2);

    // Group 13 (Vehicle speed and cam angles)
    engineObj["vehicle_speed"] =
        haltech_group13_vehicle_speed_decode(group13.vehicle_speed);
    engineObj["intake_cam_angle_1"] =
        haltech_group13_intake_cam_angle_1_decode(group13.intake_cam_angle_1);
    engineObj["intake_cam_angle_2"] =
        haltech_group13_intake_cam_angle_2_decode(group13.intake_cam_angle_2);

    // Group 15 (Battery and boost)
    engineObj["battery_voltage"] =
        haltech_group15_battery_voltage_decode(group15.battery_voltage);
    engineObj["target_boost_level"] =
        haltech_group15_target_boost_level_decode(group15.target_boost_level);
    engineObj["barometric_pressure"] =
        haltech_group15_barometric_pressure_decode(group15.barometric_pressure);

    // Group 20 (Temperatures)
    JsonObject temps = engineObj.createNestedObject("temperature");
    temps["coolant"] =
        haltech_group20_coolant_temperature_decode(group20.coolant_temperature);
    temps["air"] = haltech_group20_air_temperature_decode(group20.air_temperature);
    temps["fuel"] =
        haltech_group20_fuel_temperature_decode(group20.fuel_temperature);
    temps["oil"] = haltech_group20_oil_temperature_decode(group20.oil_temperature);

    // Group 24 (Switches and indicators)
    JsonObject switches = engineObj.createNestedObject("switches");
    switches["neutral"] =
        haltech_group24_neutral_switch_decode(group24.neutral_switch);
    switches["gear"] = haltech_group24_gear_switch_decode(group24.gear_switch);
    switches["clutch"] = haltech_group24_clutch_switch_decode(group24.clutch_switch);
    switches["oil_pressure_light"] =
        haltech_group24_oil_pressure_light_decode(group24.oil_pressure_light);
    switches["flat_shift"] =
        haltech_group24_flat_shift_switch_decode(group24.flat_shift_switch);
    switches["check_engine_light"] =
        haltech_group24_check_engine_light_decode(group24.check_engine_light);

    // Group 25 (Steering angle)
    engineObj["steering_angle"] =
        haltech_group25_steering_wheel_angle_decode(group25.steering_wheel_angle);

    // Group 37 (Damper travel)
    engineObj["travel_front_left"] = haltech_group37_shock_travel_sensor_front_left_decode(group37.shock_travel_sensor_front_left);
    engineObj["travel_rear_left"] = haltech_group37_shock_travel_sensor_rear_left_decode(group37.shock_travel_sensor_rear_left);
    engineObj["travel_front_right"] = haltech_group37_shock_travel_sensor_front_right_decode(group37.shock_travel_sensor_front_right);
    engineObj["travel_rear_right"] = haltech_group37_shock_travel_sensor_rear_right_decode(group37.shock_travel_sensor_rear_right);

    // Group 39 (Gear info)
    engineObj["gear"] = haltech_group39_gear_decode(group39.gear);
    engineObj["gear_selector_position"] =
        haltech_group39_gear_selector_position_decode(group39.gear_selector_position);
  }

  // Send the JSON data via radio
  serializeJson(telemetryDoc, RADIO_SERIAL);
  RADIO_SERIAL.println(); // Add newline for easier parsing

  telemetrySentCount++;

  // Also print basic info to Serial for debugging
  Serial.print("Telemetry #");
  Serial.print(telemetrySentCount);
  Serial.print(" - CAN: ");
  Serial.print(canConnected ? "Connected" : "Disconnected");
  Serial.print(", RPM: ");
  Serial.print(haltech_group00_rpm_decode(group0.rpm));
  Serial.print(", Speed: ");
  Serial.print(haltech_group13_vehicle_speed_decode(group13.vehicle_speed));
  Serial.println(" km/h");
}

float readRegulatorVoltage()
{
  // Read analog value and convert to voltage
  int rawValue = analogRead(VOLTAGE_REG_PIN);

  // Convert the analog reading (0-1023) to voltage (0-5V)
  // Adjust scaling based on your voltage divider if used
  float voltage = rawValue * (5.0 / 1023.0);

  return voltage;
}

void debugStatus()
{
  Serial.println("\n=== System Status ===");
  Serial.print("Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");

  Serial.print("CAN Status: ");
  Serial.println(canConnected ? "Connected" : "Disconnected");

  Serial.print("CAN Messages Received: ");
  Serial.println(canMessageCount);

  Serial.print("Telemetry Packets Sent: ");
  Serial.println(telemetrySentCount);

  Serial.print("System Voltage: ");
  Serial.print(readRegulatorVoltage());
  Serial.println("V");

  Serial.print("GPS Status: ");
  Serial.println(gps.location.isValid() ? "Valid" : "Searching");
  Serial.println("====================\n");
}
