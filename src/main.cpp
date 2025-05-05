#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>

// Include the DBC-generated header
#include "r3.h"

// CAN bus setup for Teensy 4.1
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // Connected to Haltect R6

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
const unsigned long CAN_UPDATE_INTERVAL = 10;     // 100Hz
const unsigned long GPS_UPDATE_INTERVAL = 100;    // 10Hz
const unsigned long TELEMETRY_INTERVAL = 500;     // 10Hz
const unsigned long DEBUG_PRINT_INTERVAL = 1000;  // 1Hz for debugging

// Storage for CAN message structures
struct r3_group0_t group0;
struct r3_group1_t group1;
struct r3_group5_t group5;
struct r3_group13_t group13;
struct r3_group15_t group15;
struct r3_group20_t group20;
struct r3_group24_t group24;
struct r3_group39_t group39;

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

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== Telemetry Data Logger Starting ===");
    
    // Initialize GPS serial
    GPS_SERIAL.begin(9600);
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
    memset(&group39, 0, sizeof(group39));

    Serial.println("=== System initialized and ready ===\n");
    
    // Initial debug status
    debugStatus();
}

void setupCAN() {
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
  
    can1.setMBFilter(REJECT_ALL);
    can1.setMBFilterRange(MB0, R3_GROUP0_FRAME_ID, R3_GROUP0_FRAME_ID);
    can1.setMBFilterRange(MB1, R3_GROUP1_FRAME_ID, R3_GROUP1_FRAME_ID);
    can1.setMBFilterRange(MB2, R3_GROUP5_FRAME_ID, R3_GROUP5_FRAME_ID);
    can1.setMBFilterRange(MB3, R3_GROUP13_FRAME_ID, R3_GROUP13_FRAME_ID);
    can1.setMBFilterRange(MB4, R3_GROUP15_FRAME_ID, R3_GROUP15_FRAME_ID);
    can1.setMBFilterRange(MB5, R3_GROUP20_FRAME_ID, R3_GROUP20_FRAME_ID);
    can1.setMBFilterRange(MB6, R3_GROUP24_FRAME_ID, R3_GROUP24_FRAME_ID);
    can1.setMBFilterRange(MB7, R3_GROUP39_FRAME_ID, R3_GROUP39_FRAME_ID);
    
    Serial.println("CAN filters configured for groups: 0, 1, 5, 13, 15, 20, 24, 39");
    Serial.println("CAN bus initialization complete");
}

void loop() {
    unsigned long currentMillis = millis();
  
    // Check for CAN messages (highest priority)
    if (currentMillis - lastCanUpdateTime >= CAN_UPDATE_INTERVAL) {
      readCanMessages();
      lastCanUpdateTime = currentMillis;
    }
  
    // Process GPS data 
    if (currentMillis - lastGpsUpdateTime >= GPS_UPDATE_INTERVAL) {
      processGpsData();
      lastGpsUpdateTime = currentMillis;
    }
  
    // Send telemetry data
    if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL) {
      sendTelemetry();
      lastTelemetryTime = currentMillis;
    }
    
    // Print debug status
    if (currentMillis - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL) {
      debugStatus();
      lastDebugPrintTime = currentMillis;
    }
}

void readCanMessages() {
    CAN_message_t msg;
    boolean messagesReceived = false;
  
    // Check for messages on CAN1 (Haltect R6)
    while (can1.read(msg)) {
      messagesReceived = true;
      canMessageCount++;
      canConnected = true;
      
      switch (msg.id) {
        case R3_GROUP0_FRAME_ID:
          r3_group0_unpack(&group0, msg.buf, msg.len);
          break;
  
        case R3_GROUP1_FRAME_ID:
          r3_group1_unpack(&group1, msg.buf, msg.len);
          break;
  
        case R3_GROUP5_FRAME_ID:
          r3_group5_unpack(&group5, msg.buf, msg.len);
          break;
  
        case R3_GROUP13_FRAME_ID:
          r3_group13_unpack(&group13, msg.buf, msg.len);
          break;
  
        case R3_GROUP15_FRAME_ID:
          r3_group15_unpack(&group15, msg.buf, msg.len);
          break;
  
        case R3_GROUP20_FRAME_ID:
          r3_group20_unpack(&group20, msg.buf, msg.len);
          break;
  
        case R3_GROUP24_FRAME_ID:
          r3_group24_unpack(&group24, msg.buf, msg.len);
          break;
  
        case R3_GROUP39_FRAME_ID:
          r3_group39_unpack(&group39, msg.buf, msg.len);
          break;
      }
    }
}

void processGpsData() {
    // Read all available GPS data
    int bytesRead = 0;
    while (GPS_SERIAL.available() && bytesRead < 50) { // Limit to prevent blocking
      gps.encode(GPS_SERIAL.read());
      bytesRead++;
    }
}

void sendTelemetry() {
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
    if (gps.location.isValid()) {
      JsonObject gpsObj = telemetryDoc.createNestedObject("gps");
      gpsObj["lat"] = gps.location.lat();
      gpsObj["lng"] = gps.location.lng();
      gpsObj["alt"] = gps.altitude.meters();
      gpsObj["speed"] = gps.speed.kmph();
      gpsObj["course"] = gps.course.deg();
      gpsObj["satellites"] = gps.satellites.value();
      
      if (gps.date.isValid() && gps.time.isValid()) {
        char dateTime[30];
        sprintf(dateTime, "20%02d-%02d-%02dT%02d:%02d:%02dZ", 
                gps.date.year(), gps.date.month(), gps.date.day(),
                gps.time.hour(), gps.time.minute(), gps.time.second());
        gpsObj["datetime"] = dateTime;
      }
    }
    
    // Add engine data only if we have valid CAN data
    if (canConnected) {
      JsonObject engineObj = telemetryDoc.createNestedObject("engine");
      
      // Group 0 (Engine basics)
      engineObj["rpm"] = r3_group0_rpm_decode(group0.rpm);
      engineObj["manifold_pressure"] = r3_group0_manifold_pressure_decode(group0.manifold_pressure);
      engineObj["throttle_position"] = r3_group0_throttle_position_decode(group0.throttle_position);
      engineObj["coolant_pressure"] = r3_group0_coolant_pressure_decode(group0.coolant_pressure);
      
      // Group 1 (Pressures)
      engineObj["fuel_pressure"] = r3_group1_fuel_pressure_decode(group1.fuel_pressure);
      engineObj["oil_pressure"] = r3_group1_oil_pressure_decode(group1.oil_pressure);
      engineObj["engine_demand"] = r3_group1_engine_demand_decode(group1.engine_demand);
      engineObj["wastegate_pressure"] = r3_group1_wastegate_pressure_decode(group1.wastegate_pressure);
      
      // Group 5 & 39 (Wideband sensors)
      JsonObject wideband = engineObj.createNestedObject("wideband");
      wideband["sensor_1"] = r3_group5_wideband_sensor_1_decode(group5.wideband_sensor_1);
      wideband["sensor_2"] = r3_group5_wideband_sensor_2_decode(group5.wideband_sensor_2);
      wideband["sensor_3"] = r3_group5_wideband_sensor_3_decode(group5.wideband_sensor_3);
      wideband["sensor_4"] = r3_group5_wideband_sensor_4_decode(group5.wideband_sensor_4);
      wideband["overall"] = r3_group39_wideband_overall_decode(group39.wideband_overall);
      wideband["bank_1"] = r3_group39_wideband_bank_1_decode(group39.wideband_bank_1);
      wideband["bank_2"] = r3_group39_wideband_bank_2_decode(group39.wideband_bank_2);
      
      // Group 13 (Vehicle speed and cam angles)
      engineObj["vehicle_speed"] = r3_group13_vehicle_speed_decode(group13.vehicle_speed);
      engineObj["intake_cam_angle_1"] = r3_group13_intake_cam_angle_1_decode(group13.intake_cam_angle_1);
      engineObj["intake_cam_angle_2"] = r3_group13_intake_cam_angle_2_decode(group13.intake_cam_angle_2);
      
      // Group 15 (Battery and boost)
      engineObj["battery_voltage"] = r3_group15_battery_voltage_decode(group15.battery_voltage);
      engineObj["target_boost_level"] = r3_group15_target_boost_level_decode(group15.target_boost_level);
      engineObj["barometric_pressure"] = r3_group15_barometric_pressure_decode(group15.barometric_pressure);
      
      // Group 20 (Temperatures)
      JsonObject temps = engineObj.createNestedObject("temperature");
      temps["coolant"] = r3_group20_coolant_temperature_decode(group20.coolant_temperature);
      temps["air"] = r3_group20_air_temperature_decode(group20.air_temperature);
      temps["fuel"] = r3_group20_fuel_temperature_decode(group20.fuel_temperature);
      temps["oil"] = r3_group20_oil_temperature_decode(group20.oil_temperature);
      
      // Group 24 (Switches and indicators)
      JsonObject switches = engineObj.createNestedObject("switches");
      switches["neutral"] = r3_group24_neutral_switch_decode(group24.neutral_switch);
      switches["gear"] = r3_group24_gear_switch_decode(group24.gear_switch);
      switches["clutch"] = r3_group24_clutch_switch_decode(group24.clutch_switch);
      switches["oil_pressure_light"] = r3_group24_oil_pressure_light_decode(group24.oil_pressure_light);
      switches["flat_shift"] = r3_group24_flat_shift_switch_decode(group24.flat_shift_switch);
      switches["check_engine_light"] = r3_group24_check_engine_light_decode(group24.check_engine_light);
      
      // Group 39 (Gear info)
      engineObj["gear"] = r3_group39_gear_decode(group39.gear);
      engineObj["gear_selector_position"] = r3_group39_gear_selector_position_decode(group39.gear_selector_position);
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
    Serial.print(r3_group0_rpm_decode(group0.rpm));
    Serial.print(", Speed: ");
    Serial.print(r3_group13_vehicle_speed_decode(group13.vehicle_speed));
    Serial.println(" km/h");
}

float readRegulatorVoltage() {
    // Read analog value and convert to voltage
    int rawValue = analogRead(VOLTAGE_REG_PIN);
    
    // Convert the analog reading (0-1023) to voltage (0-5V)
    // Adjust scaling based on your voltage divider if used
    float voltage = rawValue * (5.0 / 1023.0);
    
    return voltage;
}

void debugStatus() {
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