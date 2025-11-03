#ifndef TELEMETRY_ENCODER_H
#define TELEMETRY_ENCODER_H

#ifdef UNIT_TEST
  // Use mocks for testing
  #include "Arduino.h"
  #include "TinyGPS++.h"
#else
  // Use real libraries for embedded
  #include <Arduino.h>
  #include <TinyGPS++.h>
#endif

#include <pb_encode.h>
#include "can_processor.h"
#include "haltech.h"
#include "telemetry.pb.h"

/**
 * @brief Reads the voltage regulator status via analog input.
 *
 * @param voltageRegPin The analog pin to read from (e.g., A0)
 * @return float Voltage value in volts (0-5V range)
 */
float readRegulatorVoltage(int voltageRegPin);

/**
 * @brief Encodes and transmits telemetry data via radio serial.
 *
 * This function creates a protobuf telemetry packet from CAN data, GPS data,
 * and system status, then transmits it over the radio serial interface.
 *
 * @param data Reference to HaltechData containing all CAN message groups
 * @param gps Reference to GPS object for location data
 * @param canConnected Boolean flag indicating if CAN is connected
 * @param canMessageCount Total number of CAN messages received
 * @param telemetrySentCount Reference to counter tracking sent telemetry packets (will be incremented)
 * @param radioSerial Serial interface for radio transmission (Serial1)
 * @param debugSerial Serial interface for debug output (Serial)
 * @param currentMillis Current system time from millis()
 * @param voltageRegPin Analog pin for voltage regulator monitoring
 * @param protobufBuffer Buffer for protobuf encoding (must be at least TelemetryPacket_size bytes)
 * @param bufferSize Size of the protobuf buffer
 * @return bool True if telemetry was successfully encoded and sent, false otherwise
 */
bool sendTelemetry(const HaltechData& data,
                   TinyGPSPlus& gps,
                   boolean canConnected,
                   unsigned long canMessageCount,
                   unsigned long& telemetrySentCount,
                   Stream& radioSerial,
                   Stream& debugSerial,
                   unsigned long currentMillis,
                   int voltageRegPin,
                   uint8_t* protobufBuffer,
                   size_t bufferSize);

#endif // TELEMETRY_ENCODER_H
