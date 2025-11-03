#include "telemetry_encoder.h"

float readRegulatorVoltage(int voltageRegPin)
{
  // Read analog value and convert to voltage
  int rawValue = analogRead(voltageRegPin);

  // Convert the analog reading (0-1023) to voltage (0-5V)
  // Adjust scaling based on your voltage divider if used
  float voltage = rawValue * (5.0 / 1023.0);

  return voltage;
}

/**
 * @brief Reads data from groups and transmits it over radio serial.
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
                   size_t bufferSize)
{
  // Clear previous data
  TelemetryPacket msg = TelemetryPacket_init_zero;

  // Add timestamp, voltage, CAN status
  msg.timestamp = currentMillis;
  msg.voltage = readRegulatorVoltage(voltageRegPin);
  msg.can_connected = canConnected;
  msg.can_messages = canMessageCount;

  // Add GPS data if valid
  msg.gps_valid = gps.location.isValid();
  if (gps.location.isValid())
  {
    msg.has_gps = true;
    msg.gps.lat = gps.location.lat();
    msg.gps.lng = gps.location.lng();
    msg.gps.alt = gps.altitude.meters();
    msg.gps.speed = gps.speed.kmph();
    msg.gps.course = gps.course.deg();
  }

  // Add engine data only if we have valid CAN data
  if (canConnected)
  {
    msg.has_engine = true;
    EnginePacket *eng = &msg.engine;

    msg.has_suspension = true;
    SuspensionPacket *susp = &msg.suspension;

    // Group 0 (Engine basics)
    eng->rpm = haltech_group00_rpm_decode(data.group0.rpm);
    eng->manifold_pressure = haltech_group00_manifold_pressure_decode(data.group0.manifold_pressure);
    eng->throttle_position = haltech_group00_throttle_position_decode(data.group0.throttle_position);

    // Group 1 (Pressures)
    eng->fuel_pressure = haltech_group01_fuel_pressure_decode(data.group1.fuel_pressure);
    eng->oil_pressure = haltech_group01_oil_pressure_decode(data.group1.oil_pressure);
    eng->engine_demand = haltech_group01_engine_demand_decode(data.group1.engine_demand);

    // Group 5 & 39 (Wideband sensors)
    eng->sensor_1 = haltech_group05_wideband_sensor_1_decode(data.group5.wideband_sensor_1);
    eng->sensor_2 = haltech_group05_wideband_sensor_2_decode(data.group5.wideband_sensor_2);
    eng->overall = haltech_group39_wideband_overall_decode(data.group39.wideband_overall);

    // Group 8 (Brake pressure and lateral G)
    susp->brakes.pressure_front = haltech_group08_brake_pressure_front_decode(data.group8.brake_pressure_front);
    susp->gforce.lateral_g = haltech_group08_lateral_g_decode(data.group8.lateral_g);

    // Group 11 (Longitudinal G)
    susp->gforce.longitudinal_g = haltech_group11_longitudinal_g_decode(data.group11.longitudinal_g);

    // Group 13 (Vehicle speed)
    eng->vehicle_speed = haltech_group13_vehicle_speed_decode(data.group13.vehicle_speed);

    // Group 15 (Battery and baro pressure)
    eng->battery_voltage = haltech_group15_battery_voltage_decode(data.group15.battery_voltage);
    eng->barometric_pressure = haltech_group15_barometric_pressure_decode(data.group15.barometric_pressure);

    // Group 20 (Temperatures)
    eng->has_temperature = true;
    eng->temperature.coolant = haltech_group20_coolant_temperature_decode(data.group20.coolant_temperature);
    eng->temperature.air = haltech_group20_air_temperature_decode(data.group20.air_temperature);
    eng->temperature.fuel = haltech_group20_fuel_temperature_decode(data.group20.fuel_temperature);
    eng->temperature.oil = haltech_group20_oil_temperature_decode(data.group20.oil_temperature);

    // Group 24 (Switches and indicators)
    eng->has_switches = true;
    eng->switches.neutral = haltech_group24_neutral_switch_decode(data.group24.neutral_switch);
    eng->switches.oil_pressure_light = haltech_group24_oil_pressure_light_decode(data.group24.oil_pressure_light);
    eng->switches.launch_control_active = haltech_group24_launch_control_active_decode(data.group24.launch_control_active);
    eng->switches.launch_control_switch = haltech_group24_launch_control_switch_decode(data.group24.launch_control_switch);
    eng->switches.thermo_fan = haltech_group24_thermo_fan_1_on_decode(data.group24.thermo_fan_1_on);
    eng->switches.rotary_trim_pot_1 = haltech_group24_rotary_trim_pot_1_decode(data.group24.rotary_trim_pot_1);
    eng->switches.rotary_trim_pot_2 = haltech_group24_rotary_trim_pot_2_decode(data.group24.rotary_trim_pot_2);
    eng->switches.rotary_trim_pot_3 = haltech_group24_rotary_trim_pot_3_decode(data.group24.rotary_trim_pot_3);
    eng->switches.check_engine_light = haltech_group24_check_engine_light_decode(data.group24.check_engine_light);

    // Group 25 (Steering angle and pit lane speed limiter)
    eng->switches.pit_lane_speed_limiter_active = haltech_group25_pit_lane_speed_limiter_active_decode(data.group25.pit_lane_speed_limiter_active);
    eng->switches.pit_lane_speed_limiter_switch_state = haltech_group25_pit_lane_speed_limiter_switch_state_decode(data.group25.pit_lane_speed_limiter_switch_state);
    eng->steering_angle = haltech_group25_steering_wheel_angle_decode(data.group25.steering_wheel_angle);

    // Group 37 (Damper travel)
    susp->damper.travel_front_left = haltech_group37_shock_travel_sensor_front_left_decode(data.group37.shock_travel_sensor_front_left);
    susp->damper.travel_rear_left = haltech_group37_shock_travel_sensor_rear_left_decode(data.group37.shock_travel_sensor_rear_left);
    susp->damper.travel_front_right = haltech_group37_shock_travel_sensor_front_right_decode(data.group37.shock_travel_sensor_front_right);
    susp->damper.travel_rear_right = haltech_group37_shock_travel_sensor_rear_right_decode(data.group37.shock_travel_sensor_rear_right);

    // Group 39 (Gear info)
    eng->gear = haltech_group39_gear_decode(data.group39.gear);

    // Group 40 (APPS)
    eng->accelerator_pedal_position = haltech_group40_accelerator_pedal_position_decode(data.group40.accelerator_pedal_position);

    // Group 43 (G and roll rate)
    susp->gforce.vertical_g = haltech_group43_vertical_g_decode(data.group43.vertical_g);
    susp->rate.pitch_rate = haltech_group43_pitch_rate_decode(data.group43.pitch_rate);
    susp->rate.roll_rate = haltech_group43_roll_rate_decode(data.group43.roll_rate);
    susp->rate.yaw_rate = haltech_group43_yaw_rate_decode(data.group43.yaw_rate);

    // Group 45 (Brake pressure)
    susp->brakes.pressure_rear = haltech_group45_brake_pressure_rear_decode(data.group45.brake_pressure_rear);
    susp->brakes.pressure_front_ratio = haltech_group45_brake_pressure_front_ratio_decode(data.group45.brake_pressure_front_ratio);
    susp->brakes.pressure_rear_ratio = haltech_group45_brake_pressure_rear_ratio_decode(data.group45.brake_pressure_rear_ratio);
    susp->brakes.pressure_difference = haltech_group45_brake_pressure_difference_decode(data.group45.brake_pressure_difference);
  }

  // Encode protobuf message
  pb_ostream_t stream = pb_ostream_from_buffer(protobufBuffer, bufferSize);
  bool status = pb_encode(&stream, TelemetryPacket_fields, &msg);

  if (status)
  {
    // Send the protobuf data via radio
    radioSerial.write(protobufBuffer, stream.bytes_written);
    radioSerial.print("~!");
    telemetrySentCount++;
  }
  else
  {
    debugSerial.println("Protobuf encoding failed");
  }

  // Also print basic info to Serial for debugging
  debugSerial.print("Telemetry #");
  debugSerial.print(telemetrySentCount);
  debugSerial.print(" - CAN: ");
  debugSerial.print(canConnected ? "Connected" : "Disconnected");
  debugSerial.print(", RPM: ");
  debugSerial.print(haltech_group00_rpm_decode(data.group0.rpm));
  debugSerial.print(", Speed: ");
  debugSerial.print(haltech_group13_vehicle_speed_decode(data.group13.vehicle_speed));
  debugSerial.println(" km/h");

  return status;
}
