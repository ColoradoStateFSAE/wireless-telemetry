#include <gtest/gtest.h>
#include "Arduino.h"
#include "mcp2515.h"
#include "haltech.h"

/**
 * This test suite verifies that CAN messages are correctly read and unpacked
 * into the haltech storage groups, mimicking the readCanMessages() function
 * from main.cpp.
 */

// Mock the global storage groups and counters like in main.cpp
struct haltech_group00_t group0;
struct haltech_group01_t group1;
struct haltech_group05_t group5;
struct haltech_group08_t group8;
struct haltech_group11_t group11;
struct haltech_group13_t group13;
struct haltech_group15_t group15;
struct haltech_group20_t group20;
struct haltech_group24_t group24;
struct haltech_group25_t group25;
struct haltech_group37_t group37;
struct haltech_group39_t group39;
struct haltech_group40_t group40;
struct haltech_group43_t group43;
struct haltech_group45_t group45;

unsigned long canMessageCount = 0;
boolean canConnected = false;

// Simulates the readCanMessages() function from main.cpp
void readCanMessages(MCP2515* can)
{
  struct can_frame msg;

  // Check for messages (mimics the actual function)
  while (can->readMessage(&msg) == MCP2515::ERROR_OK)
  {
    canMessageCount++;
    canConnected = true;

    switch (msg.can_id)
    {
    case HALTECH_GROUP00_FRAME_ID:
      haltech_group00_unpack(&group0, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP01_FRAME_ID:
      haltech_group01_unpack(&group1, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP05_FRAME_ID:
      haltech_group05_unpack(&group5, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP08_FRAME_ID:
      haltech_group08_unpack(&group8, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP11_FRAME_ID:
      haltech_group11_unpack(&group11, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP13_FRAME_ID:
      haltech_group13_unpack(&group13, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP15_FRAME_ID:
      haltech_group15_unpack(&group15, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP20_FRAME_ID:
      haltech_group20_unpack(&group20, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP24_FRAME_ID:
      haltech_group24_unpack(&group24, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP25_FRAME_ID:
      haltech_group25_unpack(&group25, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP37_FRAME_ID:
      haltech_group37_unpack(&group37, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP39_FRAME_ID:
      haltech_group39_unpack(&group39, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP40_FRAME_ID:
      haltech_group40_unpack(&group40, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP43_FRAME_ID:
      haltech_group43_unpack(&group43, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP45_FRAME_ID:
      haltech_group45_unpack(&group45, msg.data, msg.can_dlc);
      break;
    }
  }
}

// Test fixture
class CANProcessingTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_millis_value = 0;

        // Initialize CAN controller mock
        can = new MCP2515(9, 10000000, &SPI1);
        can->setBitrate(CAN_1000KBPS, MCP_8MHZ);
        can->setNormalMode();

        // Reset all global storage groups to zero
        memset(&group0, 0, sizeof(group0));
        memset(&group1, 0, sizeof(group1));
        memset(&group5, 0, sizeof(group5));
        memset(&group8, 0, sizeof(group8));
        memset(&group11, 0, sizeof(group11));
        memset(&group13, 0, sizeof(group13));
        memset(&group15, 0, sizeof(group15));
        memset(&group20, 0, sizeof(group20));
        memset(&group24, 0, sizeof(group24));
        memset(&group25, 0, sizeof(group25));
        memset(&group37, 0, sizeof(group37));
        memset(&group39, 0, sizeof(group39));
        memset(&group40, 0, sizeof(group40));
        memset(&group43, 0, sizeof(group43));
        memset(&group45, 0, sizeof(group45));

        // Reset counters
        canMessageCount = 0;
        canConnected = false;
    }

    void TearDown() override {
        delete can;
    }

    // Helper to create and inject a CAN message
    template<typename T>
    void injectMessage(uint32_t frame_id, T& data, int (*pack_fn)(uint8_t*, const T*, size_t)) {
        can_frame frame;
        frame.can_id = frame_id;
        frame.can_dlc = 8;
        pack_fn(frame.data, &data, 8);
        can->injectMessage(frame);
    }

    MCP2515* can;
};

// Test Group 0 (Engine Basics: RPM, Manifold Pressure, Throttle)
TEST_F(CANProcessingTest, Group0_EngineBasics) {
    // Create test data
    struct haltech_group00_t test_data;
    haltech_group00_init(&test_data);
    test_data.rpm = haltech_group00_rpm_encode(3500.0);
    test_data.manifold_pressure = haltech_group00_manifold_pressure_encode(105.5);
    test_data.throttle_position = haltech_group00_throttle_position_encode(65.0);

    // Inject the message
    injectMessage(HALTECH_GROUP00_FRAME_ID, test_data, haltech_group00_pack);

    // Process messages
    readCanMessages(can);

    // Verify data was stored correctly in global group0
    EXPECT_EQ(canMessageCount, 1);
    EXPECT_TRUE(canConnected);
    EXPECT_DOUBLE_EQ(haltech_group00_rpm_decode(group0.rpm), 3500.0);
    EXPECT_DOUBLE_EQ(haltech_group00_manifold_pressure_decode(group0.manifold_pressure), 105.5);
    EXPECT_DOUBLE_EQ(haltech_group00_throttle_position_decode(group0.throttle_position), 65.0);
}

// Test Group 1 (Pressures: Fuel, Oil, Engine Demand)
TEST_F(CANProcessingTest, Group1_Pressures) {
    struct haltech_group01_t test_data;
    haltech_group01_init(&test_data);
    test_data.fuel_pressure = haltech_group01_fuel_pressure_encode(450.0);
    test_data.oil_pressure = haltech_group01_oil_pressure_encode(580.0);
    test_data.engine_demand = haltech_group01_engine_demand_encode(85.5);

    injectMessage(HALTECH_GROUP01_FRAME_ID, test_data, haltech_group01_pack);
    readCanMessages(can);

    EXPECT_EQ(canMessageCount, 1);
    EXPECT_NEAR(haltech_group01_fuel_pressure_decode(group1.fuel_pressure), 450.0, 0.2);
    EXPECT_NEAR(haltech_group01_oil_pressure_decode(group1.oil_pressure), 580.0, 0.2);
    EXPECT_NEAR(haltech_group01_engine_demand_decode(group1.engine_demand), 85.5, 0.1);
}

// Test Group 13 (Vehicle Speed)
TEST_F(CANProcessingTest, Group13_VehicleSpeed) {
    struct haltech_group13_t test_data;
    haltech_group13_init(&test_data);
    test_data.vehicle_speed = haltech_group13_vehicle_speed_encode(125.5);

    injectMessage(HALTECH_GROUP13_FRAME_ID, test_data, haltech_group13_pack);
    readCanMessages(can);

    EXPECT_EQ(canMessageCount, 1);
    EXPECT_DOUBLE_EQ(haltech_group13_vehicle_speed_decode(group13.vehicle_speed), 125.5);
}

// Test Group 20 (Temperatures: Coolant, Air, Fuel, Oil)
TEST_F(CANProcessingTest, Group20_Temperatures) {
    struct haltech_group20_t test_data;
    haltech_group20_init(&test_data);
    test_data.coolant_temperature = haltech_group20_coolant_temperature_encode(92.5);
    test_data.air_temperature = haltech_group20_air_temperature_encode(28.0);
    test_data.fuel_temperature = haltech_group20_fuel_temperature_encode(35.5);
    test_data.oil_temperature = haltech_group20_oil_temperature_encode(95.0);

    injectMessage(HALTECH_GROUP20_FRAME_ID, test_data, haltech_group20_pack);
    readCanMessages(can);

    EXPECT_EQ(canMessageCount, 1);
    EXPECT_DOUBLE_EQ(haltech_group20_coolant_temperature_decode(group20.coolant_temperature), 92.5);
    EXPECT_DOUBLE_EQ(haltech_group20_air_temperature_decode(group20.air_temperature), 28.0);
    EXPECT_DOUBLE_EQ(haltech_group20_fuel_temperature_decode(group20.fuel_temperature), 35.5);
    EXPECT_DOUBLE_EQ(haltech_group20_oil_temperature_decode(group20.oil_temperature), 95.0);
}

// Test Group 15 (Battery Voltage and Barometric Pressure)
TEST_F(CANProcessingTest, Group15_BatteryAndBaro) {
    struct haltech_group15_t test_data;
    haltech_group15_init(&test_data);
    test_data.battery_voltage = haltech_group15_battery_voltage_encode(13.8);
    test_data.barometric_pressure = haltech_group15_barometric_pressure_encode(101.3);

    injectMessage(HALTECH_GROUP15_FRAME_ID, test_data, haltech_group15_pack);
    readCanMessages(can);

    EXPECT_EQ(canMessageCount, 1);
    EXPECT_NEAR(haltech_group15_battery_voltage_decode(group15.battery_voltage), 13.8, 0.1);
    EXPECT_NEAR(haltech_group15_barometric_pressure_decode(group15.barometric_pressure), 101.3, 0.2);
}

// Test multiple messages from different groups
TEST_F(CANProcessingTest, MultipleGroups_Sequential) {
    // Create Group 0 message (RPM)
    struct haltech_group00_t data0;
    haltech_group00_init(&data0);
    data0.rpm = haltech_group00_rpm_encode(4200.0);
    injectMessage(HALTECH_GROUP00_FRAME_ID, data0, haltech_group00_pack);

    // Create Group 13 message (Speed)
    struct haltech_group13_t data13;
    haltech_group13_init(&data13);
    data13.vehicle_speed = haltech_group13_vehicle_speed_encode(98.5);
    injectMessage(HALTECH_GROUP13_FRAME_ID, data13, haltech_group13_pack);

    // Create Group 20 message (Temps)
    struct haltech_group20_t data20;
    haltech_group20_init(&data20);
    data20.coolant_temperature = haltech_group20_coolant_temperature_encode(88.0);
    injectMessage(HALTECH_GROUP20_FRAME_ID, data20, haltech_group20_pack);

    // Process all messages at once
    readCanMessages(can);

    // Verify all groups were populated correctly
    EXPECT_EQ(canMessageCount, 3);
    EXPECT_TRUE(canConnected);
    EXPECT_DOUBLE_EQ(haltech_group00_rpm_decode(group0.rpm), 4200.0);
    EXPECT_DOUBLE_EQ(haltech_group13_vehicle_speed_decode(group13.vehicle_speed), 98.5);
    EXPECT_DOUBLE_EQ(haltech_group20_coolant_temperature_decode(group20.coolant_temperature), 88.0);
}

// Test that data persists across multiple readCanMessages calls
TEST_F(CANProcessingTest, DataPersistence_AcrossReads) {
    // First batch: Set RPM
    struct haltech_group00_t data0;
    haltech_group00_init(&data0);
    data0.rpm = haltech_group00_rpm_encode(3000.0);
    injectMessage(HALTECH_GROUP00_FRAME_ID, data0, haltech_group00_pack);
    readCanMessages(can);

    EXPECT_EQ(canMessageCount, 1);
    EXPECT_DOUBLE_EQ(haltech_group00_rpm_decode(group0.rpm), 3000.0);

    // Second batch: Update RPM
    data0.rpm = haltech_group00_rpm_encode(3500.0);
    injectMessage(HALTECH_GROUP00_FRAME_ID, data0, haltech_group00_pack);
    readCanMessages(can);

    EXPECT_EQ(canMessageCount, 2);
    EXPECT_DOUBLE_EQ(haltech_group00_rpm_decode(group0.rpm), 3500.0);

    // Third batch: Add different group (speed), RPM should remain
    struct haltech_group13_t data13;
    haltech_group13_init(&data13);
    data13.vehicle_speed = haltech_group13_vehicle_speed_encode(75.0);
    injectMessage(HALTECH_GROUP13_FRAME_ID, data13, haltech_group13_pack);
    readCanMessages(can);

    EXPECT_EQ(canMessageCount, 3);
    EXPECT_DOUBLE_EQ(haltech_group00_rpm_decode(group0.rpm), 3500.0);  // Should still be 3500
    EXPECT_DOUBLE_EQ(haltech_group13_vehicle_speed_decode(group13.vehicle_speed), 75.0);
}

// Test empty queue doesn't crash or change state
TEST_F(CANProcessingTest, NoMessages_NoChanges) {
    // Don't inject any messages
    readCanMessages(can);

    EXPECT_EQ(canMessageCount, 0);
    EXPECT_FALSE(canConnected);
    EXPECT_EQ(group0.rpm, 0);  // Should still be zero-initialized
}

// Test rapid message burst
TEST_F(CANProcessingTest, RapidMessageBurst) {
    // Inject 10 messages rapidly
    for (int i = 0; i < 10; i++) {
        struct haltech_group00_t data;
        haltech_group00_init(&data);
        data.rpm = haltech_group00_rpm_encode(1000.0 + (i * 500.0));
        injectMessage(HALTECH_GROUP00_FRAME_ID, data, haltech_group00_pack);
    }

    // Process all at once
    readCanMessages(can);

    EXPECT_EQ(canMessageCount, 10);
    EXPECT_TRUE(canConnected);
    // Last message should be RPM = 1000 + (9 * 500) = 5500
    EXPECT_DOUBLE_EQ(haltech_group00_rpm_decode(group0.rpm), 5500.0);
}

// Test Group 39 (Gear and Wideband Overall)
TEST_F(CANProcessingTest, Group39_GearAndWideband) {
    struct haltech_group39_t test_data;
    haltech_group39_init(&test_data);
    test_data.gear = haltech_group39_gear_encode(4.0);  // 4th gear
    test_data.wideband_overall = haltech_group39_wideband_overall_encode(14.7);  // AFR

    injectMessage(HALTECH_GROUP39_FRAME_ID, test_data, haltech_group39_pack);
    readCanMessages(can);

    EXPECT_EQ(canMessageCount, 1);
    EXPECT_DOUBLE_EQ(haltech_group39_gear_decode(group39.gear), 4.0);
    EXPECT_NEAR(haltech_group39_wideband_overall_decode(group39.wideband_overall), 14.7, 0.01);
}

// Test all groups can be populated simultaneously
TEST_F(CANProcessingTest, AllGroups_Simultaneous) {
    // Inject one message from each group
    struct haltech_group00_t d0;
    haltech_group00_init(&d0);
    d0.rpm = haltech_group00_rpm_encode(5000.0);
    injectMessage(HALTECH_GROUP00_FRAME_ID, d0, haltech_group00_pack);

    struct haltech_group01_t d1;
    haltech_group01_init(&d1);
    d1.fuel_pressure = haltech_group01_fuel_pressure_encode(400.0);
    injectMessage(HALTECH_GROUP01_FRAME_ID, d1, haltech_group01_pack);

    struct haltech_group13_t d13;
    haltech_group13_init(&d13);
    d13.vehicle_speed = haltech_group13_vehicle_speed_encode(100.0);
    injectMessage(HALTECH_GROUP13_FRAME_ID, d13, haltech_group13_pack);

    struct haltech_group20_t d20;
    haltech_group20_init(&d20);
    d20.coolant_temperature = haltech_group20_coolant_temperature_encode(90.0);
    injectMessage(HALTECH_GROUP20_FRAME_ID, d20, haltech_group20_pack);

    // Process all
    readCanMessages(can);

    // Verify
    EXPECT_EQ(canMessageCount, 4);
    EXPECT_DOUBLE_EQ(haltech_group00_rpm_decode(group0.rpm), 5000.0);
    EXPECT_DOUBLE_EQ(haltech_group01_fuel_pressure_decode(group1.fuel_pressure), 400.0);
    EXPECT_DOUBLE_EQ(haltech_group13_vehicle_speed_decode(group13.vehicle_speed), 100.0);
    EXPECT_DOUBLE_EQ(haltech_group20_coolant_temperature_decode(group20.coolant_temperature), 90.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
