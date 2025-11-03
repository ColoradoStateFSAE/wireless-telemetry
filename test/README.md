# Running Tests
```bash
# Run all tests
~/.platformio/penv/bin/pio test -e native
```

# How to Use the Mocks

## Injecting CAN Messages
```cpp
// Create a properly formatted message
struct haltech_group00_t data;
haltech_group00_init(&data);
data.rpm = haltech_group00_rpm_encode(3000.0);

// Pack into CAN frame
can_frame frame;
frame.can_id = HALTECH_GROUP00_FRAME_ID;
frame.can_dlc = 8;
haltech_group00_pack(frame.data, &data, 8);

// Inject into mock CAN controller
can->injectMessage(frame);

// Read and process
can_frame received;
can->readMessage(&received);
```

## Controlling Time
```cpp
mock_millis_value = 0;      // Reset to zero
delay(100);                 // Advance by 100ms
EXPECT_EQ(millis(), 100);   // Verify time
```

## Mock Serial Communication
```cpp
// Set data to be read from Serial
Serial1.setReadData("test data");

// Capture written data
Serial1.print("hello");
std::string written = Serial1.getWrittenData();
EXPECT_EQ(written, "hello");
```

# Testing Code

1. **Create a test file** in `test/`
2. **Use the mocks** to simulate hardware behavior
3. **Format test data properly** using pack/encode functions
4. **Write assertions** to verify expected behavior

Example:
```cpp
#include <gtest/gtest.h>
#include "Arduino.h"
#include "mcp2515.h"
#include "haltech.h"

TEST(MyTest, ProcessEngineData) {
    // Create test message
    struct haltech_group00_t data;
    haltech_group00_init(&data);
    data.rpm = haltech_group00_rpm_encode(5000.0);

    // Your processing logic here

    // Verify results
    EXPECT_DOUBLE_EQ(haltech_group00_rpm_decode(data.rpm), 5000.0);
}
```

# Support

For detailed information on:
- Google Test syntax → See [GTest Documentation](https://google.github.io/googletest/)
- PlatformIO testing → See [PlatformIO Docs](https://docs.platformio.org/en/latest/advanced/unit-testing/)
