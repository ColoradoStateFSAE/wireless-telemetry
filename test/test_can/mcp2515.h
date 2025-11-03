#ifndef MCP2515_MOCK_H
#define MCP2515_MOCK_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include <queue>

// CAN frame structure
struct can_frame {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
};

// CAN bitrate constants
enum CAN_SPEED {
    CAN_5KBPS,
    CAN_10KBPS,
    CAN_20KBPS,
    CAN_31K25BPS,
    CAN_33KBPS,
    CAN_40KBPS,
    CAN_50KBPS,
    CAN_80KBPS,
    CAN_83K3BPS,
    CAN_95KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_200KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
};

// CAN clock constants
enum CAN_CLOCK {
    MCP_8MHZ,
    MCP_16MHZ,
    MCP_20MHZ
};

// Mock SPI class forward declaration
class MockSPI;

class MCP2515 {
public:
    enum ERROR {
        ERROR_OK = 0,
        ERROR_FAIL = 1,
        ERROR_ALLTXBUSY = 2,
        ERROR_FAILINIT = 3,
        ERROR_FAILTX = 4,
        ERROR_NOMSG = 5
    };

    enum MASK {
        MASK0,
        MASK1
    };

    enum RXF {
        RXF0,
        RXF1,
        RXF2,
        RXF3,
        RXF4,
        RXF5
    };

    // Constructor
    MCP2515(int cs_pin, int spi_speed, MockSPI* spi);

    // Configuration methods
    ERROR setBitrate(CAN_SPEED speed, CAN_CLOCK clock);
    ERROR setConfigMode();
    ERROR setNormalMode();
    ERROR setFilterMask(MASK mask, bool extended, uint32_t mask_value);
    ERROR setFilter(RXF filter, bool extended, uint32_t filter_value);

    // Message handling
    ERROR readMessage(can_frame* frame);
    ERROR sendMessage(const can_frame* frame);

    // Test helpers
    void injectMessage(const can_frame& frame);
    void clearMessageQueue();
    size_t getQueuedMessageCount() const { return message_queue.size(); }

private:
    int cs_pin;
    int spi_speed;
    MockSPI* spi;
    std::queue<can_frame> message_queue;

    // Configuration tracking
    bool config_mode;
    bool normal_mode;
    CAN_SPEED current_speed;
    CAN_CLOCK current_clock;
};

#endif // MCP2515_MOCK_H
