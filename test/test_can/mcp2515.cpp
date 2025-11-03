#include "mcp2515.h"
#include <cstring>

MCP2515::MCP2515(int cs_pin, int spi_speed, MockSPI* spi)
    : cs_pin(cs_pin)
    , spi_speed(spi_speed)
    , spi(spi)
    , config_mode(false)
    , normal_mode(false)
    , current_speed(CAN_1000KBPS)
    , current_clock(MCP_8MHZ)
{
}

MCP2515::ERROR MCP2515::setBitrate(CAN_SPEED speed, CAN_CLOCK clock) {
    current_speed = speed;
    current_clock = clock;
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::setConfigMode() {
    config_mode = true;
    normal_mode = false;
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::setNormalMode() {
    config_mode = false;
    normal_mode = true;
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::setFilterMask(MASK mask, bool extended, uint32_t mask_value) {
    // Mock implementation - just return success
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::setFilter(RXF filter, bool extended, uint32_t filter_value) {
    // Mock implementation - just return success
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::readMessage(can_frame* frame) {
    if (message_queue.empty()) {
        return ERROR_NOMSG;
    }

    *frame = message_queue.front();
    message_queue.pop();
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::sendMessage(const can_frame* frame) {
    // Mock implementation - could log sent messages if needed
    return ERROR_OK;
}

void MCP2515::injectMessage(const can_frame& frame) {
    message_queue.push(frame);
}

void MCP2515::clearMessageQueue() {
    while (!message_queue.empty()) {
        message_queue.pop();
    }
}
