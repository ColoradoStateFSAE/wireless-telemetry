#include "can_processor.h"

/**
 * @brief Pulls all CAN messages and stores them in their respective groups.
 */
void readCanMessages(MCP2515* can,
                     HaltechData& data,
                     unsigned long& messageCount,
                     boolean& connected)
{
  struct can_frame msg;

  // Check for messages on CAN1 (Haltech)
  while (can->readMessage(&msg) == MCP2515::ERROR_OK)
  {
    messageCount++;
    connected = true;

    switch (msg.can_id)
    {
    case HALTECH_GROUP00_FRAME_ID:
      haltech_group00_unpack(&data.group0, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP01_FRAME_ID:
      haltech_group01_unpack(&data.group1, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP05_FRAME_ID:
      haltech_group05_unpack(&data.group5, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP08_FRAME_ID:
      haltech_group08_unpack(&data.group8, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP11_FRAME_ID:
      haltech_group11_unpack(&data.group11, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP13_FRAME_ID:
      haltech_group13_unpack(&data.group13, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP15_FRAME_ID:
      haltech_group15_unpack(&data.group15, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP20_FRAME_ID:
      haltech_group20_unpack(&data.group20, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP24_FRAME_ID:
      haltech_group24_unpack(&data.group24, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP25_FRAME_ID:
      haltech_group25_unpack(&data.group25, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP37_FRAME_ID:
      haltech_group37_unpack(&data.group37, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP39_FRAME_ID:
      haltech_group39_unpack(&data.group39, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP40_FRAME_ID:
      haltech_group40_unpack(&data.group40, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP43_FRAME_ID:
      haltech_group43_unpack(&data.group43, msg.data, msg.can_dlc);
      break;

    case HALTECH_GROUP45_FRAME_ID:
      haltech_group45_unpack(&data.group45, msg.data, msg.can_dlc);
      break;
    }
  }
}
