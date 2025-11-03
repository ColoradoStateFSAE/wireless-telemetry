#ifndef CAN_PROCESSOR_H
#define CAN_PROCESSOR_H

#ifdef UNIT_TEST
  // Use mocks for testing
  #include "Arduino.h"
  #include "mcp2515.h"
#else
  // Use real libraries for embedded
  #include <Arduino.h>
  #include <mcp2515.h>
#endif

#include "haltech.h"

/**
 * Container for all Haltech CAN message group data structures.
 * This consolidates all 15 group structs into a single manageable unit.
 */
struct HaltechData {
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

    // Initialize all groups to zero
    void init() {
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
    }
};

/**
 * @brief Reads CAN messages and stores them in their respective Haltech group structures.
 *
 * @param can Pointer to the MCP2515 CAN controller
 * @param data Reference to HaltechData structure containing all group structs
 * @param messageCount Reference to counter tracking total messages received
 * @param connected Reference to boolean flag indicating CAN connection status
 */
void readCanMessages(MCP2515* can,
                     HaltechData& data,
                     unsigned long& messageCount,
                     boolean& connected);

#endif // CAN_PROCESSOR_H
