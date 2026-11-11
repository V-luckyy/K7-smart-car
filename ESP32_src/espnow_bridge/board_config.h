#pragma once
#include <cstdint>

// Flash the same sketch three times: (true,1), (false,2), (false,3).
constexpr bool IS_LEADER = true;
constexpr uint16_t VEHICLE_ID = 1;
constexpr uint16_t LEADER_ID = 1;
constexpr uint8_t RADIO_CHANNEL = 6;
constexpr uint32_t USB_BAUD = 115200;

// On both followers, replace with car1 ESP32's STA MAC (see fleet/link).
// The zero value intentionally accepts no leader; it is not a wildcard.
constexpr uint8_t LEADER_MAC[6] = {0, 0, 0, 0, 0, 0};

static_assert(VEHICLE_ID >= 1 && LEADER_ID >= 1, "vehicle IDs start at 1");
static_assert(RADIO_CHANNEL >= 1 && RADIO_CHANNEL <= 11, "use channel 1..11");
static_assert(IS_LEADER == (VEHICLE_ID == LEADER_ID), "role and vehicle ID disagree");
