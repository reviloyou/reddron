// Template for the SITL build (env:mavlink_request_sitl).
//
// Copy this file to secrets.h in the same folder and fill it in.
// secrets.h is listed in .gitignore, so your WiFi password never reaches
// GitHub. Never put real credentials in this example file.

#pragma once

// A 2.4 GHz network: the ESP32 cannot join 5 GHz.
#define PILLARB_WIFI_SSID "your-network-name"
#define PILLARB_WIFI_PASSWORD "your-wifi-password"

// IPv4 address of the Windows PC running Mission Planner's SITL.
// Find it with `ipconfig` (the IPv4 Address of your WiFi adapter).
#define PILLARB_SITL_HOST "192.168.1.50"

// A SITL MAVLink port that Mission Planner itself is NOT connected to,
// so the two do not fight over stream rates. See HARDWARE_GUIDE.md.
#define PILLARB_SITL_PORT 5762
