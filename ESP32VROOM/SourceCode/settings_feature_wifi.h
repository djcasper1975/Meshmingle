// settings_feature_wifi.h
#ifndef SETTINGS_FEATURE_WIFI_H
#define SETTINGS_FEATURE_WIFI_H

// Call *before* you start Wi-Fi so stored SSID/PASS are already in RAM.
void loadConfig();

// Call *after* you have created / started AsyncWebServer.
void setupSettingsRoutes();

// These hold the currently loaded values; handy if other code
// (e.g. Wi-Fi setup, OLED status screen) needs to read them.
extern String cfg_ssid;
extern String cfg_pass;

#endif  // SETTINGS_FEATURE_WIFI_H
