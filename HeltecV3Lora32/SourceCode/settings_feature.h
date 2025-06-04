#pragma once
#include <Arduino.h>
#include <WiFi.h>

// globals that live in settings_feature.cpp
extern String  cfg_ssid;
extern String  cfg_pass;
extern float   cfg_freq_mhz;
extern bool    cfg_duty_override;
extern uint8_t cfg_channel;
extern bool    cfg_lora_enabled;    // LoRa On/Off flag
extern wifi_power_t cfg_wifi_power; // ESP32 Wi-Fi TX power
extern uint8_t cfg_lora_power;    // LoRa TX power (0–22 dBm)

// functions that live in settings_feature.cpp
void loadConfig();
void saveConfig(const String &ssid, const String &pass,
                float freq, bool dutyOvrd, uint8_t channel, bool loraEn, wifi_power_t wifiPower, uint8_t loraPower);
void setupSettingsRoutes();
