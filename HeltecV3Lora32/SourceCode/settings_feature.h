#pragma once
#include <Arduino.h>

// globals that live in settings_feature.cpp
extern String  cfg_ssid;
extern String  cfg_pass;
extern float   cfg_freq_mhz;
extern bool    cfg_duty_override;
extern uint8_t cfg_channel;

// functions that live in settings_feature.cpp
void loadConfig();
void saveConfig(const String &ssid, const String &pass,
                float freq, bool dutyOvrd, uint8_t channel);
void setupSettingsRoutes();
