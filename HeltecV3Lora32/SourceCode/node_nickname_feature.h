#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <map>

void loadNodeNicknames();
void saveNodeNicknames();
void setNodeNickname(const String& nodeId, const String& nickname);
String getNodeNickname(const String& nodeId);
const std::map<String, String>& getAllNodeNicknames();