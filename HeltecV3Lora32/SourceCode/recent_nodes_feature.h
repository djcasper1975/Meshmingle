#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <set>

void  loadRecentNodes();                 // call once from setup()
void  saveRecentNodes();                 // call when you’ve added one
void  addRecentNode(const String& id);   // call whenever you see a new node
const std::set<String>& getRecentNodes();// read-only accessor