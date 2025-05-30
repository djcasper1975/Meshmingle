////////////////////////////////////////////////////////////////////////
// M    M  EEEEE  SSSSS  H   H  M    M  I  N   N  GGGGG  L      EEEEE //
// MM  MM  E      S      H   H  MM  MM  I  NN  N  G      L      E     //
// M MM M  EEEE   SSSSS  HHHHH  M MM M  I  N N N  G  GG  L      EEEE  //
// M    M  E          S  H   H  M    M  I  N  NN  G   G  L      E     //
// M    M  EEEEE  SSSSS  H   H  M    M  I  N   N   GGG   LLLLL  EEEEE //
////////////////////////////////////////////////////////////////////////
//
//Test v1.00.034
//29-05-2025
//
//
//
//YOU MUST UPDATE ALL YOUR NODES FROM LAST VERSION OR YOU WONT SEE RELAYS ANYMORE!!!!!
//
//MAKE SURE ALL NODES USE THE SAME VERSION OR EXPECT STRANGE THINGS HAPPENING.
//EU868 Band P (869.4 MHz - 869.65 MHz): 10%, 500 mW ERP (10% 24hr 8640 seconds = 6 mins per hour TX Time.)
//After Accounting for Heartbeats: 20 sec after boot then every 15 mins thereafter.
//Per Hour: 136 Max Char messages within the 6-minute (360,000 ms) duty cycle
//Per Day: 3,296 Max Char messages within the 8,640,000 ms (10% duty cycle) allowance
//Added channel number for wifi settings.
//Fixed channel issues we now have 1 node auto assign root
//added battery monitor its not the best but its something for now.(heltec sux for this)
//Added 0% actual voltage
//Added battery cache instead of refreshing readings at will.
//we now calculate empty and full charge and save it for calibration. heltec voltate dividers and batterys all have diff values. hopefully this will be a little better.
//we need to fully charge the battery until orange light goes OFF. Then let the battery run flat. Now you have a calibrated battery and board. 
//added voltage spike reduction when charging currently at -100mv of each reading so when unplugged we dont drop in charge 10% by unplugging.
//V3.2 is now default if you want v3 you must define it.
//changed delay to heltec_delay now the other button powers the unit on and off yippeeee
//Added a battery cell beside the title on oled screen.
//added settings page we can now set wifi ssid and password and select region which auto dissables duty cycle for usa.
//Fixed header links.
//Added wifi Channel to settings page
//Added Wifi TX Power to setings page.
//Added Lora TX Power to setings page.
//edded a loop to start rx every 10 min
//
//
//Uncomment to enable Heltec V3-specific logic or leave as is for Heltec V3.2 if your screen does not work uncommented your on V3.2 board
//#define HELTEC_V3

#ifdef HELTEC_V3
  // V3:   LOW  = connect divider, HIGH = disconnect
  #define FET_CONNECT    LOW
  #define FET_DISCONNECT HIGH
#else
  // V3.2: HIGH = connect divider, LOW = disconnect
  #define FET_CONNECT    HIGH
  #define FET_DISCONNECT LOW
#endif
//
#define RADIOLIB_SX1262
#define HELTEC_POWER_BUTTON // Use the power button feature of Heltec
#include <heltec_unofficial.h> // 0.9.2 Heltec library for OLED and LoRa
#include <painlessMesh.h>  //1.5.4
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <esp_task_wdt.h> // Watchdog timer library
#include <esp_wifi.h>     // for esp_wifi_ap_get_sta_list()
#include <vector>         // For handling list of messages and our queue
#include <map>            // For unified retransmission tracking
#include <RadioLib.h>     //7.1.2
#include <set>            // Used for node id count checking all 3 sources of nodes. i.e wifi, direct lora, indirect lora. now add to total node count.
#include <Preferences.h>  
#include "settings_feature.h"

// ── Battery monitoring ───────────────────────────────────
#define VBAT_Read  1    // ADC1 channel 1 (divider output)
#define ADC_Ctrl   37   // GPIO37 drives the FET gate

// Meshmingle Parameters
#define MESH_PORT 5555

// LoRa Parameters

#define PAUSE 5400000  //Required timeout Time for dutycycle (54 Min)
#define BANDWIDTH 250.0 
#define SPREADING_FACTOR 11 
#define TRANSMIT_POWER 22 //This is max power for This Boards EU868 Config.
#define CODING_RATE 8 

// Some Global Variables
bool enableRxBoost = true; //enable or disable RX Boost Mode
bool sendAggregatedHeartbeats = false;  //0 hop nodelist sharing. Theres issues with this right now!!!!
bool bypassDutyCycle = false;  // Enable or Disable DutyCycle (For Non EU Use ONLY!!!)

String ourApMac;          // upper-case, colon-separated

// --- LoRa RX keep-alive ---
static unsigned long lastRxKick   = 0;
const  unsigned long RX_KICK_MS   = 600000UL;   // 10 min
const  unsigned long RX_IDLE_CAP  = 3000UL;     // avoid kicking while busy

// Calibration: actual vs raw based on 3.7 v 1100mah 4.07w Lithium Battery
const float measuredV  = 4.20f;   // battery full
const float reportedV  = 3.800f;  // divider pin at full
const float reportedV0 = 2.800f;  // divider pin at empty
const float measuredV0 = (reportedV0 / reportedV) * measuredV;

// ——— Battery min/max persistence ———
Preferences prefs;
float      vMin, vMax;               
const float presetVMin = measuredV;  
const float presetVMax = measuredV0;
// how much surface-charge offset to subtract from your peak
const float calibrationOffset = 0.100f; // 100 mV we minus this value of our max v reading when charging to avoid voltage drop when charger is off and batt full. you can tweek this for more accuracy. or set to 0.0 for none.
const float updateThreshold = 0.005f; // 5 mV

// nominal battery endpoints
const float nominalVmin = 3.20f;
const float nominalVmax = 4.20f;


float  calSlope     = 1.0f;
float  calIntercept = 0.0f;

// raw-extrema storage (instead of vMin/vMax)
float vMinRaw, vMaxRaw;

extern float  cachedBatteryVoltage;
extern int    cachedBatteryPercentage;

// ——— BATTERY CACHE ———
float   cachedBatteryVoltage   = 0.0f;
int     cachedBatteryPercentage= 0;
unsigned long lastBatteryCacheUpdate = 0;
const unsigned long batteryCacheInterval = 60000UL; // 60 s

// ---------------------------------------------------------------------
// NEW: Define an enum to track the origin of each message
// ---------------------------------------------------------------------
enum OriginChannel {
  ORIGIN_UNKNOWN,
  ORIGIN_WIFI,
  ORIGIN_LORA
};

// ===================
// TRANSMISSION TRACKING
// ===================
struct TransmissionStatus {
  bool transmittedViaWiFi = false;
  bool transmittedViaLoRa = false;
  bool addedToMessages = false;  // Flag to track if the message has been added to messages vector
  bool relayedViaWiFi = false;     // NEW: whether the message has been relayed via WiFi
  bool relayedViaLoRa = false;     // NEW: whether the message has been relayed via LoRa
  bool queuedForLoRa = false;      // NEW: whether the message is already queued for LoRa transmission
  bool relayRetryAttempted = false; // NEW: whether a retry has already been attempted
  bool pendingWiFiRelay = false;   // NEW: flag to indicate WiFi relay pending after LoRa finishes
  OriginChannel origin = ORIGIN_UNKNOWN; // NEW: track the origin (WiFi vs LoRa) of this message
  uint64_t timestamp = millis();   // record when the entry was created/updated
};

// Map to track transmissions by message ID
std::map<String, TransmissionStatus> messageTransmissions;

// --- CLIENT TRACKING GLOBALS & HELPERS ---

// Keep every MAC we’ve ever seen
static std::set<String> clientMacs;

// Convert raw 6-byte MAC to "AA:BB:CC:DD:EE:FF"
static String macToString(const uint8_t* mac) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}


// Pull the current AP station list and add to our master set
void updateClientList() {
  wifi_sta_list_t staList;
  esp_wifi_ap_get_sta_list(&staList);

  for (int i = 0; i < staList.num; i++) {
    String mac = macToString(staList.sta[i].mac);
    if (mac == ourApMac) continue;   //  <<< skip ourselves
    clientMacs.insert(mac);
  }
}

// ---------------------------------------------------------------------
// NEW: Relay Log Definitions
// ---------------------------------------------------------------------
struct RelayLogEntry {
  String relayID;
  uint64_t timestamp;
};

// Global relay log: maps messageID to a vector of relay log entries.
std::map<String, std::vector<RelayLogEntry>> relayLog;
 
String rxdata;
// Global RX flag
volatile bool rxFlag = false;
long counter = 0;
uint64_t tx_time;
uint64_t last_tx = 0;
uint64_t minimum_pause = 0;
unsigned long lastTransmitTime = 0;  

// Timeout for pending transmissions (e.g. 5 minutes)
const unsigned long pendingTxTimeout = 150000; // 150,000 ms = 2.5 minutes

// Retain transmission history for 24 hours
const unsigned long transmissionHistoryRetention = 86400000; // 86,400,000 ms = 24 hours

// Instead of a single message buffer, we now use a queue for outgoing LoRa messages.
std::vector<String> loraTransmissionQueue;

// Aggregated Heartbeat Interval: 31 minutes (31 * 60 * 1000 = 1,860,000 ms)
const unsigned long aggregatedHeartbeatInterval = 1860000;
unsigned long lastAggregatedHeartbeatTime = 0;

// Define a constant for the direct node timeout (16 minutes)
const uint64_t DIRECT_NODE_TIMEOUT = 16UL * 60UL * 1000UL; // 16 minutes in milliseconds

// Duty Cycle Definitions and Variables
#define DUTY_CYCLE_LIMIT_MS 360000   // 6 minutes in a 60-minute window
#define DUTY_CYCLE_WINDOW   3600000  // 60 minutes in milliseconds

uint64_t cumulativeTxTime = 0;
uint64_t dutyCycleStartTime = 0;

void recalcCalibration() {
  // map [vMinRaw…vMaxRaw] → [nominalVmin…nominalVmax]
  calSlope     = (nominalVmax - nominalVmin) / (vMaxRaw - vMinRaw);
  calIntercept = nominalVmin - calSlope * vMinRaw;
  Serial.printf("Recalculated mapping: Vout = %.3f·Vraw + %.3f\n",
                calSlope, calIntercept);
}

// put this just below the existing recalcCalibration() call
void primeBatteryCacheCalibrated() {
  float rawV = readBatteryVoltage() / 1000.0f;

  // Avoid divide-by-zero if we don’t have a low yet
  float span = vMaxRaw - vMinRaw;
  if (span < 0.05f) {              // never discharged yet → guess 3.20 V
    vMinRaw = rawV;                // treat current value as min for now
    span    = 0.05f;               // 50 mV fake span keeps maths sane
  }

  float calV = nominalVmin +
               (rawV - vMinRaw) * (nominalVmax - nominalVmin) / span;

  calV = constrain(calV, nominalVmin, nominalVmax);

  cachedBatteryVoltage    = calV;
  cachedBatteryPercentage = constrain(
      int((calV - nominalVmin) / (nominalVmax - nominalVmin) * 100), 0, 100);
}


void resetDutyCycle() {
    cumulativeTxTime = 0;
    dutyCycleStartTime = millis();
    Serial.println("[Duty Cycle] Reset duty cycle counter.");
}

void calculateDutyCyclePause(uint64_t tx_time) {
    cumulativeTxTime += tx_time;
    if (millis() - dutyCycleStartTime >= DUTY_CYCLE_WINDOW) {
        resetDutyCycle();
    }

    if (cumulativeTxTime >= DUTY_CYCLE_LIMIT_MS) {
        minimum_pause = DUTY_CYCLE_WINDOW - (millis() - dutyCycleStartTime);
        if (minimum_pause < 0) minimum_pause = 0;
        Serial.printf("[Duty Cycle] Duty cycle limit reached, waiting for %llu ms.\n", minimum_pause);
    } else {
        minimum_pause = 0;
        Serial.printf("[Duty Cycle] Duty cycle time used: %llu ms.\n", cumulativeTxTime);
    }
}

const int maxMessages = 50;

// Duty Cycle Variables    
bool dutyCycleActive = false;     
bool lastDutyCycleActive = false; 

AsyncWebServer server(80);
DNSServer dnsServer;
painlessMesh mesh;

// --- Updated Message structure with a new recipient field ---
// NEW: Added relayIDs vector to hold all relay IDs for our own message.
struct Message {
  String nodeId;     // originator node ID
  String sender;
  String recipient;  // target node (or "ALL" for public messages)
  String content;
  String source;     // e.g., "[WiFi]" or "[LoRa]"
  String messageID;  
  String relayID;    
  std::vector<String> relayIDs; // NEW: List of relay IDs for our own message
  int rssi;          
  float snr;         
  uint64_t timeReceived;
};

std::vector<Message> messages;  

int totalNodeCount = 0;  // <-- now holds the WiFi (mesh) count only
uint32_t currentNodeId = 0;

unsigned long loRaTransmitDelay = 0; 
unsigned long messageCounter = 0;

// Global jitter offset (derived from chip ID) used for all random delays
uint32_t global_jitter_offset = 0;

// New global: (we no longer solely rely on global lastRxRSSI for relayed messages)
  
// Returns a custom formatted node id, for example: "!Mxxxxxx"
String getCustomNodeId(uint32_t nodeId) {
    String hexNodeId = String(nodeId, HEX);
    while (hexNodeId.length() < 6) {
        hexNodeId = "0" + hexNodeId;
    }
    if (hexNodeId.length() > 6) {
        hexNodeId = hexNodeId.substring(hexNodeId.length() - 6);
    }
    return "!M" + hexNodeId;
}

String generateMessageID(const String& nodeId) {
  messageCounter++;
  return nodeId + ":" + String(messageCounter);
}

uint16_t crc16_ccitt(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)buf[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// --- Updated constructMessage() to include the recipient field ---
// The message format is now: 
// messageID|originatorID|sender|recipient|content|relayID|CRC
String constructMessage(const String& messageID, const String& originatorID, const String& sender, const String& recipient, const String& content, const String& relayID) {
  String messageWithoutCRC = messageID + "|" + originatorID + "|" + sender + "|" + recipient + "|" + content + "|" + relayID;
  uint16_t crc = crc16_ccitt((const uint8_t *)messageWithoutCRC.c_str(), messageWithoutCRC.length());
  char crcStr[5];
  sprintf(crcStr, "%04X", crc);
  String fullMessage = messageWithoutCRC + "|" + String(crcStr);
  return fullMessage;
}

// --- METRICS HISTORY CHANGES ---
struct NodeMetricsSample {
  uint64_t timestamp;
  int rssi;
  float snr;
};

struct LoRaNode {
  String nodeId;
  int lastRSSI;
  float lastSNR;
  uint64_t lastSeen;
  std::vector<NodeMetricsSample> history; 
  String statusEmoji; // NEW: Holds the emoji for this node
};

// Global container for direct LoRa nodes
std::map<String, LoRaNode> loraNodes;

// --- NEW: Structure for Indirect Nodes ---
// (Modified to include a history vector.)
struct IndirectNode {
  String originatorId;   // The node that originally sent the message (which we never hear directly)
  String relayId;        // The node that relayed the message to us
  int rssi;              // RSSI as measured from the relay’s transmission
  float snr;             // SNR as measured from the relay’s transmission
  uint64_t lastSeen;     // Timestamp when we last received a relayed message from this originator via the relay
  String statusEmoji;    // NEW: Holds the emoji for this indirect node
  std::vector<NodeMetricsSample> history;  // NEW: History samples (up to 24 hours)
};

// Global container to hold indirect nodes keyed by a composite key (originatorID-relayID)
std::map<String, IndirectNode> indirectNodes;

unsigned long lastCleanupTime = 0;
const unsigned long cleanupInterval = 60000; // 1 minute

uint32_t getNodeId() {
  return currentNodeId;
}

void cleanupLoRaNodes() {
  uint64_t currentTime = millis();
  const uint64_t timeout = 86400000; // 24 hours
  for (auto it = loraNodes.begin(); it != loraNodes.end();) {
    if (currentTime - it->second.lastSeen > timeout) {
      Serial.printf("[LoRa Nodes] Removing inactive LoRa node: %s\n", it->first.c_str());
      it = loraNodes.erase(it);
    } else {
      ++it;
    }
  }
}

// --- NEW: Cleanup function for Indirect Nodes ---
void cleanupIndirectNodes() {
  uint64_t currentTime = millis();
  const uint64_t timeout = 86400000; // 24 hours
  for (auto it = indirectNodes.begin(); it != indirectNodes.end();) {
    if (currentTime - it->second.lastSeen > timeout) {
      Serial.printf("[Indirect Nodes] Removing inactive indirect node: %s\n", it->first.c_str());
      it = indirectNodes.erase(it);
    } else {
      ++it;
    }
  }
}

void cleanupPendingTxQueue() {
  unsigned long now = millis();
  // Iterate over the queue in reverse order so we can remove items safely.
  for (int i = loraTransmissionQueue.size() - 1; i >= 0; i--) {
    String message = loraTransmissionQueue[i];
    // Extract messageID from the message (assuming the format "messageID|...")
    int separatorIndex = message.indexOf('|');
    if (separatorIndex == -1) continue;
    String messageID = message.substring(0, separatorIndex);
    
    // Look up the transmission status for this message
    auto it = messageTransmissions.find(messageID);
    if (it != messageTransmissions.end()) {
      // Remove from queue if the message is already relayed...
      // OR if it has been pending longer than pendingTxTimeout.
      if (it->second.relayedViaLoRa || (now - it->second.timestamp > pendingTxTimeout)) {
        Serial.printf("[Cleanup] Removing pending message %s from queue (stale or already relayed).\n", messageID.c_str());
        loraTransmissionQueue.erase(loraTransmissionQueue.begin() + i);
        // Reset the queued flag so that future scheduling is possible.
        it->second.queuedForLoRa = false;
      }
    } else {
      // If we have no record for this message, remove it.
      loraTransmissionQueue.erase(loraTransmissionQueue.begin() + i);
    }
  }
}

void cleanupTransmissionHistory() {
  unsigned long now = millis();
  for (auto it = messageTransmissions.begin(); it != messageTransmissions.end(); ) {
    if (now - it->second.timestamp > transmissionHistoryRetention) {
      Serial.printf("[Cleanup] Removing transmission history for message %s (older than 24 hrs).\n", it->first.c_str());
      it = messageTransmissions.erase(it);
    } else {
      ++it;
    }
  }
}


// --- Helper functions for dynamic slot calculation ---
// getTxDuration: returns the measured TX duration in ms; if not available, returns preset 3000 ms.
unsigned long getTxDuration(String message) {
  // For now, we simply return 3000. (You can modify this function to compute based on message length.)
  return 3000;
}

// getDynamicSlot: computes the slot duration as baseSlot plus an offset derived from RSSI.
// Here we map RSSI (from -120 to -50) to an offset from 0 to 500 ms.
unsigned long getDynamicSlot(unsigned long baseSlot, int rssi) {
  if (baseSlot == 0) baseSlot = 3000; // default if not available
  if (rssi < -180) rssi = -180;
  if (rssi > -15) rssi = -15;
  int rssiOffset = map(rssi, -180, -15, 0, 4000);
  return baseSlot + rssiOffset;
}

// --- UPDATED addMessage() to accept the recipient and only add private messages if intended ---
// Also, for messages originating from our node, update the relay info and store each unique relayID.
void addMessage(const String& nodeId, const String& messageID, const String& sender, 
                const String& recipient, String content, const String& source, 
                const String& relayID, int rssi = 0, float snr = 0.0) {
  const int maxMessageLength = 150;
  if (content.length() > maxMessageLength) {
    Serial.println("Message too long, truncating...");
    content = content.substring(0, maxMessageLength);
  }

  auto& status = messageTransmissions[messageID];

  // Early exit if message already processed.
  if (status.addedToMessages) {
    // Optionally update relayIDs if needed
    if (source == "[LoRa]") {
      // For our own message, if we get a new relayID different from our own, update relayIDs.
      String myId = getCustomNodeId(getNodeId());
      for (auto &msg : messages) {
        if (msg.messageID == messageID) {
          if (msg.nodeId == myId && relayID != myId) {
            bool exists = false;
            for (auto &id : msg.relayIDs) {
              if (id == relayID) { exists = true; break; }
            }
            if (!exists) {
              Serial.printf("Updating our message %s with relay info from %s\n", messageID.c_str(), relayID.c_str());
              msg.relayIDs.push_back(relayID);
            }
          }
          // Also, if a message received via WiFi now comes with LoRa details, update them.
          if (source == "[LoRa]" && msg.source != "[LoRa]") {
            Serial.printf("Updating message %s from WiFi to LoRa details\n", messageID.c_str());
            msg.source = "[LoRa]";
            msg.rssi   = rssi;
            msg.snr    = snr;
            msg.relayID = relayID;
          }
          break;
        }
      }
    }
    return;
  }

  // For private messages (recipient != "ALL") only add if this node is either the originator or the designated recipient.
  String myId = getCustomNodeId(getNodeId());
  if (recipient != "ALL" && myId != nodeId && myId != recipient) {
    Serial.println("Private message not for me, skipping local addition.");
    return;
  }

  String finalSource = "";
  if (nodeId != myId) {
    finalSource = source;
  }

  Message newMessage = {
    nodeId,
    sender,
    recipient,
    content,
    finalSource,
    messageID,
    relayID,
    std::vector<String>(), // Initialize relayIDs as empty
    rssi,
    snr,
    millis()
  };

  // Insert the new message at the beginning of the list.
  messages.insert(messages.begin(), newMessage);
  status.addedToMessages = true;

  // Trim the messages vector if it exceeds our maximum.
  if (messages.size() > maxMessages) {
    messages.pop_back();
  }

  Serial.printf("Message added: NodeID: %s, Sender: %s, Recipient: %s, Content: %s, Source: %s, ID: %s, RelayID: %s\n",
                nodeId.c_str(), sender.c_str(), recipient.c_str(), content.c_str(),
                finalSource.c_str(), messageID.c_str(), relayID.c_str());
}

// --- TX History Logging ---
// NEW: Structure and global vector to store TX history.
struct TxHistoryEntry {
  uint64_t timestamp;
  String type; // Emoji: "❤️" for heartbeat, "📡" for agg heartbeat, "⌨️" for our message, "🛰️" for relayed, "💬" for private message
  uint64_t txTime; // in ms
  bool success;
};

std::vector<TxHistoryEntry> txHistory;

// --- Helper: Determine TX type from message ---
String determineTxType(const String &message) {
  if (message.startsWith("AGG_HEARTBEAT|")) {
    return "📡";
  } else if (message.startsWith("HEARTBEAT|")) {
    return "❤️";
  } else {
    int pos1 = message.indexOf('|');
    int pos2 = message.indexOf('|', pos1 + 1);
    int pos3 = message.indexOf('|', pos2 + 1);
    int pos4 = message.indexOf('|', pos3 + 1);
    if (pos1 == -1 || pos2 == -1 || pos3 == -1 || pos4 == -1) {
      return "❓";
    }
    String originatorID = message.substring(pos1 + 1, pos2);
    String recipient = message.substring(pos3 + 1, pos4);
    String myId = getCustomNodeId(getNodeId());
    bool isPrivate = (recipient != "ALL");
    if (isPrivate) {
      if (originatorID == myId) {
        return "⌨️💬";
      } else {
        return "🛰️💬";
      }
    } else {
      if (originatorID == myId) {
        return "⌨️";
      } else {
        return "🛰️";
      }
    }
  }
}


// --- scheduleLoRaTransmission() updated to parse 6 fields ---
// Expected format: messageID|originatorID|sender|recipient|content|relayID|CRC
// Modified to take an additional parameter measuredRssi (default -100) so that for relayed messages we can use the measured RX RSSI.
void scheduleLoRaTransmission(String message, int measuredRssi = -100) {
    int lastSeparator = message.lastIndexOf('|');
    if (lastSeparator == -1) {
        Serial.println("[LoRa Schedule] Invalid format (no CRC).");
        return;
    }

    // Extract and validate the CRC.
    String crcStr = message.substring(lastSeparator + 1);
    String messageWithoutCRC = message.substring(0, lastSeparator);
    uint16_t receivedCRC = (uint16_t)strtol(crcStr.c_str(), NULL, 16);
    uint16_t computedCRC = crc16_ccitt((const uint8_t *)messageWithoutCRC.c_str(), messageWithoutCRC.length());
    if (receivedCRC != computedCRC) {
        Serial.printf("[LoRa Schedule] CRC mismatch. Received: %04X, Computed: %04X\n", receivedCRC, computedCRC);
        return;
    }

    // Parse the message fields.
    int firstSeparator = messageWithoutCRC.indexOf('|');
    int secondSeparator = messageWithoutCRC.indexOf('|', firstSeparator + 1);
    int thirdSeparator = messageWithoutCRC.indexOf('|', secondSeparator + 1);
    int fourthSeparator = messageWithoutCRC.indexOf('|', thirdSeparator + 1);
    int fifthSeparator = messageWithoutCRC.indexOf('|', fourthSeparator + 1);

    if (firstSeparator == -1 || secondSeparator == -1 || thirdSeparator == -1 ||
        fourthSeparator == -1 || fifthSeparator == -1) {
        Serial.println("[LoRa Schedule] Invalid message format.");
        return;
    }

    String messageID = messageWithoutCRC.substring(0, firstSeparator);
    String originatorID = messageWithoutCRC.substring(firstSeparator + 1, secondSeparator);
    String senderID = messageWithoutCRC.substring(secondSeparator + 1, thirdSeparator);
    String recipientID = messageWithoutCRC.substring(thirdSeparator + 1, fourthSeparator);
    String messageContent = messageWithoutCRC.substring(fourthSeparator + 1, fifthSeparator);
    String relayID = messageWithoutCRC.substring(fifthSeparator + 1);

    String myId = getCustomNodeId(getNodeId());

    // If this is a private message that has reached its recipient, don't relay.
    if (recipientID != "ALL" && myId == recipientID) {
        Serial.println("[LoRa Schedule] Private message reached its recipient. Not scheduling retransmission.");
        return;
    }

    auto &status = messageTransmissions[messageID];
    if (status.queuedForLoRa) {
        Serial.println("[LoRa Schedule] Message already queued for LoRa, skipping...");
        return;
    }

    // Check if the message is originating from our own node.
    bool isOwnMessage = (originatorID == myId);

    if (!isOwnMessage) {
        String newRelayID = myId;
        String updatedMessage = constructMessage(messageID, originatorID, senderID, recipientID, messageContent, newRelayID);
        loraTransmissionQueue.push_back(updatedMessage);
        status.queuedForLoRa = true;
        // Calculate dynamic slot:
        unsigned long baseSlot = getTxDuration(message); // default returns 3000 ms
        unsigned long dynamicSlot = getDynamicSlot(baseSlot, measuredRssi);
        unsigned long totalWindow = 35000; // 35 seconds total window
        int numSlots = totalWindow / dynamicSlot;
        if(numSlots < 1) numSlots = 1;
        int randomSlot = random(1, numSlots + 1);
        loRaTransmitDelay = millis() + (randomSlot * dynamicSlot);
        Serial.printf("[LoRa Schedule] Scheduled relay with dynamic slot: baseSlot=%lu, dynamicSlot=%lu, randomSlot=%d, delay=%lu ms: %s\n",
                      baseSlot, dynamicSlot, randomSlot, randomSlot * dynamicSlot, updatedMessage.c_str());
    } else {
        // For our own original message, keep the fixed 2-second delay.
        loraTransmissionQueue.push_back(message);
        status.queuedForLoRa = true;
        loRaTransmitDelay = millis() + 2000;
        Serial.printf("[LoRa Schedule] Scheduled original message with fixed 2-second delay: %s\n",
                      message.c_str());
    }
}

void transmitViaWiFi(const String& message) {
  // Early exit if already transmitted via WiFi.
  int separatorIndex = message.indexOf('|');
  if (separatorIndex == -1) {
    Serial.println("[WiFi Tx] Invalid format.");
    return;
  }
  String messageID = message.substring(0, separatorIndex);
  auto& status = messageTransmissions[messageID];
  if (status.transmittedViaWiFi) {
    Serial.println("[WiFi Tx] Already sent via WiFi, skipping...");
    return;
  }
  
  // Mark as transmitted via WiFi immediately.
  status.transmittedViaWiFi = true;
  mesh.sendBroadcast(message);
  Serial.printf("[WiFi Tx] Sent: %s\n", message.c_str());
}

bool isDutyCycleAllowed() {
  if (bypassDutyCycle) {
    dutyCycleActive = false;
    return true;
  }
  if (millis() > last_tx + minimum_pause) {
    dutyCycleActive = false;
  } else {
    dutyCycleActive = true;
  }
  return !dutyCycleActive;
}

// ----------------------------------------------------------------------------
// CAROUSEL CHANGES: Global variables for the carousel
// ----------------------------------------------------------------------------
unsigned long lastCarouselChange = 0;
const unsigned long carouselInterval = 3000; // 3 seconds each screen
int carouselIndex = 0;
long lastTxTimeMillis = -1; // store last LoRa Tx time for the display

void drawMonospacedLine(int16_t x, int16_t y, const String &line, int charWidth = 7) {
  for (uint16_t i = 0; i < line.length(); i++) {
    display.drawString(x + i * charWidth, y, String(line[i]));
  }
}

void showScrollingMonospacedAsciiArt() {
  display.clear();
  display.setFont(ArialMT_Plain_10);

  String lines[5];
  lines[0] = "M    M  EEEEE  SSSSS  H   H  M    M  I  N   N  GGGGG  L      EEEEE  ";
  lines[1] = "MM  MM  E      S      H   H  MM  MM  I  NN  N  G      L      E     ";
  lines[2] = "M MM M  EEEE   SSSSS  HHHHH  M MM M  I  N N N  G  GG  L      EEEE   ";
  lines[3] = "M    M  E          S  H   H  M    M  I  N  NN  G   G  L      E     ";
  lines[4] = "M    M  EEEEE  SSSSS  H   H  M    M  I  N   N   GGG   LLLLL  EEEEE  ";

  const int screenWidth  = 128;
  const int screenHeight = 64;
  const int lineHeight   = 10;
  const int totalBlockHeight = 5 * lineHeight;
  int verticalOffset = (screenHeight - totalBlockHeight) / 2;

  int charWidth = 7;
  uint16_t maxChars = 0;
  for (int i = 0; i < 5; i++) {
    if (lines[i].length() > maxChars) {
      maxChars = lines[i].length();
    }
  }
  int totalBlockWidth = maxChars * charWidth;

  if (totalBlockWidth <= screenWidth) {
    int offsetX = (screenWidth - totalBlockWidth) / 2;
    for (int i = 0; i < 5; i++) {
      drawMonospacedLine(offsetX, verticalOffset + i * lineHeight, lines[i], charWidth);
    }
    display.display();
    heltec_delay(3000);
    return;
  }

  const int blankStart = 20; 
  const int blankEnd   = 20; 

  for (int offset = -blankStart; offset <= (totalBlockWidth + screenWidth + blankEnd); offset += 2) {
    display.clear();
    for (int i = 0; i < 5; i++) {
      drawMonospacedLine(-offset, verticalOffset + i * lineHeight, lines[i], charWidth);
    }
    display.display();
    heltec_delay(30);
  }
}

void drawMainScreen(long txTimeMillis = -1) {
  if (txTimeMillis >= 0) {
    lastTxTimeMillis = txTimeMillis;
  }

  display.clear();
  display.setFont(ArialMT_Plain_10);

  // ——— Title ———
  const char* title = "Meshmingle 1.0";
  int16_t titleWidth = display.getStringWidth(title);
  display.drawString((128 - titleWidth) / 2, 0, title);

  // ——— Battery icon ———
  // how many cells to fill (0–4)
  int pct      = cachedBatteryPercentage;          
  int segments = (pct + 24) / 25;                  
  // top-right corner
  const int bx = 128 - 19; // 16px body + 2px cap + 1px margin
  const int by = 0;
  // outline
  display.drawRect(bx,   by, 16, 8);
  // positive terminal
  display.drawRect(bx + 16, by + 2, 2, 4);
  // fill cells
  for (int i = 0; i < 4; i++) {
    if (i < segments) {
      display.fillRect(bx + 1 + i*4, by + 1, 3, 6);
    }
  }

  // ——— Node info ———
  display.drawString(0, 13, "Node ID: " + getCustomNodeId(getNodeId()));

  // ——— Mesh counts ———
  uint64_t currentTime = millis();
  int activeDirectLoRa = 0;
  const uint64_t DIRECT_TIMEOUT = 900000; // 15 min
  for (auto& kv : loraNodes) {
    if (currentTime - kv.second.lastSeen <= DIRECT_TIMEOUT) activeDirectLoRa++;
  }
  int wifiCount = totalNodeCount;
  String combined = "WiFi: " + String(wifiCount) + "  LoRa: " + String(activeDirectLoRa);
  if (display.getStringWidth(combined) > 128) {
    combined = "WiFi:" + String(wifiCount) + " LoRa:" + String(activeDirectLoRa);
  }
  display.drawString(0, 27, combined);

  // ——— Duty cycle / TxOK ———
  display.drawString(0, 40, dutyCycleActive 
                                ? "Duty Cycle Limit Reached!"
                                : "LoRa Tx Allowed");

  if (lastTxTimeMillis >= 0) {
    String txMsg = "TxOK (" + String(lastTxTimeMillis) + " ms)";
    int16_t w = display.getStringWidth(txMsg);
    display.drawString((128 - w) / 2, 54, txMsg);
  }

  display.display();
}


void displayCarousel() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastCarouselChange >= carouselInterval) {
    lastCarouselChange = currentMillis;

    String myId = getCustomNodeId(getNodeId());
    std::vector<Message> filteredMessages;
    
    for (const auto& msg : messages) {
      if (msg.recipient != "ALL" && (msg.recipient == myId || msg.nodeId == myId)) {
        filteredMessages.push_back(msg);
      }
    }

    if (filteredMessages.empty()) {
      carouselIndex = 0;
    } else {
      carouselIndex++;
      if (carouselIndex > (int)filteredMessages.size()) {
        carouselIndex = 0;
      }
    }

    display.clear();
    display.setFont(ArialMT_Plain_10);

    if (carouselIndex == 0) {
      drawMainScreen(-1);
    } else {
      int msgIndex = carouselIndex - 1;
      if (msgIndex < (int)filteredMessages.size()) {
        String nodeLine = "Node: " + filteredMessages[msgIndex].nodeId;
        display.drawString(0, 0, nodeLine);

        String nameLine = "Name: " + filteredMessages[msgIndex].sender;
        display.drawString(0, 13, nameLine);

        display.drawString(0, 26, "[Private]");
        display.drawString(0, 36, filteredMessages[msgIndex].content);
      }
    }
    display.display();
  }
}

long lastTxTimeMillisVar = -1;

void transmitWithDutyCycle(const String& message) {
  if (!isDutyCycleAllowed()) {
    Serial.printf("[LoRa Tx] Duty cycle active. Delaying transmission for %llu ms.\n", minimum_pause);
    loRaTransmitDelay = millis() + minimum_pause;
    return;
  }

  if (millis() < loRaTransmitDelay) {
    Serial.println("[LoRa Tx] LoRa delay not expired, waiting...");
    return;
  }

  int separatorIndex = message.indexOf('|');
  if (separatorIndex == -1) {
    Serial.println("[LoRa Tx] Invalid message format.");
    return;
  }
  String messageID = message.substring(0, separatorIndex);
  
  tx_time = millis();
  Serial.printf("[LoRa Tx] Transmitting: %s\n", message.c_str());
  heltec_led(50);
  int transmitStatus = radio.transmit(message.c_str());
  tx_time = millis() - tx_time;
  heltec_led(0);

  TxHistoryEntry txEntry;
  txEntry.timestamp = millis();
  txEntry.type = determineTxType(message);
  txEntry.txTime = tx_time;

  if (transmitStatus == RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa Tx] Sent successfully (%i ms)\n", (int)tx_time);
    messageTransmissions[messageID].transmittedViaLoRa = true;
    messageTransmissions[messageID].relayedViaLoRa = true;
    messageTransmissions[messageID].queuedForLoRa = false;
    messageTransmissions[messageID].relayRetryAttempted = false;
    calculateDutyCyclePause(tx_time);
    last_tx = millis();
    drawMainScreen(tx_time);
    
    radio.startReceive();
    
    // Channel Isolation:
    // After LoRa Tx, if a WiFi relay was pending and the message did not originate via WiFi, perform it.
    if (messageTransmissions[messageID].pendingWiFiRelay && messageTransmissions[messageID].origin != ORIGIN_WIFI) {
      transmitViaWiFi(message);
      messageTransmissions[messageID].pendingWiFiRelay = false;
    }
    
    if (!loraTransmissionQueue.empty()) {
      loraTransmissionQueue.erase(loraTransmissionQueue.begin());
      Serial.printf("[LoRa Tx] Removed top queued entry for message %s\n", messageID.c_str());
    }
    
    relayLog.erase(messageID);
    
    txEntry.success = true;
  } else {
    Serial.printf("[LoRa Tx] Transmission failed with error code: %i\n", transmitStatus);
    if (!messageTransmissions[messageID].relayRetryAttempted) {
      messageTransmissions[messageID].relayRetryAttempted = true;
      // Retry branch: if the message originated via our node, keep fixed 2-second delay; else, use dynamic slot delay.
      if (messageTransmissions[messageID].origin == ORIGIN_WIFI) {
          loRaTransmitDelay = millis() + 2000;
          Serial.printf("[LoRa Tx] Scheduling a retry for own message %s with fixed 2-second delay\n", messageID.c_str());
      } else {
          unsigned long baseSlot = getTxDuration(message); // default 3000 ms if not available
          unsigned long dynamicSlot = getDynamicSlot(baseSlot, -100); // For retry, if no new measurement, use default RSSI of -100
          unsigned long totalWindow = 35000; // 35 seconds total window
          int numSlots = totalWindow / dynamicSlot;
          if(numSlots < 1) numSlots = 1;
          int randomSlot = random(1, numSlots + 1);
          loRaTransmitDelay = millis() + (randomSlot * dynamicSlot);
          Serial.printf("[LoRa Tx] Scheduling a retry for message %s in %lu ms (dynamic slot, random slot %d)\n", 
                        messageID.c_str(), loRaTransmitDelay - millis(), randomSlot);
      }
    } else {
      messageTransmissions[messageID].queuedForLoRa = false;
      if (!loraTransmissionQueue.empty()) {
        loraTransmissionQueue.erase(loraTransmissionQueue.begin());
        Serial.printf("[LoRa Tx] Removing top queued entry for message %s after retry failure\n", messageID.c_str());
      }
    }
    txEntry.success = false;
  }
  txHistory.push_back(txEntry);
  if(txHistory.size() > 100) {
    txHistory.erase(txHistory.begin());
  }
}

// ----------------------------
// NEW: Heartbeat scheduling with random jitter
// ----------------------------
const unsigned long firstHeartbeatDelayValue = 20000; // 20 seconds for first heartbeat
const unsigned long heartbeatIntervalValue = 900000;    // 15 minutes for subsequent heartbeats
unsigned long nextHeartbeatTime = 0;
unsigned long nextAggregatedHeartbeatTime = 0;

void sendHeartbeat() {
  String heartbeatWithoutCRC = "HEARTBEAT|" + getCustomNodeId(getNodeId());
  uint16_t crc = crc16_ccitt((const uint8_t *)heartbeatWithoutCRC.c_str(), heartbeatWithoutCRC.length());
  char crcStr[5];
  sprintf(crcStr, "%04X", crc);
  String heartbeatMessage = heartbeatWithoutCRC + "|" + String(crcStr);

  if (!isDutyCycleAllowed()) {
    Serial.println("[Heartbeat Tx] Duty cycle limit reached, skipping.");
    return;
  }

  if (radio.available()) {
    Serial.println("[Heartbeat Tx] Radio is busy receiving a packet. Delaying heartbeat by 500ms.");
    return;
  }

  uint64_t txStart = millis();
  Serial.printf("[Heartbeat Tx] Sending: %s\n", heartbeatMessage.c_str());
  heltec_led(50);
  int transmitStatus = radio.transmit(heartbeatMessage.c_str());
  uint64_t txTime = millis() - txStart;
  heltec_led(0);

  {
    TxHistoryEntry entry;
    entry.timestamp = millis();
    entry.type = "❤️";
    entry.txTime = txTime;
    entry.success = (transmitStatus == RADIOLIB_ERR_NONE);
    txHistory.push_back(entry);
    if (txHistory.size() > 100) txHistory.erase(txHistory.begin());
  }

  if (transmitStatus == RADIOLIB_ERR_NONE) {
    Serial.printf("[Heartbeat Tx] Sent successfully (%llu ms)\n", txTime);
    calculateDutyCyclePause(txTime);
    last_tx = millis();
    drawMainScreen(txTime);
    radio.startReceive();
  } else {
    Serial.printf("[Heartbeat Tx] Failed with error code: %i\n", transmitStatus);
  }
}

void sendAggregatedHeartbeat() {
  if (!sendAggregatedHeartbeats) {
    Serial.println("[Aggregated Heartbeat Tx] Aggregated heartbeats are disabled by global flag.");
    return;
  }
  
  String aggMsgWithoutCRC = "AGG_HEARTBEAT|" + getCustomNodeId(getNodeId());
  
  auto directNodes = mesh.getNodeList();
  for (uint32_t node : directNodes) {
    String nodeIdStr = getCustomNodeId(node);
    if (nodeIdStr != getCustomNodeId(getNodeId())) {
      aggMsgWithoutCRC += "|" + nodeIdStr;
    }
  }
  
  uint16_t crc = crc16_ccitt((const uint8_t *)aggMsgWithoutCRC.c_str(), aggMsgWithoutCRC.length());
  char crcStr[5];
  sprintf(crcStr, "%04X", crc);
  String aggMessage = aggMsgWithoutCRC + "|" + String(crcStr);
  
  if (!isDutyCycleAllowed()) {
    Serial.println("[Aggregated Heartbeat Tx] Duty cycle limit reached, skipping.");
    return;
  }
  
  if (radio.available()) {
    Serial.println("[Aggregated Heartbeat Tx] Radio busy, delaying aggregated heartbeat.");
    return;
  }
  
  uint64_t txStart = millis();
  Serial.printf("[Aggregated Heartbeat Tx] Sending: %s\n", aggMessage.c_str());
  heltec_led(50);
  int transmitStatus = radio.transmit(aggMessage.c_str());
  uint64_t txTime = millis() - txStart;
  heltec_led(0);

  {
    TxHistoryEntry entry;
    entry.timestamp = millis();
    entry.type = "📡";
    entry.txTime = txTime;
    entry.success = (transmitStatus == RADIOLIB_ERR_NONE);
    txHistory.push_back(entry);
    if (txHistory.size() > 100) txHistory.erase(txHistory.begin());
  }
  
  if (transmitStatus == RADIOLIB_ERR_NONE) {
    Serial.printf("[Aggregated Heartbeat Tx] Sent successfully (%llu ms)\n", txTime);
    calculateDutyCyclePause(txTime);
    last_tx = millis();
    drawMainScreen(txTime);
    radio.startReceive();
  } else {
    Serial.printf("[Aggregated Heartbeat Tx] Failed with error code: %i\n", transmitStatus);
  }
}

// --------------------------------------------------------------------------
// IMPORTANT: Callback for radio RX events. (Renamed to avoid conflict.)
// --------------------------------------------------------------------------
void onRadioRx() {
  rxFlag = true;
}

void setup() {
  loadConfig();
  mesh.init(cfg_ssid.c_str(),           // <— from settings_feature
          cfg_pass.c_str(),
          MESH_PORT,
          WIFI_AP_STA,
          cfg_channel);
  bypassDutyCycle = cfg_duty_override;
  heltec_setup();
  Serial.printf(">> Loaded mesh/Wi-Fi channel: %u\n", cfg_channel);
  heltec_led(0);
#ifdef HELTEC_V3
  // (nothing special for V3)
#else
  // V3.2 only:
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  heltec_delay(100);
#endif
  // Prepare ADC and gate-control pin
  pinMode(ADC_Ctrl, OUTPUT);
  digitalWrite(ADC_Ctrl, FET_DISCONNECT);
  pinMode(VBAT_Read, INPUT);
  adcAttachPin(VBAT_Read);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);     // ← explicit, even though it's the default
  display.init();
#ifdef HELTEC_V3
  // (V3 uses default contrast)
#else
  // V3.2 boost:
  display.setContrast(200);
#endif
  display.flipScreenVertically();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  drawMainScreen();

  showScrollingMonospacedAsciiArt();
  Serial.println("Setup complete. Starting battery readings...");
  Serial.printf(">>> LoRa enabled? %s\n", cfg_lora_enabled ? "YES" : "NO");
// LoRa initialization wrapped in the new toggle
if (cfg_lora_enabled) {
  Serial.println("[LoRa] Initializing radio...");
  RADIOLIB_OR_HALT(radio.begin());
  radio.setDio1Action(onRadioRx);

  // collapse the two branches into one call
  RADIOLIB_OR_HALT(radio.setRxBoostedGainMode(enableRxBoost));

  RADIOLIB_OR_HALT(radio.setFrequency(cfg_freq_mhz));
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  RADIOLIB_OR_HALT(radio.setCodingRate(CODING_RATE));
  RADIOLIB_OR_HALT(radio.setOutputPower(cfg_lora_power));
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
}else{
  Serial.println("[LoRa] Disabled by configuration.");
}

  heltec_delay(2000);
  WiFi.softAP(cfg_ssid.c_str(), cfg_pass.c_str(), cfg_channel);   // if you keep a manual Soft-AP
  ourApMac = WiFi.softAPmacAddress();
  ourApMac.toUpperCase();            // so it matches macToString()
  //WiFi.softAP(MESH_SSID, MESH_PASSWORD);  // removed because we was on ch1 then 3
  WiFi.setTxPower(cfg_wifi_power);
  WiFi.setSleep(false);
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.onReceive(receivedCallback);
  mesh.onChangedConnections([]() {
    updateMeshData();
  });
  //mesh.setContainsRoot(false); this was causing us to never have a root or have one auto assigned.

  setupServerRoutes();
  setupSettingsRoutes();
  server.begin();
  dnsServer.start(53, "*", WiFi.softAPIP());

  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  uint64_t chipid = ESP.getEfuseMac();
  uint32_t seed = (uint32_t)(chipid & 0xFFFFFFFF) ^ (uint32_t)(chipid >> 16);
  randomSeed(seed);
  Serial.printf("Random seed set using improved chip ID: 0x%08X\n", seed);

  uint32_t chip_offset = (uint32_t)(ESP.getEfuseMac() & 0xFFFF);
  global_jitter_offset = chip_offset % 10000;
  nextHeartbeatTime = millis() + firstHeartbeatDelayValue + global_jitter_offset;
  nextAggregatedHeartbeatTime = millis() + aggregatedHeartbeatInterval + global_jitter_offset;

// Prime the battery cache immediately
lastBatteryCacheUpdate = millis() - batteryCacheInterval;

// ——— Persistent raw min/max init ———
prefs.begin("battery", false);
// default vMinRaw to the measured empty voltage, vMaxRaw to the measured full voltage
vMinRaw = prefs.getFloat("vMinRaw", measuredV0);
vMaxRaw = prefs.getFloat("vMaxRaw", measuredV);
recalcCalibration();
primeBatteryCacheCalibrated();
Serial.printf("Loaded vMinRaw: %.3f V, vMaxRaw: %.3f V\n", vMinRaw, vMaxRaw);

}

void loop() {
  esp_task_wdt_reset();
  heltec_loop();
  mesh.update();
    // ——— RX keeper ———
if (cfg_lora_enabled
    && (millis() - lastRxKick > RX_KICK_MS)) {
  radio.standby();                                   // drop out of any half-state
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  lastRxKick = millis();
  Serial.println("[RX-Kick] SX1262 receiver refreshed");
}

  // ——— cache & calibrate battery every 60 seconds ———
  if (millis() - lastBatteryCacheUpdate >= batteryCacheInterval) {
    lastBatteryCacheUpdate = millis();

    // 1) raw divider reading in volts
    float rawV = readBatteryVoltage() / 1000.0f;

    // 2a) update raw MIN only if it's at least 5 mV below the saved value
    if (rawV <= vMinRaw - updateThreshold) {
      vMinRaw = rawV;
      prefs.putFloat("vMinRaw", vMinRaw);
      Serial.printf("New vMinRaw (saved): %.3f V\n", vMinRaw);
    }

    // 2b) update raw MAX (with calibrationOffset) only if it's at least 5 mV above
    float candidateMax = rawV - calibrationOffset;
    if (candidateMax >= vMaxRaw + updateThreshold) {
      vMaxRaw = candidateMax;
      prefs.putFloat("vMaxRaw", vMaxRaw);
      Serial.printf("New vMaxRaw (saved): %.3f V\n", vMaxRaw);
    }

    // 3) map rawV → calibrated [nominalVmin…nominalVmax]
    float calV = nominalVmin
               + (rawV - vMinRaw)
                 * (nominalVmax - nominalVmin)
                 / (vMaxRaw - vMinRaw);
    calV = constrain(calV, nominalVmin, nominalVmax);

    // 4) update your displayed/cache values
    cachedBatteryVoltage    = calV;
    cachedBatteryPercentage = constrain(
      int((calV - nominalVmin)
          / (nominalVmax - nominalVmin)
          * 100.0f),
      0, 100
    );
    Serial.printf("Cached battery: %.3f V (%d%%)\n",
                  cachedBatteryVoltage, cachedBatteryPercentage);
  }

  if (millis() >= nextHeartbeatTime) {
    sendHeartbeat();
    nextHeartbeatTime = millis() + heartbeatIntervalValue + global_jitter_offset;
  }
  if (millis() >= nextAggregatedHeartbeatTime) {
    sendAggregatedHeartbeat();
    nextAggregatedHeartbeatTime = millis() + aggregatedHeartbeatInterval + global_jitter_offset;
  }

if (cfg_lora_enabled) {
  if (rxFlag) {
    rxFlag = false;
    String message;
    int state = radio.readData(message);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.printf("[LoRa Rx] %s\n", message.c_str());

      int lastSeparatorIndex = message.lastIndexOf('|');
      if (lastSeparatorIndex == -1) {
        Serial.println("[LoRa Rx] Invalid format (no CRC).");
      } else {
        String crcStr = message.substring(lastSeparatorIndex + 1);
        String messageWithoutCRC = message.substring(0, lastSeparatorIndex);

        uint16_t receivedCRC = (uint16_t)strtol(crcStr.c_str(), NULL, 16);
        uint16_t computedCRC = crc16_ccitt((const uint8_t *)messageWithoutCRC.c_str(), messageWithoutCRC.length());

        if (receivedCRC != computedCRC) {
          Serial.printf("[LoRa Rx] CRC mismatch. Recv: %04X, Computed: %04X\n", receivedCRC, computedCRC);
        } else {
          Serial.println("[LoRa Rx] CRC valid.");

          if (!messageWithoutCRC.startsWith("HEARTBEAT|") && !messageWithoutCRC.startsWith("AGG_HEARTBEAT|")) {
            int firstSeparator = messageWithoutCRC.indexOf('|');
            if (firstSeparator == -1) {
              Serial.println("[LoRa Rx] Invalid message format.");
              radio.startReceive();
              return;
            }
            String messageID = messageWithoutCRC.substring(0, firstSeparator);
            if (!messageID.startsWith("!M")) {
              Serial.println("[LoRa Rx] Foreign message detected, ignoring.");
              radio.startReceive();
              return;
            }
          }

          if (messageWithoutCRC.startsWith("HEARTBEAT|")) {
            String senderNodeId = messageWithoutCRC.substring(strlen("HEARTBEAT|"));
            Serial.printf("[LoRa Rx] Heartbeat from %s\n", senderNodeId.c_str());
            int rssi = radio.getRSSI();
            float snr = radio.getSNR();
            uint64_t currentTime = millis();
            // For heartbeat packets we update here (if needed)
            if (senderNodeId != getCustomNodeId(getNodeId())) {
              LoRaNode& node = loraNodes[senderNodeId];
              node.nodeId = senderNodeId;
              node.lastRSSI = rssi;
              node.lastSNR = snr;
              node.lastSeen = currentTime;

              NodeMetricsSample sample = { currentTime, rssi, snr };
              node.history.push_back(sample);
              if (node.history.size() > 60) {
                node.history.erase(node.history.begin());
              }
              node.statusEmoji = "❤️";
              Serial.printf("[LoRa Nodes] Updated/Added node: %s (Heartbeat)\n", senderNodeId.c_str());
            } else {
              Serial.println("[LoRa Rx] Own heartbeat, ignore.");
            }
          }
          else if (messageWithoutCRC.startsWith("AGG_HEARTBEAT|")) {
            Serial.printf("[LoRa Rx] Aggregated Heartbeat received: %s\n", messageWithoutCRC.c_str());
            int firstSep = messageWithoutCRC.indexOf('|');
            if (firstSep == -1) {
              Serial.println("[LoRa Rx] Invalid AGG_HEARTBEAT format.");
            } else {
              String senderRelayId = messageWithoutCRC.substring(firstSep + 1, messageWithoutCRC.indexOf('|', firstSep + 1));
              
              if (senderRelayId != getCustomNodeId(getNodeId())) {
                LoRaNode &node = loraNodes[senderRelayId];
                node.nodeId = senderRelayId;
                int rssi = radio.getRSSI();
                float snr = radio.getSNR();
                uint64_t currentTime = millis();
                node.lastRSSI = rssi;
                node.lastSNR = snr;
                node.lastSeen = currentTime;
                
                NodeMetricsSample sample = { currentTime, rssi, snr };
                node.history.push_back(sample);
                if (node.history.size() > 60) {
                  node.history.erase(node.history.begin());
                }
                node.statusEmoji = "📡";
                Serial.printf("[LoRa Nodes] Updated/Added node: %s (Aggregated Heartbeat)\n", senderRelayId.c_str());
              }
              
              int startPos = messageWithoutCRC.indexOf('|', firstSep + 1) + 1;
              while (true) {
                int nextSep = messageWithoutCRC.indexOf('|', startPos);
                String neighborId;
                if (nextSep == -1) {
                  neighborId = messageWithoutCRC.substring(startPos);
                } else {
                  neighborId = messageWithoutCRC.substring(startPos, nextSep);
                }
                
                if (neighborId.length() > 0 && neighborId != getCustomNodeId(getNodeId())) {
                  uint64_t currentTime = millis();
                  int rssi = radio.getRSSI();
                  float snr = radio.getSNR();
                  String key = neighborId + "-" + senderRelayId;
                  auto it = indirectNodes.find(key);
                  if (it != indirectNodes.end()) {
                    it->second.lastSeen = currentTime;
                    it->second.rssi = rssi;
                    it->second.snr = snr;
                  } else {
                    IndirectNode indNode;
                    indNode.originatorId = neighborId;
                    indNode.relayId = senderRelayId;
                    indNode.rssi = rssi;
                    indNode.snr = snr;
                    indNode.lastSeen = currentTime;
                    indNode.statusEmoji = "🛰️";
                    indirectNodes[key] = indNode;
                  }
                  Serial.printf("[Indirect Nodes] Updated 1-hop indirect node: Originator: %s, via Relay: %s, RSSI: %d, SNR: %.2f\n",
                                neighborId.c_str(), senderRelayId.c_str(), rssi, snr);
                }
                
                if (nextSep == -1) break;
                startPos = nextSep + 1;
              }
            }
          }
          else {
            int firstSeparator = messageWithoutCRC.indexOf('|');
            int secondSeparator = messageWithoutCRC.indexOf('|', firstSeparator + 1);
            int thirdSeparator = messageWithoutCRC.indexOf('|', secondSeparator + 1);
            int fourthSeparator = messageWithoutCRC.indexOf('|', thirdSeparator + 1);
            int fifthSeparator = messageWithoutCRC.indexOf('|', fourthSeparator + 1);

            if (firstSeparator == -1 || secondSeparator == -1 || thirdSeparator == -1 ||
                fourthSeparator == -1 || fifthSeparator == -1) {
              Serial.println("[LoRa Rx] Invalid format.");
            } else {
              String messageID = messageWithoutCRC.substring(0, firstSeparator);
              String originatorID = messageWithoutCRC.substring(firstSeparator + 1, secondSeparator);
              String senderID = messageWithoutCRC.substring(secondSeparator + 1, thirdSeparator);
              String recipientID = messageWithoutCRC.substring(thirdSeparator + 1, fourthSeparator);
              String messageContent = messageWithoutCRC.substring(fourthSeparator + 1, fifthSeparator);
              String relayID = messageWithoutCRC.substring(fifthSeparator + 1);

              int rssi = radio.getRSSI();
              float snr = radio.getSNR();

              String myId = getCustomNodeId(getNodeId());

              auto &status = messageTransmissions[messageID];
              status.origin = ORIGIN_LORA;

              if (originatorID == myId && relayID == myId) {
                Serial.println("[LoRa Rx] Own original message, ignore.");
              } else {
                if (recipientID == "ALL" || myId == originatorID || myId == recipientID) {
                  addMessage(originatorID, messageID, senderID, recipientID, messageContent, "[LoRa]", relayID, rssi, snr);
                } else {
                  Serial.println("[LoRa Rx] Private message not for me, ignoring display.");
                }
                if (!status.queuedForLoRa && !status.relayedViaLoRa) {
                  // Pass the measured rssi from this message into scheduleLoRaTransmission
                  scheduleLoRaTransmission(message, rssi);
                }
                uint64_t currentTime = millis();

                if (relayID != myId) {
                  LoRaNode& relayNode = loraNodes[relayID];
                  relayNode.nodeId = relayID;
                  relayNode.lastRSSI = rssi;
                  relayNode.lastSNR = snr;
                  relayNode.lastSeen = currentTime;

                  NodeMetricsSample sample = { currentTime, rssi, snr };
                  relayNode.history.push_back(sample);
                  if (relayNode.history.size() > 60) {
                    relayNode.history.erase(relayNode.history.begin());
                  }
                  if (relayID == originatorID) {
                    relayNode.statusEmoji = "⌨️";
                  } else {
                    relayNode.statusEmoji = "🛰️";
                  }
                  Serial.printf("[LoRa Nodes] Updated/Added node: %s\n", relayID.c_str());
                } else {
                  Serial.println("[LoRa Nodes] RelayID is own node, not updating.");
                }

                if (originatorID != myId && relayID != myId && relayID != originatorID) {
                  const uint64_t ACTIVE_THRESHOLD = 960000; // 16 minutes in milliseconds

                  bool seenDirectly = false;
                  auto directIt = loraNodes.find(originatorID);
                  if (directIt != loraNodes.end()) {
                      if (millis() - directIt->second.lastSeen <= ACTIVE_THRESHOLD) {
                          seenDirectly = true;
                      }
                  }

                  if (seenDirectly) {
                      Serial.printf("[Indirect Nodes] Skipped indirect update for %s because it was seen directly within the last 15 minutes.\n", originatorID.c_str());
                  } else {
                      uint64_t currentTime = millis();
                      NodeMetricsSample sample = { currentTime, rssi, snr };
                      String key = originatorID + "-" + relayID;
                      auto it = indirectNodes.find(key);
                      if (it != indirectNodes.end()) {
                          it->second.lastSeen = currentTime;
                          it->second.rssi = rssi;
                          it->second.snr = snr;
                          it->second.history.push_back(sample);
                          if (it->second.history.size() > 60) {
                              it->second.history.erase(it->second.history.begin());
                          }
                      } else {
                          IndirectNode indNode;
                          indNode.originatorId = originatorID;
                          indNode.relayId = relayID;
                          indNode.rssi = rssi;
                          indNode.snr = snr;
                          indNode.lastSeen = currentTime;
                          indNode.statusEmoji = "🛰️";
                          indNode.history.push_back(sample);
                          indirectNodes[key] = indNode;
                      }
                      Serial.printf("[Indirect Nodes] Updated indirect node: Originator: %s, Relay: %s, RSSI: %d, SNR: %.2f\n",
                                    originatorID.c_str(), relayID.c_str(), rssi, snr);
                  }
                }
              }

              bool alreadyLogged = false;
              if (relayLog.find(messageID) != relayLog.end()) {
                for (auto &entry : relayLog[messageID]) {
                  if (entry.relayID == relayID) { 
                    alreadyLogged = true; 
                    break; 
                  }
                }
                if (!alreadyLogged) {
                  RelayLogEntry newEntry = { relayID, millis() };
                  relayLog[messageID].push_back(newEntry);
                  Serial.printf("[Relay Log] New relay %s for message %s added.\n", relayID.c_str(), messageID.c_str());
                }
              } else {
                relayLog[messageID] = { { relayID, millis() } };
                Serial.printf("[Relay Log] Created new relay log for message %s with relay %s.\n", messageID.c_str(), relayID.c_str());
              }
              
              if (!status.relayedViaLoRa && !status.queuedForLoRa) {
                  Serial.printf("[Relay Log] Forcing relay attempt for message %s due to new relay log entry.\n", messageID.c_str());
                  scheduleLoRaTransmission(message, rssi);
              }
            }
          }
        }
        radio.startReceive();
      }
    } else {
      Serial.printf("[LoRa Rx] Receive failed, code %d\n", state);
      radio.startReceive();
    }
  }

  if (!loraTransmissionQueue.empty() && millis() >= loRaTransmitDelay) {
    transmitWithDutyCycle(loraTransmissionQueue.front());
  }
}
  updateMeshData();
  displayCarousel();
  dnsServer.processNextRequest();

  if (millis() - lastCleanupTime >= cleanupInterval) {
    cleanupLoRaNodes();
    cleanupIndirectNodes();
    cleanupPendingTxQueue();
    cleanupTransmissionHistory();
    lastCleanupTime = millis();
  }
}

void initMesh() {
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.init(cfg_ssid.c_str(), cfg_pass.c_str(), MESH_PORT, WIFI_AP_STA, cfg_channel);
  mesh.onReceive(receivedCallback);

  mesh.onChangedConnections([]() {
    updateMeshData();
  });

  mesh.setContainsRoot(false);
}

uint16_t readBatteryVoltage() {
  // ADC & divider parameters
  const int resolution = 12;
  const int adcMax = (1 << resolution) - 1;
  const float adcRef = 3.3f;
  const float R1 = 390.0f;
  const float R2 = 100.0f;
// (we already declared measuredV, reportedV, reportedV0, measuredV0 as globals above)

  // Optimize sampling: connect once, take quick reads, then disconnect
  const int samples = 5;
  long sumRaw = 0;
  digitalWrite(ADC_Ctrl, FET_CONNECT);
  heltec_delay(5);
  for (int i = 0; i < samples; ++i) {
    sumRaw += analogRead(VBAT_Read);
    // tiny pause so ADC stabilizes; can be omitted if too slow
    // heltec_delay(1);
  }
  digitalWrite(ADC_Ctrl, FET_DISCONNECT);

  float avgRaw = sumRaw / float(samples);
// map raw ADC reading up to real battery volts using the global calibration
  float factor = (adcRef / adcMax) * ((R1 + R2) / R2) * ( ::measuredV / ::reportedV );
  float volts = avgRaw * factor;
  return uint16_t(volts * 1000.0f);
}

void receivedCallback(uint32_t from, String& message) {
  Serial.printf("[WiFi Rx] From %s: %s\n", getCustomNodeId(from).c_str(), message.c_str());

  int lastSeparatorIndex = message.lastIndexOf('|');
  if (lastSeparatorIndex == -1) {
    Serial.println("[WiFi Rx] Invalid format (no CRC).");
    return;
  }
  String crcStr = message.substring(lastSeparatorIndex + 1);
  String messageWithoutCRC = message.substring(0, lastSeparatorIndex);

  uint16_t receivedCRC = (uint16_t)strtol(crcStr.c_str(), NULL, 16);
  uint16_t computedCRC = crc16_ccitt((const uint8_t *)messageWithoutCRC.c_str(), messageWithoutCRC.length());

  if (receivedCRC != computedCRC) {
    Serial.printf("[WiFi Rx] CRC mismatch. Recv: %04X, Computed: %04X\n", receivedCRC, computedCRC);
    return;
  } else {
    Serial.println("[WiFi Rx] CRC valid.");
  }

  if (messageWithoutCRC.startsWith("HEARTBEAT|") || messageWithoutCRC.startsWith("AGG_HEARTBEAT|")) {
    Serial.println("[WiFi Rx] Skipping heartbeat relay over WiFi.");
    return;
  }

  int firstSeparator = messageWithoutCRC.indexOf('|');
  if (firstSeparator == -1) {
    Serial.println("[WiFi Rx] Invalid message format.");
    return;
  }
  String messageID = messageWithoutCRC.substring(0, firstSeparator);
  if (!messageWithoutCRC.startsWith("HEARTBEAT|") && !messageID.startsWith("!M")) {
    Serial.println("[WiFi Rx] Foreign message detected, ignoring.");
    return;
  }

  int secondSeparator = messageWithoutCRC.indexOf('|', firstSeparator + 1);
  int thirdSeparator = messageWithoutCRC.indexOf('|', secondSeparator + 1);
  int fourthSeparator = messageWithoutCRC.indexOf('|', thirdSeparator + 1);
  int fifthSeparator = messageWithoutCRC.indexOf('|', fourthSeparator + 1);

  if (firstSeparator == -1 || secondSeparator == -1 || thirdSeparator == -1 ||
      fourthSeparator == -1 || fifthSeparator == -1) {
    Serial.println("[WiFi Rx] Invalid format (missing fields).");
    return;
  }

  messageID = messageWithoutCRC.substring(0, firstSeparator);
  String originatorID = messageWithoutCRC.substring(firstSeparator + 1, secondSeparator);
  String senderID = messageWithoutCRC.substring(secondSeparator + 1, thirdSeparator);
  String recipientID = messageWithoutCRC.substring(thirdSeparator + 1, fourthSeparator);
  String messageContent = messageWithoutCRC.substring(fourthSeparator + 1, fifthSeparator);
  String relayID = messageWithoutCRC.substring(fifthSeparator + 1);

  String myId = getCustomNodeId(getNodeId());
  if (originatorID == myId) {
    addMessage(originatorID, messageID, senderID, recipientID, messageContent, "[WiFi]", relayID);
  } else if (recipientID == "ALL" || myId == recipientID) {
    addMessage(originatorID, messageID, senderID, recipientID, messageContent, "[WiFi]", relayID);
  } else {
    Serial.println("[WiFi Rx] Private message not for me, ignoring display.");
  }

  auto &status = messageTransmissions[messageID];
  // --- Added lines to mark WiFi origin ---
  status.origin = ORIGIN_WIFI;
  status.transmittedViaWiFi = true;

  if (!status.queuedForLoRa && !status.relayedViaLoRa) {
    // For WiFi messages, we call scheduleLoRaTransmission with default measured RSSI (-100)
    scheduleLoRaTransmission(message, -100);
  }
}

const char mainPageHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Meshmingle Public Chat</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #f4f7f6;
      margin: 0;
      padding: 0;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: flex-start;
      height: 100vh;
      overflow: hidden;
    }
    .warning {
      color: red;
      font-weight: normal;
      font-size: 0.9em;
      padding: 10px;
      background-color: #fff3f3;
      border: 1px solid red;
      max-width: 100%;
      text-align: center;
      border-radius: 5px;
      margin: 0;
    }
    h2 {
      color: #333;
      margin: 10px 0 5px;
      font-size: 1.2em;
    }
    #chat-container {
      background-color: #fff;
      width: 100%;
      max-width: 600px;
      height: calc(100vh - 250px);
      margin-top: 10px;
      box-shadow: 0 0 10px rgba(0,0,0,0.1);
      overflow-y: auto;
      padding: 10px;
      border-radius: 10px;
      box-sizing: border-box;
    }
    #messageForm {
      width: 100%;
      max-width: 600px;
      display: flex;
      position: fixed;
      bottom: 0;
      background-color: #fff;
      padding: 10px;
      box-shadow: 0 -2px 10px rgba(0, 0, 0, 0.1);
      box-sizing: border-box;
      flex-wrap: wrap;
      gap: 5px;
    }
    #nameInput, #messageInput, #targetInput {
      padding: 10px;
      border: 1px solid #ccc;
      border-radius: 5px;
      box-sizing: border-box;
    }
    #nameInput {
      width: 30%;
    }
    #targetInput {
      width: 30%;
    }
    #messageInput {
      width: 30%;
    }
    #messageForm input[type=submit] {
      background-color: #007bff;
      color: white;
      border: none;
      padding: 10px;
      cursor: pointer;
      border-radius: 5px;
    }
    #messageList {
      list-style-type: none;
      padding: 0;
      margin: 0;
      display: flex;
      flex-direction: column;
    }
    .message {
      display: flex;
      flex-direction: column;
      margin: 5px 0;
      padding: 8px;
      border-radius: 5px;
      width: 80%;
      box-sizing: border-box;
      border: 2px solid;
      word-wrap: break-word;
      position: relative;
    }
    .message.sent {
      align-self: flex-end;
      border-color: green;
      background-color: #eaffea;
    }
    .message.received.wifi {
      align-self: flex-start;
      border-color: blue;
      background-color: #e7f0ff;
    }
    .message.received.lora {
      align-self: flex-start;
      border-color: orange;
      background-color: #fff4e0;
    }
    .message-nodeid {
      font-size: 0.7em;
      color: #666;
    }
    .message-relayid {
      font-size: 0.7em;
      color: #555;
      margin-bottom: 2px;
      display: block;
    }
    .message-rssi-snr {
      font-size: 0.7em;
      color: #666;
      text-align: right;
      margin-top: 2px;
    }
    .message-content {
      font-size: 0.85em;
      color: #333;
    }
    .message-time {
      font-size: 0.7em;
      color: #999;
      text-align: right;
      margin-top: 5px;
    }
    .message.sent .relay-status {
      position: absolute;
      bottom: 2px;
      left: 2px;
      line-height: 1;
    }
    .message.sent .relay-status .circle {
      font-size: 0.75em;
    }
    .message.sent .relay-status .satellite,
    .message.sent .relay-status .keyboard {
      font-size: 1em;
    }
    #deviceCount {
      margin: 5px 0;
      font-weight: normal;
      font-size: 0.9em;
    }
    #deviceCount a {
      color: #007bff;
      text-decoration: none;
    }
    #deviceCount a:hover {
      text-decoration: underline;
    }
  </style>
<script>
  let deviceCurrentTime = 0;

  function sendMessage(event) {
    event.preventDefault();

    const nameInput = document.getElementById('nameInput');
    const targetInput = document.getElementById('targetInput');
    const messageInput = document.getElementById('messageInput');
    const sendButton = document.getElementById('sendButton');

    const sender = nameInput.value;
    const target = targetInput.value;
    const msg = messageInput.value;

    if (!sender || !msg) {
      alert('Please enter both a name and a message.');
      return;
    }

    localStorage.setItem('username', sender);

    const formData = new URLSearchParams();
    formData.append('sender', sender);
    formData.append('msg', msg);
    formData.append('target', target);

    sendButton.disabled = true;
    let countdown = 15;
    sendButton.value = `Wait ${countdown}s`;

    const countdownInterval = setInterval(() => {
      countdown--;
      sendButton.value = `Wait ${countdown}s`;
      if (countdown <= 0) {
        clearInterval(countdownInterval);
        sendButton.disabled = false;
        sendButton.value = 'Send';
      }
    }, 1000);

    fetch('/update', {
      method: 'POST',
      body: formData
    })
      .then(response => {
        if (!response.ok) throw new Error('Failed to send message');
        messageInput.value = '';
        targetInput.value = '';
        fetchData();
      })
      .catch(error => {
        console.error('Error sending message:', error);
        clearInterval(countdownInterval);
        sendButton.disabled = false;
        sendButton.value = 'Send';
        alert('Failed to send message. Please try again.');
      });
  }

  function fetchData() {
    fetch('/messages')
      .then(response => response.json())
      .then(data => {
        deviceCurrentTime = data.currentDeviceTime;
        const ul = document.getElementById('messageList');
        ul.innerHTML = '';
        const currentNodeId = localStorage.getItem('nodeId');

        data.messages.forEach(msg => {
          if (msg.recipient && msg.recipient !== "ALL" &&
              msg.nodeId !== currentNodeId && msg.recipient !== currentNodeId) {
            return;
          }

          const li = document.createElement('li');
          li.classList.add('message');
          const isSentByCurrentNode = msg.nodeId === currentNodeId;
          if (isSentByCurrentNode) {
            li.classList.add('sent');
          } else {
            li.classList.add('received');
            if (msg.source === '[LoRa]') {
              li.classList.add('lora');
            } else {
              li.classList.add('wifi');
            }
          }

          const messageAgeMillis = deviceCurrentTime - msg.timeReceived;
          const messageAgeSeconds = Math.floor(messageAgeMillis / 1000);
          let timestamp = '';
          if (messageAgeSeconds < 60) {
            timestamp = `${messageAgeSeconds} sec ago`;
          } else if (messageAgeSeconds < 3600) {
            const minutes = Math.floor(messageAgeSeconds / 60);
            timestamp = `${minutes} min ago`;
          } else if (messageAgeSeconds < 86400) {
            const hours = Math.floor(messageAgeSeconds / 3600);
            const minutes = Math.floor((messageAgeSeconds % 3600) / 60);
            timestamp = minutes > 0 ? `${hours} hr ${minutes} min ago` : `${hours} hr ago`;
          } else if (messageAgeSeconds < 604800) {
            const days = Math.floor(messageAgeSeconds / 86400);
            timestamp = `${days} day${days > 1 ? 's' : ''} ago`;
          } else if (messageAgeSeconds < 2592000) {
            const weeks = Math.floor(messageAgeSeconds / 604800);
            timestamp = `${weeks} week${weeks > 1 ? 's' : ''} ago`;
          } else if (messageAgeSeconds < 31536000) {
            const months = Math.floor(messageAgeSeconds / 2592000);
            timestamp = `${months} month${months > 1 ? 's' : ''} ago`;
          } else {
            const years = Math.floor(messageAgeSeconds / 31536000);
            timestamp = `${years} year${years > 1 ? 's' : ''} ago`;
          }

          const nodeIdHtml = `Node Id: ${msg.nodeId}`;
          const senderHtml = `<strong>${msg.sender || 'Unknown'}:</strong> `;
          let privateIndicator = "";
          if (msg.recipient && msg.recipient !== "ALL") {
            privateIndicator = `<span style="color:red;">[Private to ${msg.recipient}]</span> `;
          }

          let rssiSnrHtml = "";
          if (msg.source === "[LoRa]" && msg.rssi !== undefined && msg.snr !== undefined) {
            rssiSnrHtml = `<span class="message-rssi-snr">RSSI: ${msg.rssi} dBm, SNR: ${msg.snr} dB</span>`;
          }

          let relayHtml = "";
          if (!isSentByCurrentNode) {
            relayHtml = `<span class="message-relayid">Relay Id: ${msg.relayID}</span>`;
          }

          li.innerHTML = `
            <span class="message-nodeid">${nodeIdHtml}</span>
            <div class="message-content">${senderHtml}${privateIndicator}${msg.content}</div>
            ${relayHtml}
            <span class="message-time">${timestamp}</span>
            ${rssiSnrHtml}
          `;
          if (isSentByCurrentNode) {
            let indicator = "";
            if (msg.relayIDs && msg.relayIDs.length > 0) {
              indicator += '<span class="circle">🟢</span>';
              for (let i = 0; i < msg.relayIDs.length; i++) {
                indicator += '<span class="satellite">🛰️</span>';
              }
            } else {
              indicator = '<span class="circle">🔴</span><span class="keyboard">⌨️</span>';
            }
            li.innerHTML += `<span class="relay-status">${indicator}</span>`;
          }
          ul.appendChild(li);
        });

        ul.scrollTop = ul.scrollHeight;
      })
      .catch(error => console.error('Error fetching messages:', error));

    fetch('/deviceCount')
      .then(response => response.json())
      .then(data => {
        localStorage.setItem('nodeId', data.nodeId);
         document.getElementById('deviceCount').innerHTML =
          `Mesh Nodes: <a href="/nodes">${data.totalCount}</a>, Node ID: <a href="/txHistory">${data.nodeId}</a>`;
      })
      .catch(error => console.error('Error fetching device count:', error));
  }

  window.onload = function() {
    const savedName = localStorage.getItem('username');
    if (savedName) {
      document.getElementById('nameInput').value = savedName;
    }
    fetchData();
    setInterval(fetchData, 5000);
    document.getElementById('messageForm').addEventListener('submit', sendMessage);
  };
// ——————————————————————————————  
// Poll the battery endpoint every 60 s
// ——————————————————————————————
function updateBattery() {
  fetch('/battery')
    .then(r => r.json())
    .then(data => {
      document.getElementById('batVolt').textContent = data.voltage.toFixed(3);
      document.getElementById('batPct').textContent = data.percentage;
    })
    .catch(console.error);
}
setInterval(updateBattery, 60000);
updateBattery();  // fire on load


</script>
</head>
<body>
  <div id="batteryStatus" style="margin:10px; font-size:0.9em; color:#333;">
    Battery: <span id="batVolt">--</span> V (<span id="batPct">--</span>%)
  </div>
  <div class="warning">For your safety, do not share your location or any personal information!</div>
  
  <h2>Meshmingle Public Chat</h2>
  
  <div id="deviceCount">Wifi Mesh Nodes: 0</div>
  
  <div id="chat-container">
    <ul id="messageList"></ul>
  </div>
  
  <form id="messageForm" style="display: flex; align-items: center; gap: 5px;">
    <input type="text" id="nameInput" name="sender" placeholder="Your name" maxlength="15" required style="flex: 1;">
    <input type="text" id="messageInput" name="msg" placeholder="Your message" maxlength="150" required style="flex: 1;">
    <input type="text" id="targetInput" name="target" placeholder="Node" maxlength="8" style="width: 8ch;">
    <input type="submit" id="sendButton" value="Send" style="flex-shrink: 0;">
  </form>
  
</body>
</html>
)rawliteral";

const char nodesPageHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Meshmingle Nodes</title>
  <style>
    body { 
      font-family: Arial, sans-serif; 
      margin: 0; 
      padding: 20px; 
      text-align: center; 
      background-color: #f4f7f6;
    }
    h2 { 
      color: #333; 
      margin-top: 20px;
    }
    ul { 
      list-style-type: none; 
      padding: 0; 
      margin: 10px auto; 
      max-width: 600px; 
    }
    .node {
      background-color: #fff;
      border: 2px solid;
      border-radius: 10px;
      padding: 15px;
      margin: 10px 0;
      text-align: left;
      font-size: 0.85em;
      color: #333;
      box-shadow: 0 2px 5px rgba(0,0,0,0.1);
      position: relative;
    }
    .node.wifi {
      background-color: #e7f0ff;
      border-color: blue;
    }
    .node.lora {
      background-color: #fff4e0;
      border-color: orange;
    }
    .node-header {
      display: flex;
      align-items: center;
      gap: 10px;
      margin-bottom: 8px;
    }
    .node-header strong {
      font-size: 1em;
    }
    .node-header a {
      text-decoration: none;
      color: #007bff;
      font-weight: bold;
    }
    .node-header a:hover {
      text-decoration: underline;
    }
    .node-info {
      margin-left: 20px;
      font-size: 0.9em;
    }
    .node-emoji {
      position: absolute;
      bottom: 5px;
      right: 5px;
      font-size: 1.2em;
    }
    #nodeCount { 
      margin: 20px auto; 
      max-width: 600px; 
      font-weight: bold;
      text-align: left;
    }
    .node-section {
      margin-bottom: 30px;
    }
    @media (max-width: 600px) {
      .node-info {
        margin-left: 0;
      }
    }
    .nav-links {
      text-align: center;
      margin-bottom: 20px;
    }
    .nav-links a {
      margin: 0 15px;
      text-decoration: none;
      color: #007bff;
      font-weight: bold;
    }
    .nav-links a:hover {
      text-decoration: underline;
    }
  </style>
  <script>
    function fetchNodes() {
      fetch('/nodesData')
        .then(response => response.json())
        .then(data => {
          const wifiUl = document.getElementById('wifiNodeList');
          wifiUl.innerHTML = '';
          const wifiCount = data.wifiNodes.length;
          document.getElementById('wifiCount').innerText = 'WiFi Nodes Connected: ' + wifiCount;
          data.wifiNodes.forEach((node, index) => {
            const li = document.createElement('li');
            li.classList.add('node', 'wifi');
            li.innerHTML = `
              <div class="node-header">
                <strong>Node ${index + 1}:</strong>
                <span>${node}</span>
              </div>
              <div class="node-info"></div>
            `;
            wifiUl.appendChild(li);
          });

          const loraUl = document.getElementById('loraNodeList');
          loraUl.innerHTML = '';
          const loraCount = data.loraNodes.length;
          document.getElementById('loraCount').innerText = 'Direct LoRa Nodes Active: ' + loraCount;
          data.loraNodes.forEach((node, index) => {
            const li = document.createElement('li');
            li.classList.add('node', 'lora');
            li.innerHTML = `
              <div class="node-header">
                <strong>Node ${index + 1}:</strong>
                <a href="/loraDetails?nodeId=${encodeURIComponent(node.nodeId)}">${node.nodeId}</a>
              </div>
              <div class="node-info">
                RSSI: ${node.lastRSSI} dBm, SNR: ${node.lastSNR} dB<br>
                Last seen: ${node.lastSeen}
              </div>
              <div class="node-emoji">${node.statusEmoji || ""}</div>
            `;
            loraUl.appendChild(li);
          });

          const indirectUl = document.getElementById('indirectNodeList');
          indirectUl.innerHTML = "";
          const groupedIndirect = {};
          data.indirectNodes.forEach((node) => {
            let originator = node.originatorId;
            if (!groupedIndirect[originator] || node.lastSeen > groupedIndirect[originator].lastSeen) {
              groupedIndirect[originator] = node;
            }
          });
          const groupedIndirectArray = Object.values(groupedIndirect);
          document.getElementById('indirectCount').innerText = 'Indirect Nodes Last Seen: ' + groupedIndirectArray.length;
          groupedIndirectArray.forEach((node, index) => {
            const li = document.createElement('li');
            li.classList.add('node');
            li.style.backgroundColor = '#f9f9f9';
            li.style.borderColor = '#999';
            li.innerHTML = `
              <div class="node-header">
                <strong>Originator:</strong>
                <span>${node.originatorId}</span>
              </div>
              <div class="node-info">
                Last Relay: <a href="/loraDetails?nodeId=${encodeURIComponent(node.relayId)}">${node.relayId}</a><br>
                RSSI: ${node.rssi} dBm, SNR: ${node.snr} dB<br>
                Last seen: ${node.lastSeen}
              </div>
              <div class="node-emoji">${node.statusEmoji || ""}</div>
            `;
            indirectUl.appendChild(li);
          });
        })
        .catch(error => console.error('Error fetching nodes:', error));
    }

    window.onload = function() {
      fetchNodes();
      setInterval(fetchNodes, 5000);
    };
  </script>
</head>
<body>
  <div class="nav-links">
    <a href="/">Chat</a> | <a href="/metrics">RX History</a> | <a href="/txHistory">TX History</a>
  </div>
  <h2>Meshmingle Nodes</h2>
  
  <div class="node-section">
    <span id="wifiCount">WiFi Nodes Connected: 0</span>
    <ul id="wifiNodeList"></ul>
  </div>

  <div class="node-section">
    <span id="loraCount">Direct LoRa Nodes Active: 0</span>
    <ul id="loraNodeList"></ul>
  </div>
  
  <div class="node-section">
    <span id="indirectCount">Indirect Nodes Last Seen: 0</span>
    <ul id="indirectNodeList"></ul>
  </div>
</body>
</html>
)rawliteral";

const char metricsPageHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>LoRa Signal History</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #f4f7f6;
      margin: 0;
      padding: 20px;
    }
    h2 {
      text-align: center;
      color: #333;
    }
    h3 {
  text-align: center;
}
    .node-id {
      text-align: center;
      font-size: 1.2em;
      color: #555;
      margin-top: 10px;
    }
    .node-block {
      background-color: #fff;
      border: 2px solid #ccc;
      border-radius: 8px;
      margin: 20px auto;
      max-width: 600px;
      padding: 10px;
      box-sizing: border-box;
    }
    .sub-node-block {
      background-color: #f9f9f9;
      border: 1px solid #ccc;
      border-radius: 6px;
      margin: 10px 0;
      padding: 8px;
    }
    .node-title {
      font-weight: bold;
      margin-bottom: 8px;
    }
    table {
      border-collapse: collapse;
      width: 100%;
      margin-bottom: 10px;
    }
    th, td {
      border: 1px solid #ccc;
      padding: 5px;
      text-align: left;
      font-size: 0.9em;
    }
    th {
      background-color: #eee;
    }
    .nav-links {
      display: flex;
      justify-content: center;
      align-items: center;
      gap: 10px;
      margin-bottom: 20px;
    }
    .nav-links a {
      text-decoration: none;
      color: #007bff;
      font-weight: bold;
    }
    .nav-links a:hover {
      text-decoration: underline;
    }
    .history-container {
      display: none;
      margin-top: 10px;
    }
    .toggle-btn {
      background-color: #007bff;
      color: #fff;
      border: none;
      padding: 5px 10px;
      cursor: pointer;
      border-radius: 4px;
      margin: 10px auto;
      display: block;
    }
    .toggle-btn:hover {
      background-color: #0056b3;
    }
  </style>
  <script>
    window.onload = function() {
      fetch("/deviceCount")
        .then(response => response.json())
        .then(data => {
          const nodeIdElement = document.getElementById("nodeIdDisplay");
          nodeIdElement.textContent = `Node ID: ${data.nodeId}`;
        })
        .catch(error => console.error("Error fetching Node ID:", error));

      fetchHistory();
      setInterval(fetchHistory, 5000);
    };

    function fetchHistory() {
      if (document.querySelector('.history-container[style*="display: block"]')) {
        return;
      }
      fetch("/metricsHistoryData")
        .then(response => response.json())
        .then(data => {
          const container = document.getElementById("historyContainer");
          container.innerHTML = "";

          if (data.loraNodes && data.loraNodes.length > 0) {
            const directHeader = document.createElement("h3");
            directHeader.textContent = "Direct LoRa Nodes History";
            container.appendChild(directHeader);

            data.loraNodes.forEach(node => {
              const nodeBlock = document.createElement("div");
              nodeBlock.classList.add("node-block");

              const nodeTitle = document.createElement("div");
              nodeTitle.classList.add("node-title");
              nodeTitle.textContent = `Node ID: ${node.nodeId}`;
              nodeBlock.appendChild(nodeTitle);

              const summaryTable = document.createElement("table");
              const summaryHeader = document.createElement("thead");
              summaryHeader.innerHTML = `
                <tr>
                  <th>Best Signal RSSI (dBm)</th>
                  <th>Best Signal SNR (dB)</th>
                </tr>`;
              summaryTable.appendChild(summaryHeader);
              const summaryBody = document.createElement("tbody");
              const summaryRow = document.createElement("tr");
              summaryRow.innerHTML = `<td>${node.bestRssi}</td><td>${node.bestSnr}</td>`;
              summaryBody.appendChild(summaryRow);
              summaryTable.appendChild(summaryBody);
              nodeBlock.appendChild(summaryTable);

              const toggleBtn = document.createElement("button");
              toggleBtn.textContent = "Show Details";
              toggleBtn.classList.add("toggle-btn");
              toggleBtn.onclick = function() {
                toggleDetails(this);
              };
              nodeBlock.appendChild(toggleBtn);

              const historyDiv = document.createElement("div");
              historyDiv.classList.add("history-container");
              const historyTable = document.createElement("table");
              const historyHeader = document.createElement("thead");
              historyHeader.innerHTML = `
                <tr>
                  <th>Time Ago</th>
                  <th>RSSI (dBm)</th>
                  <th>SNR (dB)</th>
                </tr>`;
              historyTable.appendChild(historyHeader);
              const historyBody = document.createElement("tbody");
              node.history.forEach(sample => {
                const row = document.createElement("tr");
                row.innerHTML = `<td>${sample.timestamp}</td><td>${sample.rssi}</td><td>${sample.snr}</td>`;
                historyBody.appendChild(row);
              });
              historyTable.appendChild(historyBody);
              historyDiv.appendChild(historyTable);
              nodeBlock.appendChild(historyDiv);

              container.appendChild(nodeBlock);
            });
          }

          if (data.indirectNodes && data.indirectNodes.length > 0) {
            const indirectHeader = document.createElement("h3");
            indirectHeader.textContent = "Indirect Nodes History (Grouped by Originator)";
            container.appendChild(indirectHeader);

            const groupedIndirect = {};
            data.indirectNodes.forEach(indirect => {
              let originator = indirect.nodeId;
              if (!groupedIndirect[originator]) {
                groupedIndirect[originator] = [];
              }
              groupedIndirect[originator].push(indirect);
            });

            Object.keys(groupedIndirect).forEach(originator => {
              const group = groupedIndirect[originator];
              const originatorBlock = document.createElement("div");
              originatorBlock.classList.add("node-block");

              const originatorTitle = document.createElement("div");
              originatorTitle.classList.add("node-title");
              originatorTitle.textContent = `Originator: ${originator}`;
              originatorBlock.appendChild(originatorTitle);

              group.forEach(relay => {
                const subNodeBlock = document.createElement("div");
                subNodeBlock.classList.add("sub-node-block");

                const relayTitle = document.createElement("div");
                relayTitle.classList.add("node-title");
                relayTitle.textContent = `Relay ID: ${relay.relayId}`;
                subNodeBlock.appendChild(relayTitle);

                const relaySummaryTable = document.createElement("table");
                const relaySummaryHeader = document.createElement("thead");
                relaySummaryHeader.innerHTML = `
                  <tr>
                    <th>Best Signal RSSI (dBm)</th>
                    <th>Best Signal SNR (dB)</th>
                  </tr>`;
                relaySummaryTable.appendChild(relaySummaryHeader);
                const relaySummaryBody = document.createElement("tbody");
                const relaySummaryRow = document.createElement("tr");
                relaySummaryRow.innerHTML = `<td>${relay.bestRssi}</td><td>${relay.bestSnr}</td>`;
                relaySummaryBody.appendChild(relaySummaryRow);
                relaySummaryTable.appendChild(relaySummaryBody);
                subNodeBlock.appendChild(relaySummaryTable);

                const toggleBtnRelay = document.createElement("button");
                toggleBtnRelay.textContent = "Show Details";
                toggleBtnRelay.classList.add("toggle-btn");
                toggleBtnRelay.onclick = function() {
                  toggleDetails(this);
                };
                subNodeBlock.appendChild(toggleBtnRelay);

                const relayHistoryDiv = document.createElement("div");
                relayHistoryDiv.classList.add("history-container");
                const relayHistoryTable = document.createElement("table");
                const relayHistoryHeader = document.createElement("thead");
                relayHistoryHeader.innerHTML = `
                  <tr>
                    <th>Time Ago</th>
                    <th>RSSI (dBm)</th>
                    <th>SNR (dB)</th>
                  </tr>`;
                relayHistoryTable.appendChild(relayHistoryHeader);
                const relayHistoryBody = document.createElement("tbody");
                relay.history.forEach(sample => {
                  const row = document.createElement("tr");
                  row.innerHTML = `<td>${sample.timestamp}</td><td>${sample.rssi}</td><td>${sample.snr}</td>`;
                  relayHistoryBody.appendChild(row);
                });
                relayHistoryTable.appendChild(relayHistoryBody);
                relayHistoryDiv.appendChild(relayHistoryTable);
                subNodeBlock.appendChild(relayHistoryDiv);

                originatorBlock.appendChild(subNodeBlock);
              });

              container.appendChild(originatorBlock);
            });
          }
        })
        .catch(error => {
          console.error("Error fetching metrics history:", error);
        });
    }

    function toggleDetails(button) {
      var container = button.nextElementSibling;
      if (container.style.display === "none" || container.style.display === "") {
        container.style.display = "block";
        button.textContent = "Hide Details";
      } else {
        container.style.display = "none";
        button.textContent = "Show Details";
      }
    }
  </script>
</head>
<body>
  <div class="nav-links">
    <a href="/">Chat</a> | <a href="/nodes">NodeList</a> | <a href="/txHistory">TX History</a>
  </div>
  <h2>LoRa 24hr RX History</h2>
  <div class="node-id" id="nodeIdDisplay">Node ID: Loading...</div>
  <div id="historyContainer"></div>
</body>
</html>
)rawliteral";

const char txHistoryPageHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>TX History</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #f4f7f6;
      margin: 0;
      padding: 20px;
    }
    h2 {
      text-align: center;
      color: #333;
    }
    table {
      border-collapse: collapse;
      width: 100%;
      margin: 20px auto;
    }
    th, td {
      border: 1px solid #ccc;
      padding: 8px;
      text-align: left;
      font-size: 0.9em;
    }
    th {
      background-color: #eee;
    }
    .nav-links {
      text-align: center;
      margin-bottom: 20px;
    }
    .nav-links a {
      margin: 0 15px;
      text-decoration: none;
      color: #007bff;
      font-weight: bold;
    }
    .nav-links a:hover {
      text-decoration: underline;
    }
  </style>
  <script>
    function fetchTxHistory() {
      fetch('/txHistoryData')
        .then(response => response.json())
        .then(data => {
          const container = document.getElementById('historyContainer');
          container.innerHTML = '';
          if (data.txHistory.length === 0) {
            container.innerHTML = '<p>No TX history available.</p>';
            return;
          }
          let html = '<table><thead><tr><th>Timestamp</th><th>Type</th><th>TX Time (ms)</th><th>Status</th></tr></thead><tbody>';
          data.txHistory.forEach(entry => {
            html += '<tr>';
            html += '<td>' + entry.timestamp + '</td>';
            html += '<td>' + entry.type + '</td>';
            html += '<td>' + entry.txTime + '</td>';
            html += '<td>' + (entry.success ? '✅' : '❌') + '</td>';
            html += '</tr>';
          });
          html += '</tbody></table>';
          container.innerHTML = html;
        })
        .catch(error => {
          console.error('Error fetching TX history:', error);
        });
    }
    window.onload = function() {
      fetchTxHistory();
      setInterval(fetchTxHistory, 5000);
    };
  </script>
</head>
<body>
  <div class="nav-links">
    <a href="/">Chat</a> | <a href="/nodes">Nodes</a> | <a href="/metrics">RX History</a>
  </div>
  <h2>Lora 24hr TX History</h2>
  <div id="historyContainer"></div>
</body>
</html>
)rawliteral";

// The PROGMEM HTML page at /clients
const char clientsPageHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Meshmingle Clients</title>
  <style>
    body { font-family: Arial,sans-serif; background:#f4f7f6; margin:0; padding:20px; text-align:center; }
    h2 { color:#333; margin-bottom:10px; }
    #macList { list-style:none; padding:0; max-width:400px; margin:auto; }
    .mac-item { background:#fff; border:1px solid #ccc; border-radius:5px; padding:8px; margin:5px 0; font-family:monospace; }
    .nav-links { margin-bottom:20px; }
    .nav-links a { margin:0 10px; color:#007bff; text-decoration:none; font-weight:bold; }
    .nav-links a:hover { text-decoration:underline; }
  </style>
  <script>
function fetchClients() {
  fetch('/clientsData')
    .then(r => r.json())
    .then(data => {
      document.getElementById('ourMac').textContent = data.ourMac;  // new
      const ul = document.getElementById('macList');
      ul.innerHTML = '';
      data.macs.forEach(mac => {
        const li = document.createElement('li');
        li.className = 'mac-item';
        li.textContent = mac;
        ul.appendChild(li);
      });
    })
    .catch(console.error);
}
    window.onload = function() {
      fetchClients();
      setInterval(fetchClients, 10000);
    };
  </script>
</head>
<body>
  <div class="nav-links">
    <a href="/">Chat</a>
    <a href="/nodes">Nodes</a>
    <a href="/metrics">RX History</a>
  </div>
  <p><strong>Node Mac:</strong> <code id="ourMac"></code></p>   <!-- new line -->
  <h2>All Clients (Past &amp; Present)</h2>
  <ul id="macList"></ul>
  </body>
  </html>
  )rawliteral";

// --- Helper function to format relative time ---
String formatRelativeTime(uint64_t ageMs) {
  uint64_t ageSec = ageMs / 1000;
  if (ageSec < 60) {
    return String(ageSec) + " sec ago";
  } else if (ageSec < 3600) {
    uint64_t minutes = ageSec / 60;
    uint64_t seconds = ageSec % 60;
    String result = String(minutes) + " min";
    if (seconds > 0) {
      result += " " + String(seconds) + " sec";
    }
    result += " ago";
    return result;
  } else {
    uint64_t hours = ageSec / 3600;
    uint64_t minutes = (ageSec % 3600) / 60;
    uint64_t seconds = ageSec % 60;
    String result = String(hours) + " hr";
    if (minutes > 0) {
      result += " " + String(minutes) + " min";
    }
    if (seconds > 0) {
      result += " " + String(seconds) + " sec";
    }
    result += " ago";
    return result;
  }
}

// --- Helper to filter messages for a given node ---
// UPDATED: Now checks if the nodeId is in the relayIDs vector as well.
std::vector<Message> getNodeMessages(const String& nodeId) {
  std::vector<Message> result;
  for (auto &m : messages) {
    if (m.recipient != "ALL") continue;
    bool inRelayList = false;
    for (auto &rid : m.relayIDs) {
      if (rid == nodeId) { 
        inRelayList = true; 
        break; 
      }
    }
    if (m.nodeId == nodeId || m.relayID == nodeId || inRelayList) {
      result.push_back(m);
    }
  }
  return result;
}

void setupServer() {
  setupServerRoutes();
}

void setupServerRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", mainPageHtml);
  });

  server.on("/nodes", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", nodesPageHtml);
  });

server.on("/messages", HTTP_GET, [](AsyncWebServerRequest* request) {
  String json = "{\"messages\":[";
  bool first = true;
  for (const auto& msg : messages) {
    if (!first) json += ",";
    String fullMsg = msg.messageID + "|" + msg.nodeId + "|" + msg.sender + "|" + msg.recipient + "|" + msg.content + "|" + msg.relayID;
    String emoji = determineTxType(fullMsg);
    json += "{\"nodeId\":\"" + msg.nodeId + "\",";
    json += "\"sender\":\"" + msg.sender + "\",";
    json += "\"recipient\":\"" + msg.recipient + "\",";
    json += "\"content\":\"" + msg.content + "\",";
    json += "\"source\":\"" + msg.source + "\",";
    json += "\"messageID\":\"" + msg.messageID + "\",";
    json += "\"relayID\":\"" + msg.relayID + "\",";
    json += "\"timeReceived\":" + String(msg.timeReceived) + ",";
    json += "\"emoji\":\"" + emoji + "\"";
    if (msg.source == "[LoRa]") {
      json += ",\"rssi\":" + String(msg.rssi) + ",\"snr\":" + String(msg.snr, 2);
    }
    json += ",\"relayIDs\":[";
    bool firstRelay = true;
    for (auto &rid : msg.relayIDs) {
      if (!firstRelay) json += ",";
      json += "\"" + rid + "\"";
      firstRelay = false;
    }
    json += "]}";
    first = false;
  }
  json += "], \"currentDeviceTime\":" + String(millis()) + "}";
  request->send(200, "application/json", json);
});

  server.on("/deviceCount", HTTP_GET, [](AsyncWebServerRequest* request) {
    updateMeshData();
    request->send(200, "application/json", "{\"totalCount\":" + String(totalNodeCount) + ", \"nodeId\":\"" + getCustomNodeId(getNodeId()) + "\"}");
  });

  server.on("/nodesData", HTTP_GET, [](AsyncWebServerRequest* request) {
    updateMeshData();
    String json = "{\"wifiNodes\":[";
    auto wifiNodeList = mesh.getNodeList();
    bool firstWifi = true;
    for (auto node : wifiNodeList) {
      if (!firstWifi) json += ",";
      json += "\"" + getCustomNodeId(node) + "\"";
      firstWifi = false;
    }
    json += "], \"loraNodes\":[";
    bool firstLora = true;
    uint64_t currentTime = millis();
    const uint64_t FIFTEEN_MINUTES = 960000;
    for (auto const& [nodeId, loraNode] : loraNodes) {
      uint64_t lastSeenTime = loraNode.lastSeen;
      if (currentTime - lastSeenTime <= FIFTEEN_MINUTES) {
        if (!firstLora) json += ",";
        json += "{\"nodeId\":\"" + nodeId + "\"," 
              + "\"lastRSSI\":" + String(loraNode.lastRSSI) + "," 
              + "\"lastSNR\":" + String(loraNode.lastSNR, 2) + "," 
              + "\"lastSeen\":\"" + formatRelativeTime(currentTime - lastSeenTime) + "\"," 
              + "\"statusEmoji\":\"" + loraNode.statusEmoji + "\"}";
        firstLora = false;
      }
    }
    json += "], \"indirectNodes\":[";
    bool firstIndirect = true;
    const uint64_t THIRTY_ONE_MINUTES = 1860000;
    for (auto const& entry : indirectNodes) {
      if (!firstIndirect) json += ",";
      const auto &node = entry.second;
      json += "{\"originatorId\":\"" + node.originatorId + "\",\"relayId\":\"" + node.relayId + "\","; 
      json += "\"rssi\":" + String(node.rssi) + ",\"snr\":" + String(node.snr, 2) + ",\"lastSeen\":\"" + formatRelativeTime(currentTime - node.lastSeen) + "\","; 
      json += "\"statusEmoji\":\"" + node.statusEmoji + "\"}";
      firstIndirect = false;
    }
    json += "]}";
    request->send(200, "application/json", json);
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest* request) {
    String newMessage = "";
    String senderName = "";
    String target = "";
    if (request->hasParam("msg", true)) {
      newMessage = request->getParam("msg", true)->value();
    }
    if (request->hasParam("sender", true)) {
      senderName = request->getParam("sender", true)->value();
    }
    if (request->hasParam("target", true)) {
      target = request->getParam("target", true)->value();
    }
    newMessage.replace("<", "&lt;");
    newMessage.replace(">", "&gt;");
    senderName.replace("<", "&lt;");
    senderName.replace(">", "&gt;");
    target.replace("<", "&lt;");
    target.replace(">", "&gt;");

    if(target.length() == 0) {
         target = "ALL";
    }

    String messageID = generateMessageID(getCustomNodeId(getNodeId()));
    String relayID = getCustomNodeId(getNodeId());
    String constructedMessage = constructMessage(messageID, getCustomNodeId(getNodeId()), senderName, target, newMessage, relayID);

    addMessage(getCustomNodeId(getNodeId()), messageID, senderName, target, newMessage, "[LoRa]", relayID);
    Serial.printf("[LoRa Tx] Adding message: %s\n", constructedMessage.c_str());

  if (cfg_lora_enabled) {
    // still queue it for LoRa → then Wi-Fi
    messageTransmissions[messageID].origin          = ORIGIN_LORA;
    messageTransmissions[messageID].pendingWiFiRelay = true;
    scheduleLoRaTransmission(constructedMessage, -100);
  }else{
    // LoRa is off, so just send it over Wi-Fi right now:
    Serial.println("[WiFi Tx] LoRa disabled, broadcasting immediately");
    transmitViaWiFi(constructedMessage);
}
    request->redirect("/");
  });

  server.on("/metrics", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", metricsPageHtml);
  });

  server.on("/metricsHistoryData", HTTP_GET, [](AsyncWebServerRequest* request) {
    uint64_t now = millis();
    const uint64_t ONE_DAY = 86400000;
    String json = "{";
    json += "\"loraNodes\":[";
    bool firstNode = true;
    for (auto const& kv : loraNodes) {
      if (!firstNode) json += ",";
      firstNode = false;
      const auto &node = kv.second;
      int bestRssi = node.history.empty() ? node.lastRSSI : node.history[0].rssi;
      float bestSnr = node.history.empty() ? node.lastSNR : node.history[0].snr;
      for (const auto &sample : node.history) {
        if (sample.rssi > bestRssi) {
          bestRssi = sample.rssi;
        }
        if (sample.snr > bestSnr) {
          bestSnr = sample.snr;
        }
      }
      json += "{\"nodeId\":\"" + node.nodeId + "\",\"bestRssi\":" + String(bestRssi)
           + ",\"bestSnr\":" + String(bestSnr, 2) + ",\"history\":[";
      bool firstSample = true;
      for (const auto &sample : node.history) {
        uint64_t ageMs = now - sample.timestamp;
        if (ageMs <= ONE_DAY && sample.rssi != 0) {
          if (!firstSample) json += ",";
          firstSample = false;
          String relativeTime = formatRelativeTime(ageMs);
          json += "{\"timestamp\":\"" + relativeTime
               + "\",\"rssi\":" + String(sample.rssi)
               + ",\"snr\":" + String(sample.snr, 2) + "}";
        }
      }
      json += "]}";
    }
    json += "],";
    json += "\"indirectNodes\":[";
    bool firstIndirect = true;
    for (auto const& kv : indirectNodes) {
      const auto &node = kv.second;
      if (!firstIndirect) json += ",";
      firstIndirect = false;
      int bestRssi = node.history.empty() ? node.rssi : node.history[0].rssi;
      float bestSnr = node.history.empty() ? node.snr : node.history[0].snr;
      for (const auto &sample : node.history) {
        if (sample.rssi > bestRssi) {
          bestRssi = sample.rssi;
        }
        if (sample.snr > bestSnr) {
          bestSnr = sample.snr;
        }
      }
      json += "{\"nodeId\":\"" + node.originatorId + "\",\"relayId\":\"" + node.relayId + "\","; 
      json += "\"bestRssi\":" + String(bestRssi) + ",\"bestSnr\":" + String(bestSnr, 2) + ",\"history\":[";
      
      bool firstSample = true;
      for (const auto &sample : node.history) {
        uint64_t ageMs = now - sample.timestamp;
        if (ageMs <= ONE_DAY && sample.rssi != 0) {
          if (!firstSample) json += ",";
          firstSample = false;
          String relativeTime = formatRelativeTime(ageMs);
          json += "{\"timestamp\":\"" + relativeTime
               + "\",\"rssi\":" + String(sample.rssi)
               + ",\"snr\":" + String(sample.snr, 2) + "}";
        }
      }
      json += "],\"statusEmoji\":\"" + node.statusEmoji + "\"}";
    }
    json += "]}";
    request->send(200, "application/json", json);
  });

  server.on("/txHistory", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", txHistoryPageHtml);
  });

server.on("/txHistoryData", HTTP_GET, [](AsyncWebServerRequest* request) {
  String json = "{\"txHistory\":[";
  bool first = true;
  uint64_t now = millis();
  for (int i = txHistory.size()-1; i >= 0; i--) {
    if (!first) json += ",";
    String formattedTime = formatRelativeTime(now - txHistory[i].timestamp);
    json += "{\"timestamp\":\"" + formattedTime + "\",";
    json += "\"type\":\"" + txHistory[i].type + "\",";
    json += "\"txTime\":" + String(txHistory[i].txTime) + ",";
    json += "\"success\":" + String(txHistory[i].success ? "true" : "false") + "}";
    first = false;
  }
  json += "]}";
  request->send(200, "application/json", json);
});

  server.on("/loraDetails", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("nodeId")) {
      String html =
        "<!DOCTYPE html><html lang='en'><head>"
        "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
        "<title>LoRa Nodes List</title>"
        "<style>"
        "body { font-family: Arial, sans-serif; background-color: #f4f7f6; margin:0; padding:20px; }"
        ".nav-links { text-align:center; margin-bottom:20px; background-color:#eee; padding:10px; }"
        ".nav-links a { margin:0 15px; text-decoration:none; color:#007bff; font-weight:bold; }"
        ".nav-links a:hover { text-decoration:underline; }"
        "h2 { text-align:center; color:#333; }"
        ".node-list { max-width:600px; margin:10px auto; background:#fff; padding:10px; border-radius:8px; }"
        ".node-link { display:block; padding:8px; border-bottom:1px solid #ccc; color:#007bff; text-decoration:none; }"
        ".node-link:hover { background-color:#f0f0f0; }"
        "</style>"
        "</head><body>"
        "<div class='nav-links'>"
        "<a href='/'>Chat</a>"
        "<a href='/nodes'>All Nodes</a>"
        "<a href='/metrics'>RX History</a>"
        "</div>"
        "<h2>LoRa Node Details</h2>"
        "<p style='text-align:center;'>Select a LoRa node to see metrics and messages.</p>"
        "<div class='node-list'>";
      uint64_t currentTime = millis();
      const uint64_t FIFTEEN_MINUTES = 900000;
      bool anyLoRa = false;
      for (auto &kv : loraNodes) {
        if (currentTime - kv.second.lastSeen <= FIFTEEN_MINUTES) {
          anyLoRa = true;
          html += "<a class='node-link' href='/loraDetails?nodeId=" + kv.first + "'>"
                  + kv.first + "</a>";
        }
      }
      if (!anyLoRa) {
        html += "<p>No LoRa nodes seen in last 15 minutes.</p>";
      }
      html += "</div></body></html>";
      request->send(200, "text/html", html);
    } else {
      String nodeId = request->getParam("nodeId")->value();
      LoRaNode *found = nullptr;
      if (loraNodes.find(nodeId) != loraNodes.end()) {
        found = &loraNodes[nodeId];
      }
      String html =
        "<!DOCTYPE html><html lang='en'><head>"
        "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
        "<title>LoRa Node " + nodeId + "</title>"
        "<style>"
        "body { font-family: Arial, sans-serif; background-color: #f4f7f6; margin:0; padding:20px; }"
        ".nav-links { text-align:center; margin-bottom:20px; background-color:#eee; padding:10px; }"
        ".nav-links a { margin:0 15px; text-decoration:none; color:#007bff; font-weight:bold; }"
        ".nav-links a:hover { text-decoration:underline; }"
        "h2 { text-align:center; color:#333; }"
        ".details { max-width:600px; margin:10px auto; background:#fff; padding:10px; border-radius:8px; }"
        "table { border-collapse: collapse; width: 100%; margin-bottom: 10px; }"
        "th, td { border: 1px solid #ccc; padding: 5px; text-align:left; font-size:0.9em; }"
        "th { background-color:#eee; }"
        ".section-title { font-weight:bold; font-size:1.1em; margin-top:20px; }"
        ".message-block { border:1px solid #ccc; margin:5px 0; padding:8px; border-radius:4px; }"
        ".message-block h4 { margin:0; font-size:0.9em; }"
        ".message-block p { margin:4px 0; font-size:0.85em; white-space: pre-wrap; word-break: break-all; overflow-wrap: break-word; }"
        ".details p { white-space: pre-wrap; word-break: break-all; overflow-wrap: break-word; }"
        "</style></head><body>";
      html +=
        "<div class='nav-links'>"
        "<a href='/'>Chat</a>"
        "<a href='/nodes'>Node List</a>"
        "<a href='/metrics'>RX History</a>"
        "</div>";
      html += "<h2>LoRa Node: " + nodeId + "</h2>";
      if (!found) {
        html += "<div class='details'><p>No details available. This node hasn't been heard from or it's older than 24h.</p></div>";
        html += "</body></html>";
        request->send(200, "text/html", html);
        return;
      }
      uint64_t ageMs = millis() - found->lastSeen;
      String ageStr = formatRelativeTime(ageMs);
      html += "<div class='details'>";
      html += "<p><strong>Last RSSI:</strong> " + String(found->lastRSSI) + " dBm<br>";
      html += "<strong>Last SNR:</strong> " + String(found->lastSNR) + " dB<br>";
      html += "<strong>Last Seen:</strong> " + ageStr + "</p>";
      const uint64_t ONE_DAY = 86400000;
      bool anySample = false;
      html += "<div class='section-title'>RSSI/SNR History (24hr)</div>";
      html += "<table><thead><tr><th>Time Ago</th><th>RSSI (dBm)</th><th>SNR (dB)</th></tr></thead><tbody>";
      for (auto &sample : found->history) {
        uint64_t sampleAge = millis() - sample.timestamp;
        if (sampleAge <= ONE_DAY) {
          anySample = true;
          html += "<tr>";
          html += "<td>" + formatRelativeTime(sampleAge) + "</td>";
          html += "<td>" + String(sample.rssi) + "</td>";
          html += "<td>" + String(sample.snr, 2) + "</td>";
          html += "</tr>";
        }
      }
      if (!anySample) {
        html += "<tr><td colspan='3'>No samples in last 24 hours.</td></tr>";
      }
      html += "</tbody></table>";
      auto nodeMsgs = getNodeMessages(nodeId);
      html += "<div class='section-title'>Messages Sent/Relayed</div>";
      if (nodeMsgs.empty()) {
        html += "<p>No messages from or through this node.</p>";
      } else {
        for (auto it = nodeMsgs.rbegin(); it != nodeMsgs.rend(); ++it) {
          uint64_t msgAgeMs = millis() - it->timeReceived;
          String msgAgeStr = formatRelativeTime(msgAgeMs);
          html += "<div class='message-block'>";
          html += "<h4>Sender: " + it->sender + " | NodeID: " + it->nodeId + "</h4>";
          html += "<p><strong>Content:</strong> " + it->content + "</p>";
          html += "<p><strong>Source:</strong> " + it->source + "</p>";
          if (!it->relayID.isEmpty()) {
            html += "<p><strong>RelayID:</strong> " + it->relayID + "</p>";
          }
          html += "<p><em>" + msgAgeStr + "</em></p>";
          html += "</div>";
        }
      }
      html += "</div></body></html>";
      request->send(200, "text/html", html);
    }
  });

    // --- Clients page: serves HTML ---
  server.on("/clients", HTTP_GET, [](AsyncWebServerRequest* request){
    updateClientList();  // refresh our master list
    request->send_P(200, "text/html", clientsPageHtml);
  });

  // --- JSON endpoint for JS to pull MAC list ---
  server.on("/clientsData", HTTP_GET, [](AsyncWebServerRequest* request){
  updateClientList();
  String json = "{\"ourMac\":\"" + ourApMac + "\",\"macs\":[";
  bool first = true;
  for (const auto& m : clientMacs) {
    if (!first) json += ",";
    json += "\"" + m + "\"";
    first = false;
  }
  json += "]}";
  request->send(200, "application/json", json);
  });

  // —————————————————————————————————————————————————————————————————
//  Battery status endpoint
// —————————————————————————————————————————————————————————————————
server.on("/battery", HTTP_GET, [](AsyncWebServerRequest* request) {
  // respond with the last‐cached values
  String json = String("{\"voltage\":") + String(cachedBatteryVoltage, 3)
              + ",\"percentage\":" + String(cachedBatteryPercentage) + "}";
  request->send(200, "application/json", json);
});
}

void updateMeshData() {
  mesh.update();
  totalNodeCount = mesh.getNodeList().size();
  currentNodeId = mesh.getNodeId();
}

int getNodeCount() {
  std::set<String> uniqueNodes;
  uint64_t now = millis();

  auto wifiNodes = mesh.getNodeList();
  for (uint32_t id : wifiNodes) {
    uniqueNodes.insert(getCustomNodeId(id));
  }

  const uint64_t DIRECT_THRESHOLD = 16UL * 60UL * 1000UL;
  for (auto const &entry : loraNodes) {
    if (now - entry.second.lastSeen <= DIRECT_THRESHOLD) {
      uniqueNodes.insert(entry.first);
    }
  }

  const uint64_t INDIRECT_THRESHOLD = 1860000;
  for (auto const &entry : indirectNodes) {
    uniqueNodes.insert(entry.second.originatorId);
  }

  return uniqueNodes.size();
}

void initServer() {
  setupServerRoutes();
}
