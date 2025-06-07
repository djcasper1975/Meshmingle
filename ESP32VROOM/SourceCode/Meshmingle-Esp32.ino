//Test v1.00.039
//06-06-2025
//MAKE SURE ALL NODES USE THE SAME VERSION OR EXPECT STRANGE THINGS HAPPENING.
//Changed network name.
//added private messages to other nodes.
//added wifi channel config
//added wifi setup page mesh.mingle/settings
////////////////////////////////////////////////////////////////////////
// M    M  EEEEE  SSSSS  H   H  M    M  I  N   N  GGGGG  L      EEEEE //
// MM  MM  E      S      H   H  MM  MM  I  NN  N  G      L      E     //
// M MM M  EEEE   SSSSS  HHHHH  M MM M  I  N N N  G  GG  L      EEEE  //
// M    M  E          S  H   H  M    M  I  N  NN  G   G  L      E     //
// M    M  EEEEE  SSSSS  H   H  M    M  I  N   N   GGG   LLLLL  EEEEE //
////////////////////////////////////////////////////////////////////////

#include <painlessMesh.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <esp_wifi.h>             // for esp_wifi_ap_get_sta_list()
#include <vector>
#include <map>
#include <set>
#include "settings_feature_wifi.h"

// Mesh network parameters:
#define MESH_PORT       5555
#define MESH_CHANNEL   3

// Global mesh/network objects:
painlessMesh mesh;
AsyncWebServer server(80);
DNSServer dnsServer;

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
    clientMacs.insert(macToString(staList.sta[i].mac));
  }
}

// Global variables for node identity and message tracking:
int totalNodeCount = 0;
uint32_t currentNodeId = 0;
uint32_t messageCounter = 0;

// -----------------------------
// Message Transmission Tracking
// -----------------------------
enum OriginChannel { ORIGIN_UNKNOWN, ORIGIN_WIFI, ORIGIN_LORA };
struct TransmissionStatus {
  bool transmittedViaWiFi = false;
  bool addedToMessages = false;
  OriginChannel origin = ORIGIN_UNKNOWN;
  uint64_t timestamp = 0;
};

std::map<String, TransmissionStatus> messageTransmissions;

// -----------------------------
// Message Structures
// -----------------------------
struct Message {
  String nodeId;     // originator node ID
  String sender;
  String recipient;  // "ALL" for public or a specific node ID
  String content;
  String source;     // e.g., "[WiFi]"
  String messageID;
  String relayID;
  uint64_t timeReceived;
};

std::vector<Message> messageList;  
const int maxMessages = 50;

// -----------------------------
// Utility functions
// -----------------------------
// Returns a custom formatted node id, e.g., "!Mxxxxxx"
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

// Simple CRC16-CCITT implementation
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

// Generates a unique message ID based on our node id and an incrementing counter.
String generateMessageID(const String& nodeId) {
  messageCounter++;
  return nodeId + ":" + String(messageCounter);
}

// Constructs a message string in the format:
// messageID|originatorID|sender|recipient|content|relayID|CRC
String constructMessage(const String& messageID, const String& originatorID, const String& sender,
                        const String& recipient, const String& content, const String& relayID) {
  String messageWithoutCRC = messageID + "|" + originatorID + "|" + sender + "|" + recipient + "|" + content + "|" + relayID;
  uint16_t crc = crc16_ccitt((const uint8_t *)messageWithoutCRC.c_str(), messageWithoutCRC.length());
  char crcStr[5];
  sprintf(crcStr, "%04X", crc);
  String fullMessage = messageWithoutCRC + "|" + String(crcStr);
  return fullMessage;
}

// -----------------------------
// Message Handling
// -----------------------------
void addMessage(const String& nodeId, const String& messageID, const String& sender, 
                const String& recipient, String content, const String& source, 
                const String& relayID) {
  const int maxMessageLength = 150;
  if (content.length() > maxMessageLength) {
    content = content.substring(0, maxMessageLength);
  }
  
  auto& status = messageTransmissions[messageID];
  if (status.addedToMessages) {
    return;
  }
  
  // For private messages, only add if the message is for this node or originated here.
  String myId = getCustomNodeId(currentNodeId);
  if (recipient != "ALL" && myId != nodeId && myId != recipient) {
    return;
  }
  
  Message newMessage = { nodeId, sender, recipient, content, source, messageID, relayID, millis() };
  messageList.insert(messageList.begin(), newMessage);
  status.addedToMessages = true;
  if (messageList.size() > maxMessages) {
    messageList.pop_back();
  }
  
  Serial.printf("Message added: NodeID: %s, Sender: %s, Recipient: %s, Content: %s, Source: %s, ID: %s, RelayID: %s\n",
                nodeId.c_str(), sender.c_str(), recipient.c_str(), content.c_str(), source.c_str(), messageID.c_str(), relayID.c_str());
}

void transmitViaWiFi(const String& message) {
  Serial.printf("[WiFi Tx] Transmitting: %s\n", message.c_str());
  int separatorIndex = message.indexOf('|');
  if (separatorIndex == -1) return;
  String messageID = message.substring(0, separatorIndex);
  
  auto& status = messageTransmissions[messageID];
  if (status.transmittedViaWiFi) {
    Serial.println("[WiFi Tx] Already transmitted, skipping.");
    return;
  }
  
  mesh.sendBroadcast(message);
  status.transmittedViaWiFi = true;
}

// -----------------------------
// Mesh Callback & Update Functions
// -----------------------------
void receivedCallback(uint32_t from, String & message) {
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
  
  // (Optional) Skip heartbeat messages:
  if (messageWithoutCRC.startsWith("HEARTBEAT|") || messageWithoutCRC.startsWith("AGG_HEARTBEAT|")) {
    Serial.println("[WiFi Rx] Heartbeat received, not adding to message list.");
    return;
  }
  
  // Parse the message fields:
  int firstSeparator = messageWithoutCRC.indexOf('|');
  int secondSeparator = messageWithoutCRC.indexOf('|', firstSeparator + 1);
  int thirdSeparator = messageWithoutCRC.indexOf('|', secondSeparator + 1);
  int fourthSeparator = messageWithoutCRC.indexOf('|', thirdSeparator + 1);
  int fifthSeparator = messageWithoutCRC.indexOf('|', fourthSeparator + 1);
  
  if (firstSeparator == -1 || secondSeparator == -1 || thirdSeparator == -1 ||
      fourthSeparator == -1 || fifthSeparator == -1) {
    Serial.println("[WiFi Rx] Invalid message format.");
    return;
  }
  
  String messageID = messageWithoutCRC.substring(0, firstSeparator);
  String originatorID = messageWithoutCRC.substring(firstSeparator + 1, secondSeparator);
  String senderID = messageWithoutCRC.substring(secondSeparator + 1, thirdSeparator);
  String recipientID = messageWithoutCRC.substring(thirdSeparator + 1, fourthSeparator);
  String messageContent = messageWithoutCRC.substring(fourthSeparator + 1, fifthSeparator);
  String relayID = messageWithoutCRC.substring(fifthSeparator + 1);
  
  String myId = getCustomNodeId(currentNodeId);
  if (originatorID == myId) {
    Serial.println("[WiFi Rx] Own message received, ignoring.");
  } else {
    if (recipientID == "ALL" || myId == originatorID || myId == recipientID) {
      addMessage(originatorID, messageID, senderID, recipientID, messageContent, "[WiFi]", relayID);
    } else {
      Serial.println("[WiFi Rx] Private message not for me, ignoring.");
    }
  }
}

void updateMeshData() {
  mesh.update();
  totalNodeCount = mesh.getNodeList().size();
  currentNodeId = mesh.getNodeId();
}

void initMesh()
{
  // make sure loadConfig() has already run so cfg_ssid/cfg_pass are filled
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);

  mesh.init( cfg_ssid.c_str(),          // SSID from settings page
             cfg_pass.c_str(),          // password (blank = open mesh)
             MESH_PORT,
             WIFI_AP_STA,
             MESH_CHANNEL );

  mesh.onReceive(receivedCallback);
  mesh.onChangedConnections([](){
    updateMeshData();
  });

  // mesh.setContainsRoot(false);   // leave commented unless you really need it
}

// -----------------------------
// Additional Functions for Node Count & Unknown Nodes
// -----------------------------
// Returns the union of WiFi nodes and any nodes seen in messages, excluding our own node.
int getTotalNodeCount() {
  std::set<String> uniqueNodes;
  String myId = getCustomNodeId(currentNodeId);
  // Add WiFi nodes:
  auto wifiNodes = mesh.getNodeList();
  for (uint32_t id : wifiNodes) {
    String nodeStr = getCustomNodeId(id);
    if (nodeStr != myId) {
      uniqueNodes.insert(nodeStr);
    }
  }
  // Also add nodes from message history:
  for (auto const &msg : messageList) {
    if (msg.nodeId != myId) {
      uniqueNodes.insert(msg.nodeId);
    }
  }
  return uniqueNodes.size();
}

// Returns a vector of "unknown" node IDs – those that appear in messages but are not in the WiFi list.
// Excludes our own node.
std::vector<String> getUnknownNodes() {
  std::set<String> wifiSet;
  auto wifiNodes = mesh.getNodeList();
  String myId = getCustomNodeId(currentNodeId);
  for (uint32_t id : wifiNodes) {
    wifiSet.insert(getCustomNodeId(id));
  }
  std::set<String> unknownSet;
  for (auto const &msg : messageList) {
    if (msg.nodeId != myId) {
      unknownSet.insert(msg.nodeId);
    }
  }
  // Remove any that are in the WiFi set:
  for (auto const &w : wifiSet) {
    unknownSet.erase(w);
  }
  // Convert set to vector:
  std::vector<String> result;
  for (auto const &s : unknownSet) {
    result.push_back(s);
  }
  return result;
}

// -----------------------------
// Web Server Pages & Routes
// -----------------------------
const char mainPageHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Meshmingle Chat Room</title>
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
          `Mesh Nodes: <a href="/nodes">${data.totalCount}</a>, Node ID: ${data.nodeId}`;
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
</script>
</head>
<body>
  <div class="warning">For your safety, do not share your location or any personal information!</div>
  
  <h2>Meshmingle Chat</h2>
  
  <div id="deviceCount">Mesh Nodes: 0</div>
  
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
          // Display WiFi (mesh) nodes:
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

          // Display Lora nodes (renamed from "unknown" nodes):
          const loraUl = document.getElementById('loraNodeList');
          loraUl.innerHTML = '';
          const loraCount = data.unknownNodes.length;
          document.getElementById('loraCount').innerText = 'Lora Nodes Active: ' + loraCount;
          data.unknownNodes.forEach((node, index) => {
            const li = document.createElement('li');
            li.classList.add('node', 'lora');
            li.innerHTML = `
              <div class="node-header">
                <strong>Node ${index + 1}:</strong>
                <span>${node}</span>
              </div>
              <div class="node-info"></div>
            `;
            loraUl.appendChild(li);
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
    <a href="/">Chat</a>
  </div>
  <h2>Meshmingle Nodes</h2>
  
  <div class="node-section">
    <span id="wifiCount">WiFi Nodes Connected: 0</span>
    <ul id="wifiNodeList"></ul>
  </div>

  <div class="node-section">
    <span id="loraCount">Lora Nodes Active: 0</span>
    <ul id="loraNodeList"></ul>
  </div>
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
    <a href="/clients">Clients</a>
  </div>
  <h2>All Clients (Past &amp; Present)</h2>
  <ul id="macList"></ul>
</body>
</html>
)rawliteral";

// Setup all server routes:
void setupServerRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", mainPageHtml);
  });
  server.on("/nodes", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", nodesPageHtml);
  });
  server.on("/messages", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = "{\"messages\":[";
    bool first = true;
    for (const auto& msg : messageList) {
      if (!first) json += ",";
      json += "{\"nodeId\":\"" + msg.nodeId + "\",\"sender\":\"" + msg.sender + "\",\"recipient\":\"" + msg.recipient +
              "\",\"content\":\"" + msg.content + "\",\"source\":\"" + msg.source +
              "\",\"messageID\":\"" + msg.messageID + "\",\"relayID\":\"" + msg.relayID +
              "\",\"timeReceived\":" + String(msg.timeReceived) + "}";
      first = false;
    }
    json += "], \"currentDeviceTime\":" + String(millis()) + "}";
    request->send(200, "application/json", json);
  });
  // Updated deviceCount endpoint returning both meshCount and totalCount.
  server.on("/deviceCount", HTTP_GET, [](AsyncWebServerRequest* request) {
    updateMeshData();
    int meshCount = mesh.getNodeList().size();
    int totalCount = getTotalNodeCount();
    String json = "{\"meshCount\":" + String(meshCount) +
                  ", \"totalCount\":" + String(totalCount) +
                  ", \"nodeId\":\"" + getCustomNodeId(currentNodeId) + "\"}";
    request->send(200, "application/json", json);
  });
  // Updated nodesData endpoint returning both wifiNodes and unknownNodes.
  server.on("/nodesData", HTTP_GET, [](AsyncWebServerRequest* request) {
    updateMeshData();
    String json = "{\"wifiNodes\":[";
    auto wifiNodeList = mesh.getNodeList();
    bool firstWifi = true;
    std::set<String> wifiSet;
    for (auto node : wifiNodeList) {
      String nodeStr = getCustomNodeId(node);
      wifiSet.insert(nodeStr);
      if (!firstWifi) json += ",";
      json += "\"" + nodeStr + "\"";
      firstWifi = false;
    }
    json += "], \"unknownNodes\":[";
    std::vector<String> unknownNodes = getUnknownNodes();
    bool firstUnknown = true;
    for (auto const &n : unknownNodes) {
      if (!firstUnknown) json += ",";
      json += "\"" + n + "\"";
      firstUnknown = false;
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
    // Sanitize input
    newMessage.replace("<", "&lt;");
    newMessage.replace(">", "&gt;");
    senderName.replace("<", "&lt;");
    senderName.replace(">", "&gt;");
    target.replace("<", "&lt;");
    target.replace(">", "&gt;");
    if (target.length() == 0) {
         target = "ALL";
    }
    String messageID = generateMessageID(getCustomNodeId(currentNodeId));
    String relayID = getCustomNodeId(currentNodeId);
    String constructedMessage = constructMessage(messageID, getCustomNodeId(currentNodeId),
                                                  senderName, target, newMessage, relayID);
    addMessage(getCustomNodeId(currentNodeId), messageID, senderName, target, newMessage, "[WiFi]", relayID);
    Serial.printf("[WiFi Tx] Adding message: %s\n", constructedMessage.c_str());
    messageTransmissions[messageID].origin = ORIGIN_WIFI;
    transmitViaWiFi(constructedMessage);
    request->redirect("/");
  });

    // --- Clients page: serves HTML ---
  server.on("/clients", HTTP_GET, [](AsyncWebServerRequest* request){
    updateClientList();  // refresh our master list
    request->send_P(200, "text/html", clientsPageHtml);
  });

  // --- JSON endpoint for JS to pull MAC list ---
  server.on("/clientsData", HTTP_GET, [](AsyncWebServerRequest* request){
    updateClientList();
    String json = "{\"macs\":[";
    bool first = true;
    for (const auto& m : clientMacs) {
      if (!first) json += ",";
      json += "\"" + m + "\"";
      first = false;
    }
    json += "]}";
    request->send(200, "application/json", json);
  });

}

// Initializes the web server.
void setupServer() {
  setupServerRoutes();
  WiFi.mode(WIFI_AP_STA);           // 2) use the stored credentials
  mesh.init(cfg_ssid.c_str(), cfg_pass.c_str(), MESH_PORT, WIFI_AP_STA, MESH_CHANNEL);
  server.begin();
  setupSettingsRoutes(); // AFTER server is running
  dnsServer.start(53, "*", WiFi.softAPIP());
}

// -----------------------------
// Setup and Loop
// -----------------------------
void setup() {
  Serial.begin(115200);
  loadConfig();
  // Initialize the mesh network:
  //initMesh();
  // Set up the web server:
  setupServer();
  // (Optional: additional WiFi configuration if needed.)
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

void loop() {
  mesh.update();
  dnsServer.processNextRequest();
  updateMeshData();
}
