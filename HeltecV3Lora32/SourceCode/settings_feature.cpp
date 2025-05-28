#include <Preferences.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>  // for WIFI_POWER_xxx constants
extern AsyncWebServer server;  // declared in main sketch

/********************************* PERSISTENT CONFIG *********************************/
Preferences settings;

String  cfg_ssid          = "meshmingle.co.uk";
String  cfg_pass          = "";
float   cfg_freq_mhz      = 869.4000;
bool    cfg_duty_override = false;
uint8_t cfg_channel       = 3;
bool    cfg_lora_enabled  = true;   // default = On
wifi_power_t cfg_wifi_power = WIFI_POWER_19_5dBm;
uint8_t cfg_lora_power    = 22;  // or your existing default, e.g. 22

void loadConfig() {
  settings.begin("config", false);
  cfg_ssid          = settings.getString("ssid",  cfg_ssid);
  cfg_pass          = settings.getString("pass",  cfg_pass);
  cfg_freq_mhz      = settings.getFloat ("freq",  cfg_freq_mhz);
  cfg_duty_override = settings.getBool  ("duty",  cfg_duty_override);
  cfg_channel       = settings.getUChar ("channel", cfg_channel);
  cfg_lora_enabled  = settings.getBool ("lora",    cfg_lora_enabled);
  cfg_wifi_power    = (wifi_power_t)settings.getUChar("wifi_pwr", cfg_wifi_power);
  cfg_lora_power    = settings.getUChar   ("lora_pwr", cfg_lora_power);
  settings.end();
}

void saveConfig(const String &ssid, const String &pass, float freq, bool dutyOvrd, uint8_t channel, bool loraEn, wifi_power_t wifiPower, uint8_t loraPower) {
  settings.begin("config", false);
  settings.putString("ssid",  ssid);
  settings.putString("pass",  pass);
  settings.putFloat ("freq",  freq);
  settings.putBool  ("duty",  dutyOvrd);
  settings.putUChar ("channel", channel);
  settings.putBool  ("lora",    loraEn);
  settings.putUChar ("wifi_pwr", wifiPower);
  settings.putUChar ("lora_pwr", loraPower);
  settings.end();

  cfg_ssid          = ssid;
  cfg_pass          = pass;
  cfg_freq_mhz      = freq;
  cfg_duty_override = dutyOvrd;
  cfg_channel       = channel;
  cfg_lora_enabled  = loraEn;
  cfg_wifi_power    = wifiPower;
  cfg_lora_power    = loraPower;
}

/************************************* HTML *******************************************/
const char settingsPageHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Meshmingle settings</title>
  <style>
    body{font-family:Arial,Helvetica,sans-serif;margin:20px;max-width:480px;}
    label{display:block;margin-top:14px;font-weight:600;}
    input,select{width:100%;padding:6px;border:1px solid #bbb;border-radius:4px;}
    button{margin-top:22px;padding:8px 24px;border:none;border-radius:6px;box-shadow:0 2px 6px rgba(0,0,0,.2);cursor:pointer;}
    .banner{background:#d4edda;border:1px solid #28a745;padding:8px 12px;border-radius:4px;color:#155724;margin-bottom:16px;display:%BANNER%;}
    .error{background:#f8d7da;border:1px solid #dc3545;color:#721c24;}
  </style>
</head><body>
  <div class="banner %ERRCLS%">%MSG%</div>
  <h2>Device settings</h2>
  <small>All mesh nodes must use the same WiFi Details.</small>
  <form action="/saveSettings" method="post">
    <label>WiFi name (SSID)
      <input name="ssid" value="%SSID%" maxlength="32" required>
    </label>
    <label>Password
      <input name="pass" type="password" value="%PASS%" maxlength="63"
             pattern="[\\x20-\\x7E]{0,63}"
             placeholder="leave blank for open network"
             title="Leave blank for open network, or 8 to 63 characters">
    </label>
    <label>WiFi Channel (1 to 13)
      <input name="channel" type="number" min="1" max="13" value="%CHAN%" required>
    </label>
    <label>Wi-Fi TX Power
      <select name="wifi_power">
        <option value="WIFI_POWER_19_5dBm" %WIFI19%>19.5 dBm</option>
        <option value="WIFI_POWER_17dBm"   %WIFI17%>17   dBm</option>
        <option value="WIFI_POWER_15dBm"   %WIFI15%>15   dBm</option>
        <option value="WIFI_POWER_13dBm"   %WIFI13%>13   dBm</option>
        <option value="WIFI_POWER_11dBm"   %WIFI11%>11   dBm</option>
      </select>
    </label>
    <label>Region (sets legal LoRa frequency and duty cycle)
      <select name="region">
        <option value="EU"%EUSEL%>EU / UK</option>
        <option value="USA"%USSEL%>USA</option>
      </select>
    </label>
    <label for="lora">LoRa Radio:</label>
      <select name="lora" id="lora">
        <option value="1" %LORA_ON%>On</option>
        <option value="0" %LORA_OFF%>Off</option>
      </select>
    <label>LoRa TX Power
      <select name="lora_power" required>
        %LORAPWR_OPTIONS%
      </select>
    </label>
    <button type="submit">Save</button>
  </form>
  <p style="margin-top:1.2em;font-size:.9em;color:#555;">After saving, reboot the node for changes to take effect.</p>
  <form action="/reboot" method="post"><button>Reboot now</button></form>
  <p><a href="/">Back to main page</a></p>
</body></html>
)rawliteral";

// Confirmation page shown while the device restarts
const char rebootPageHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
  <meta http-equiv="refresh" content="20; url=/">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Rebooting</title>
  <style>
    body{font-family:Arial,Helvetica,sans-serif;text-align:center;padding-top:22vh;}
    h1{font-size:1.9em;margin-bottom:.4em;}
    p{color:#555;line-height:1.4em;max-width:320px;margin:0 auto;}
  </style>
  <script>
    const ping = () => fetch('/')
        .then(r=>{if(r.ok) window.location.replace('/');})
        .catch(()=>{});
    setInterval(ping, 3000);
  </script>
</head><body>
  <h1>Rebooting</h1>
  <p>Your node is restarting.<br>This page will load the main interface automatically once its online or another mesh node.</p>
</body></html>
)rawliteral";


/********************************* WEB ROUTES ****************************************/
static bool validPassword(const String &pw){
  // Allow blank password for an open network. Otherwise enforce 8–63 printable ASCII.
  if(pw.length() == 0) return true;
  if(pw.length() < 8 || pw.length() > 63) return false;
  for(char c : pw){
    if(c < 0x20 || c > 0x7E) return false;
  }
  return true;
}

void setupSettingsRoutes() {
  // serve settings form
  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *req) {
    String page = FPSTR(settingsPageHtml);
    page.replace("%SSID%", cfg_ssid);
    page.replace("%PASS%", cfg_pass);

    bool isUSA = (cfg_freq_mhz > 900.0f);
    page.replace("%EUSEL%", isUSA ? "" : " selected");
    page.replace("%USSEL%", isUSA ? " selected" : "");
    page.replace("%CHAN%",  String(cfg_channel));
    page.replace("%LORA_ON%",  cfg_lora_enabled ? "selected" : "");
    page.replace("%LORA_OFF%", cfg_lora_enabled ? ""         : "selected");

    // Generate LoRa power options (2–22 dBm)
    String loraOptions = "";
    for(uint8_t p = 2; p <= 22; p++) {
      loraOptions += "<option value=\"" + String(p) + "\"";
      if(p == cfg_lora_power) loraOptions += " selected";
      loraOptions += ">" + String(p) + " dBm</option>";
    }
    page.replace("%LORAPWR_OPTIONS%", loraOptions);

    bool saved = req->hasParam("saved");
    bool err   = req->hasParam("err");
    page.replace("%ERRCLS%", err ? "error" : "");
    page.replace("%BANNER%", (saved||err) ? "block" : "none");
    page.replace("%MSG%", err
      ? "Invalid password leave blank or use 8 63 printable ASCII characters."
      : "Settings saved reboot to apply.");

    page.replace("%WIFI19%", cfg_wifi_power==WIFI_POWER_19_5dBm ? "selected":"");
    page.replace("%WIFI17%", cfg_wifi_power==WIFI_POWER_17dBm   ? "selected":"");
    page.replace("%WIFI15%", cfg_wifi_power==WIFI_POWER_15dBm   ? "selected":"");
    page.replace("%WIFI13%", cfg_wifi_power==WIFI_POWER_13dBm   ? "selected":"");
    page.replace("%WIFI11%", cfg_wifi_power==WIFI_POWER_11dBm   ? "selected":"");

    req->send(200, "text/html", page);
  });

  // handle save
  server.on("/saveSettings", HTTP_POST, [](AsyncWebServerRequest *req) {
    String nSsid  = req->getParam("ssid", true)->value();
    String nPass  = req->getParam("pass", true)->value();
    String region = req->getParam("region", true)->value();

    if(!validPassword(nPass)){
      req->redirect("/settings?err=1");
      return;
    }

    String wv = req->getParam("wifi_power", true)->value();
    wifi_power_t newWifi = WIFI_POWER_19_5dBm;
    if      (wv == "WIFI_POWER_17dBm") newWifi = WIFI_POWER_17dBm;
    else if (wv == "WIFI_POWER_15dBm") newWifi = WIFI_POWER_15dBm;
    else if (wv == "WIFI_POWER_13dBm") newWifi = WIFI_POWER_13dBm;
    else if (wv == "WIFI_POWER_11dBm") newWifi = WIFI_POWER_11dBm;

    uint8_t newLoRa = constrain(
      req->getParam("lora_power", true)->value().toInt(),
      2, 22
    );

    float newFreq = (region == "USA") ? 902.0000f : 869.4000f;
    bool  newDuty = (region == "USA");

    uint8_t newChan = constrain(
      req->getParam("channel", true)->value().toInt(), 1, 13
    );
    String loraVal = req->getParam("lora", true)->value();  // "1" or "0"
    bool   nLora   = (loraVal == "1");
    saveConfig(nSsid, nPass, newFreq, newDuty, newChan, nLora, newWifi, newLoRa);
    req->redirect("/settings?saved=1");
  });

  // reboot endpoint remains unchanged
  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *req){
    req->send_P(200, "text/html", rebootPageHtml);
    delay(400);
    ESP.restart();
  });
}
