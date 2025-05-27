#include <Preferences.h>
#include <ESPAsyncWebServer.h>

extern AsyncWebServer server;  // declared in main sketch

/********************************* PERSISTENT CONFIG *********************************/
Preferences settings;

String  cfg_ssid          = "meshmingle.co.uk";
String  cfg_pass          = "";
float   cfg_freq_mhz      = 869.4000;
bool    cfg_duty_override = false;
uint8_t cfg_channel       = 3;

void loadConfig() {
  settings.begin("config", false);
  cfg_ssid          = settings.getString("ssid",  cfg_ssid);
  cfg_pass          = settings.getString("pass",  cfg_pass);
  cfg_freq_mhz      = settings.getFloat ("freq",  cfg_freq_mhz);
  cfg_duty_override = settings.getBool  ("duty",  cfg_duty_override);
  cfg_channel       = settings.getUChar ("channel", cfg_channel);
  settings.end();
}

void saveConfig(const String &ssid, const String &pass, float freq, bool dutyOvrd, uint8_t channel) {
  settings.begin("config", false);
  settings.putString("ssid",  ssid);
  settings.putString("pass",  pass);
  settings.putFloat ("freq",  freq);
  settings.putBool  ("duty",  dutyOvrd);
  settings.putUChar("channel", channel);
  settings.end();

  cfg_ssid          = ssid;
  cfg_pass          = pass;
  cfg_freq_mhz      = freq;
  cfg_duty_override = dutyOvrd;
  cfg_channel       = channel;
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
    <label>Region (sets legal LoRa frequency and duty cycle)
      <select name="region">
        <option value="EU"%EUSEL%>EU / UK</option>
        <option value="USA"%USSEL%>USA</option>
      </select>
    </label>
    <button type="submit">Save</button>
  </form>
  <p style="margin-top:1.2em;font-size:.9em;color:#555;">After saving, reboot the node for changes to take effect.</p>
  <form action="/reboot" method="post"><button>Reboot now</button></form>
  <p><a href="/">Back to main page</a></p>
</body></html>
)rawliteral";

const char rebootPageHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
  <meta http-equiv="refresh" content="20; url=/">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Rebooting…</title>
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

    bool saved = req->hasParam("saved");
    bool err   = req->hasParam("err");
    page.replace("%ERRCLS%", err ? "error" : "");
    page.replace("%BANNER%", (saved||err) ? "block" : "none");
    page.replace("%MSG%", err
      ? "Invalid password leave blank or use 8 63 printable ASCII characters."
      : "Settings saved reboot to apply.");

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

    float newFreq = (region == "USA") ? 902.0000f : 869.4000f;
    bool  newDuty = (region == "USA");

    uint8_t newChan = constrain(
      req->getParam("channel", true)->value().toInt(), 1, 13
    );
    saveConfig(nSsid, nPass, newFreq, newDuty, newChan);
    req->redirect("/settings?saved=1");
  });

  // reboot endpoint
  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *req){
    req->send_P(200, "text/html", rebootPageHtml);
    delay(400);
    ESP.restart();
  });
}
