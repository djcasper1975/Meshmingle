#include "node_nickname_feature.h"

static Preferences prefs;
static std::map<String, String> nicknames; // nodeId → nickname

void loadNodeNicknames() {
    prefs.begin("nicknames", false);
    size_t count = prefs.getUInt("count", 0);
    nicknames.clear();
    for (size_t i = 0; i < count; ++i) {
        String id = prefs.getString(("id" + String(i)).c_str(), "");
        String nick = prefs.getString(("nick" + String(i)).c_str(), "");
        if (id.length() && nick.length()) {
            nicknames[id] = nick;
        }
    }
    prefs.end();
}

void saveNodeNicknames() {
    prefs.begin("nicknames", false);
    prefs.clear();
    size_t i = 0;
    for (auto const& kv : nicknames) {
        prefs.putString(("id" + String(i)).c_str(), kv.first);
        prefs.putString(("nick" + String(i)).c_str(), kv.second);
        ++i;
    }
    prefs.putUInt("count", i);
    prefs.end();
}

void setNodeNickname(const String& nodeId, const String& nickname) {
    if (!nodeId.length()) return;
    if (nickname.length() == 0) {
        nicknames.erase(nodeId); // Remove
    } else {
        nicknames[nodeId] = nickname;
    }
    saveNodeNicknames();
}

String getNodeNickname(const String& nodeId) {
    auto it = nicknames.find(nodeId);
    if (it != nicknames.end()) return it->second;
    return "";
}

const std::map<String, String>& getAllNodeNicknames() {
    return nicknames;
}
