#include "recent_nodes_feature.h"

extern uint32_t getNodeId();                 //  defined in the main .ino
extern String   getCustomNodeId(uint32_t);   //  ditto

static Preferences   prefs;
static std::set<String> recent;          // lives for the life of the sketch

/**************  STORAGE FORMAT  ****************
 * NVS namespace  "recent"
 *   key "list" →   comma-separated node ids  e.g. "!M123456,!MABCDEF"
 ************************************************/

void loadRecentNodes() {
  prefs.begin("recent", /*rw=*/false);
  String csv = prefs.getString("list", "");
  prefs.end();
  recent.clear();
  int start = 0, comma;
  while ((comma = csv.indexOf(',', start)) != -1) {
    recent.insert(csv.substring(start, comma));
    start = comma + 1;
  }
  if (start < (int)csv.length()) recent.insert(csv.substring(start));
}

void saveRecentNodes() {
  String csv;
  for (auto it = recent.begin(); it != recent.end(); ++it) {
    if (it != recent.begin()) csv += ",";
    csv += *it;
  }
  prefs.begin("recent", /*rw=*/false);
  prefs.putString("list", csv);
  prefs.end();
}

void addRecentNode(const String& id) {
  if (id.isEmpty()) return;
  if (id == getCustomNodeId(getNodeId())) return;        // ← NEW: skip me
  if (recent.insert(id).second) {   // true ⇒ it was new
    saveRecentNodes();
  }
}

const std::set<String>& getRecentNodes() { return recent; }
