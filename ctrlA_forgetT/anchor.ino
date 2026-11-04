#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>

#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CMD_UUID     "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // anchor -> tag
#define EVT_UUID     "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // tag -> anchor

enum : uint8_t {
  MSG_READY = 1, MSG_POLL = 2, MSG_POLLACK = 3, MSG_WAIT = 4,
  MSG_ASSIGN = 5, MSG_POLL_REQ = 6, MSG_UNASSIGNED = 7
};

struct __attribute__((packed)) Msg {
  uint8_t  type;
  uint8_t  seq;      // order for ASSIGN/POLL_REQ, poll sequence otherwise
  uint32_t t2;
  uint32_t t3;
};

// ---------- settings ----------
#define MAX_TAGS         8
#define SCAN_SEC         4      // initial scan phase length
#define QUICK_SCAN_SEC   1      // temporary scan at end of every poll cycle
#define RESP_TIMEOUT_MS  1000
#define POST_WAIT_DELAY  1000   // pause after each successful WAIT
#define MAX_FAILS        3
#define TESTS_PER_BATCH  10

// ---------- tag table ----------
struct TagInfo {
  BLEAddress addr = BLEAddress("");
  char      addrStr[20];
  char      name[24];
  uint8_t   order;
  uint8_t   fails;
  uint32_t  count;
  uint64_t  sum;
};

TagInfo tags[MAX_TAGS];
int tagCount = 0;
uint8_t nextOrder = 1;

BLEAddress pendingAddr[MAX_TAGS] = {
  BLEAddress(""), BLEAddress(""), BLEAddress(""), BLEAddress(""),
  BLEAddress(""), BLEAddress(""), BLEAddress(""), BLEAddress("")
};
char pendingNames[MAX_TAGS][24];
volatile int pendingCount = 0;

// ---------- state ----------
enum Phase { PH_SCAN, PH_POLL };
Phase phase = PH_SCAN;
int pollIdx = 0;
bool paused = false;
bool resetReq = false;

BLEClient *client = nullptr;
BLERemoteCharacteristic *cmdChar = nullptr;
BLERemoteCharacteristic *evtChar = nullptr;

volatile bool readyFlag = false;
volatile bool ackFlag = false;
volatile bool unassignedFlag = false;
volatile uint32_t t4 = 0;
Msg ackMsg;
uint32_t t1 = 0;
uint8_t seq = 0;

// ---------- helpers ----------
  
bool isKnown(const char *addr) {
  for (int i = 0; i < tagCount; i++) {
    if (strcmp(tags[i].addrStr, addr) == 0) return true;
  }
  for (int i = 0; i < pendingCount; i++) {
    if (strcmp(pendingAddr[i].toString().c_str(), addr) == 0) return true;
  }
  return false;
}

class AdvCb : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    bool uuidMatch = d.haveServiceUUID() && d.isAdvertisingService(BLEUUID(SERVICE_UUID));
    bool nameMatch = d.haveName() && strncmp(d.getName().c_str(), "TAG-", 4) == 0;

    if (!(uuidMatch || nameMatch)) return;

    String address = d.getAddress().toString();
    if (isKnown(address.c_str())) return;
    if (tagCount + pendingCount >= MAX_TAGS) return;

    pendingAddr[pendingCount] = d.getAddress();
    strncpy(pendingNames[pendingCount], d.haveName() ? d.getName().c_str() : "?", sizeof(pendingNames[0]) - 1);
    pendingCount++;

    Serial.printf("[SCAN] New tag detected: %s\n", address.c_str());
  }
};

AdvCb advCb;

static void onNotify(BLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
  uint32_t now = micros();
  if (len < sizeof(Msg)) return;
  Msg m;
  memcpy(&m, data, sizeof(Msg));
  if (m.type == MSG_READY)           readyFlag = true;
  else if (m.type == MSG_UNASSIGNED) unassignedFlag = true;
  else if (m.type == MSG_POLLACK)    { ackMsg = m; t4 = now; ackFlag = true; }
}

void closeTag() {
  cmdChar = nullptr;
  evtChar = nullptr;
  if (client) {
    if (client->isConnected()) {
      client->disconnect();
      delay(50);
    }
    delete client; // Call delete on the BLEClient instance directly
    client = nullptr;
  }
}

bool openTag(TagInfo &t) {
  closeTag();
  client = BLEDevice::createClient();
  if (!client) return false;
  
  if (!client->connect(t.addr)) { 
    Serial.println("  connect failed"); 
    closeTag(); 
    return false; 
  }
  
  BLERemoteService *svc = client->getService(BLEUUID(SERVICE_UUID));
  if (!svc) { 
    Serial.println("  service not found"); 
    closeTag(); 
    return false; 
  }
  
  cmdChar = svc->getCharacteristic(BLEUUID(CMD_UUID));
  evtChar = svc->getCharacteristic(BLEUUID(EVT_UUID));
  
  if (!cmdChar || !evtChar || !evtChar->canNotify()) {
    Serial.println("  characteristics missing"); 
    closeTag(); 
    return false;
  }
  
  evtChar->registerForNotify(onNotify);
  return true;
}

bool sendCmd(uint8_t type, uint8_t s, bool withResponse) {
  if (!client || !client->isConnected() || !cmdChar) return false;
  Msg m = {type, s, 0, 0};
  cmdChar->writeValue((uint8_t *)&m, sizeof(m), withResponse);
  return true;
}

// ---------- serial control (anchor only) ----------
char cmdBuf[16];
uint8_t cmdLen = 0;

void printStatus() {
  Serial.printf("[status] phase=%s paused=%d tags=%d\n",
                phase == PH_SCAN ? "SCAN" : "POLL", paused, tagCount);
  for (int i = 0; i < tagCount; i++)
    Serial.printf("  order %u  %s  %s  samples=%lu  fails=%u\n", tags[i].order,
                  tags[i].name, tags[i].addrStr, (unsigned long)tags[i].count, tags[i].fails);
}

void runCmd(const char *c) {
  if (!strcmp(c, "pause") || !strcmp(c, "p")) {
    paused = true;  Serial.println("[cmd] PAUSED (takes effect at the next tag boundary)");
  } else if (!strcmp(c, "continue") || !strcmp(c, "c") || !strcmp(c, "resume")) {
    paused = false; Serial.println("[cmd] CONTINUE");
  } else if (!strcmp(c, "reset") || !strcmp(c, "r")) {
    resetReq = true; Serial.println("[cmd] RESET requested");
  } else if (!strcmp(c, "status") || !strcmp(c, "s")) {
    printStatus();
  } else {
    Serial.println("[cmd] unknown. Use: pause | continue | reset | status");
  }
}

void handleSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      cmdBuf[cmdLen] = 0;
      if (cmdLen) runCmd(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = tolower(ch);
    }
  }
}

bool waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    handleSerial();
    if (resetReq) return false;
    delay(10);
  }
  return true;
}

void doReset() {
  closeTag();
  tagCount = 0; 
  pendingCount = 0; 
  nextOrder = 1; 
  pollIdx = 0;
  phase = PH_SCAN; 
  resetReq = false;
  Serial.println("\n[RESET] tag list cleared, back to SCAN phase");
}

// ---------- scan phase ----------
int scanAndAssign(int seconds) {
  pendingCount = 0;
  BLEScan *scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&advCb);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  scan->start(seconds, false);
  waitMs(seconds * 1000UL + 200);
  scan->stop();
  scan->clearResults();
  if (resetReq) return 0;

  int assigned = 0;
  for (int i = 0; i < pendingCount; i++) {
    if (tagCount >= MAX_TAGS) continue;

    TagInfo &t = tags[tagCount];
    memset(&t, 0, sizeof(t));
    t.addr = pendingAddr[i];
    strncpy(t.addrStr, pendingAddr[i].toString().c_str(), sizeof(t.addrStr) - 1);
    strncpy(t.name, pendingNames[i], sizeof(t.name) - 1);
    t.order = nextOrder;

    Serial.printf("Found new tag %s (%s), assigning order %u\n", t.name, t.addrStr, t.order);
    bool ok = openTag(t) && sendCmd(MSG_ASSIGN, t.order, true);
    closeTag();
    if (ok) {
      tagCount++; 
      nextOrder++; 
      assigned++;
      Serial.printf("  -> order %u assigned\n", t.order);
    } else {
      Serial.println("  -> assign failed, will retry on a later scan");
    }
  }
  pendingCount = 0;
  return assigned;
}

// ---------- poll phase ----------
enum { POLL_OK, POLL_FAIL, POLL_REASSIGNED };

int pollTag(TagInfo &t) {
  Serial.printf("Polling order %u (%s)\n", t.order, t.name);
  if (!openTag(t)) return POLL_FAIL;

  int r = POLL_FAIL;
  readyFlag = ackFlag = unassignedFlag = false;

  do {
    // 1) POLL_REQ -> tag answers READY (or UNASSIGNED)
    if (!sendCmd(MSG_POLL_REQ, t.order, true)) break;
    
    uint32_t t0 = millis();
    while (!readyFlag && !unassignedFlag && millis() - t0 < RESP_TIMEOUT_MS) delay(1);

    if (unassignedFlag) {
      Serial.println("  tag forgot its order, re-assigning");
      sendCmd(MSG_ASSIGN, t.order, true);
      r = POLL_REASSIGNED;
      break;
    }
    if (!readyFlag) { Serial.println("  no READY"); break; }
    readyFlag = false;

    // 2) POLL
    seq++;
    t1 = micros();
    if (!sendCmd(MSG_POLL, seq, false)) break;

    // 3) POLLACK
    t0 = millis();
    while (!ackFlag && millis() - t0 < RESP_TIMEOUT_MS) delay(1);
    if (!ackFlag) { Serial.println("  POLLACK timeout"); break; }
    ackFlag = false;

    Msg a = ackMsg;
    uint32_t t4v = t4;
    if (a.seq != seq) { Serial.println("  stale POLLACK ignored"); break; }

    uint32_t Tro = t4v - t1;
    uint32_t Trp = a.t3 - a.t2;

    // 4) WAIT
    if (!sendCmd(MSG_WAIT, seq, true)) { Serial.println("  WAIT failed"); break; }
    r = POLL_OK;

    if (Tro <= Trp) {
      Serial.println("  invalid sample (Tro <= Trp), skipped");
    } else {
      uint32_t tprop = (Tro - Trp) / 2;
      t.count++;
      t.sum += tprop;
      Serial.printf("  [order %u] Test %lu: Tro=%lu us, Trp=%lu us, Tprop=%lu us\n",
                    t.order, (unsigned long)t.count, (unsigned long)Tro,
                    (unsigned long)Trp, (unsigned long)tprop);
      if (t.count % TESTS_PER_BATCH == 0) {
        Serial.printf("  [order %u] Average Tprop over %lu tests: %.2f us\n",
                      t.order, (unsigned long)t.count, (double)t.sum / t.count);
        t.count = 0; t.sum = 0;
      }
    }
  } while (0);

  closeTag();
  return r;
}

void removeTag(int idx) {
  Serial.printf("Dropping order %u (%s) after %d failures\n", tags[idx].order, tags[idx].name, MAX_FAILS);
  for (int i = idx; i < tagCount - 1; i++) tags[i] = tags[i + 1];
  tagCount--;
}

// ---------- arduino ----------
void setup() {
  Serial.begin(115200);
  BLEDevice::init("BLE-ANCHOR");
  Serial.println("Anchor ready. Commands (end with Newline): pause | continue | reset | status");
}

void loop() {
  handleSerial();
  if (resetReq) { doReset(); return; }
  if (paused) { delay(20); return; }

  if (phase == PH_SCAN) {
    Serial.println("\n=== SCAN PHASE ===");
    scanAndAssign(SCAN_SEC);
    if (resetReq) return;
    if (tagCount > 0) {
      phase = PH_POLL; pollIdx = 0;
      Serial.println("=== POLL PHASE ===");
    }
    return;
  }

  // POLL phase
  if (tagCount == 0) { phase = PH_SCAN; return; }

  if (pollIdx >= tagCount) {
    Serial.println("\n--- temporary SCAN phase (checking for new tags) ---");
    int n = scanAndAssign(QUICK_SCAN_SEC);
    if (resetReq) return;
    if (n > 0) Serial.println("New tag(s) assigned, repolling from the 1st tag");
    pollIdx = 0;
    Serial.println("=== POLL PHASE ===");
    return;
  }

  TagInfo &t = tags[pollIdx];
  int r = pollTag(t);

  if (r == POLL_OK || r == POLL_REASSIGNED) {
    t.fails = 0;
    pollIdx++;
    if (r == POLL_OK) waitMs(POST_WAIT_DELAY);
  } else {
    if (t.fails < 255) t.fails++;
    Serial.printf("[WARNING] Order %u communication failure (%u)\n", t.order, t.fails);
    pollIdx++;
  } 
}
