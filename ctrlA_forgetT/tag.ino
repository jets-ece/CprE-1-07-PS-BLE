#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CMD_UUID     "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // anchor -> tag
#define EVT_UUID     "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // tag -> anchor

#define ORDER_TIMEOUT_MS 10000
#define ADV_FAST_UNITS   160    // 100 ms  (units of 0.625 ms) - unassigned
#define ADV_SLOW_UNITS   1024   // 640 ms  - assigned ("advertise less")

enum : uint8_t {
  MSG_READY = 1, MSG_POLL = 2, MSG_POLLACK = 3, MSG_WAIT = 4,
  MSG_ASSIGN = 5, MSG_POLL_REQ = 6, MSG_UNASSIGNED = 7
};

struct __attribute__((packed)) Msg {
  uint8_t  type;
  uint8_t  seq;
  uint32_t t2;
  uint32_t t3;
};

BLEServer *server = nullptr;
BLECharacteristic *evtChar = nullptr;
char tagName[12];

volatile bool connected = false;
volatile bool advDirty = true;

volatile bool assigned = false;
volatile uint8_t order = 0;
volatile uint32_t lastContact = 0;

volatile bool assignFlag = false;
volatile bool reqFlag = false;
volatile bool unassignedFlag = false;
volatile bool pollFlag = false;
volatile bool waitFlag = false;
volatile uint32_t t2 = 0;
volatile uint8_t pollSeq = 0;

void applyAdv() {
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->stop();
  uint16_t itv = assigned ? ADV_SLOW_UNITS : ADV_FAST_UNITS;
  adv->setMinInterval(itv);
  adv->setMaxInterval(itv);
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();
  Serial.printf("Advertising %s (%s)\n", tagName, assigned ? "slow, assigned" : "fast, unassigned");
}

class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer *) {
    connected = true;
    reqFlag = pollFlag = waitFlag = unassignedFlag = false;
  }
  void onDisconnect(BLEServer *) {
    connected = false;
    advDirty = true;                  // loop() restarts advertising with the right interval
  }
};

class CmdCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    uint32_t now = micros();          // t2 stamped as early as possible
    if (c->getLength() < sizeof(Msg)) return;
    Msg m;
    memcpy(&m, c->getData(), sizeof(Msg));
    lastContact = millis();

    switch (m.type) {
      case MSG_ASSIGN:
        order = m.seq;
        assigned = true;
        assignFlag = true;
        advDirty = true;
        break;
      case MSG_POLL_REQ:
        if (assigned) reqFlag = true; else unassignedFlag = true;
        break;
      case MSG_POLL:
        if (assigned) { t2 = now; pollSeq = m.seq; pollFlag = true; }
        break;
      case MSG_WAIT:
        waitFlag = true;
        break;
    }
  }
};

void notifyMsg(const Msg &m) {
  evtChar->setValue((uint8_t *)&m, sizeof(m));
  evtChar->notify();
}

void setup() {
  Serial.begin(115200);
  snprintf(tagName, sizeof(tagName), "TAG-%04X", (unsigned)(esp_random() & 0xFFFF));
  BLEDevice::init(tagName);
  Serial.printf("Tag name: %s\n", tagName);

  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCb());
  BLEService *svc = server->createService(SERVICE_UUID);

  BLECharacteristic *cmd = svc->createCharacteristic(
      CMD_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  cmd->setCallbacks(new CmdCb());

  evtChar = svc->createCharacteristic(EVT_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  evtChar->addDescriptor(new BLE2902());

  svc->start();
}

void loop() {
  // Order timeout: anchor took too long, forget and advertise as normal
  if (assigned && millis() - lastContact > ORDER_TIMEOUT_MS) {
    assigned = false;
    Serial.printf("Order %u timed out (10 s), forgotten\n", order);
    advDirty = true;
  }

  if (advDirty && !connected) {
    advDirty = false;
    applyAdv();
  }

  if (assignFlag) {
    assignFlag = false;
    Serial.printf("ASSIGNED as order %u\n", order);
  }

  if (unassignedFlag) {
    unassignedFlag = false;
    Msg m = {MSG_UNASSIGNED, 0, 0, 0};
    notifyMsg(m);
    Serial.println("Poll request received but no order assigned");
  }

  if (reqFlag) {                       // anchor asked first, now we answer READY
    reqFlag = false;
    Msg m = {MSG_READY, order, 0, 0};
    notifyMsg(m);
    Serial.printf("READY (order %u)\n", order);
  }

  if (pollFlag) {
    pollFlag = false;
    Msg m = {MSG_POLLACK, pollSeq, t2, 0};
    m.t3 = micros();                   // t3 stamped right before sending
    notifyMsg(m);
  }

  if (waitFlag) {
    waitFlag = false;
    Serial.printf("WAITING (order %u)\n", order);
  }

  delay(1);
}
