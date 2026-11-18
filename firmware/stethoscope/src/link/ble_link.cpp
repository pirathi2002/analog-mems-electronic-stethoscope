#include "ble_link.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if defined(CONFIG_BLUEDROID_ENABLED)
#include <BLE2902.h>
#endif
#include <string.h>

#include "../app/app_state.h"
#include "../app/buffer_print.h"
#include "../app/commands.h"
#include "../config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace ble_link {

namespace {

struct BleBlock {
  uint32_t first_index;
  uint16_t n;
  int16_t raw[BLOCK_SAMPLES];
  int16_t filt[BLOCK_SAMPLES];
};

struct Cmd {
  char text[128];
};

// Connection interval request: 7.5-15 ms (units of 1.25 ms), supervision timeout 4 s.
constexpr uint16_t kConnIntervalMin = 6;
constexpr uint16_t kConnIntervalMax = 12;
constexpr uint16_t kSupervisionTimeout = 400;
// A notification that finds the controller buffers full is retried this often (1 ms apart)
// before the packet is dropped (and counted in ble_drops).
constexpr int kNotifyRetries = 20;

QueueHandle_t g_audio_q = nullptr;
QueueHandle_t g_cmd_q = nullptr;
BLEServer* g_server = nullptr;
BLECharacteristic* g_audio = nullptr;     // filtered only (legacy)
BLECharacteristic* g_audio_rf = nullptr;  // raw + filtered
BLECharacteristic* g_status = nullptr;
#if defined(CONFIG_BLUEDROID_ENABLED)
BLE2902* g_audio_rf_cccd = nullptr;
#endif
volatile bool g_rf_subscribed = false;  // NimBLE: set by onSubscribe
volatile bool g_notify_failed = false;  // set by onStatus during notify()

bool rfSubscribed() {
#if defined(CONFIG_BLUEDROID_ENABLED)
  return g_audio_rf_cccd && g_audio_rf_cccd->getNotifications();
#else
  return g_rf_subscribed;
#endif
}

uint16_t peerMtu() {
  uint16_t mtu = g_server ? g_server->getPeerMTU(g_server->getConnId()) : 23;
  return mtu < 23 ? 23 : mtu;
}

void notifyStatus(const char* text, size_t len) {
  if (!g_status || !app::bleConnected()) return;
  // Keep within one ATT packet; long replies (help) are split in chunks.
  const size_t chunk = peerMtu() - 3;
  for (size_t off = 0; off < len; off += chunk) {
    const size_t n = (len - off) < chunk ? (len - off) : chunk;
    g_status->setValue(reinterpret_cast<uint8_t*>(const_cast<char*>(text + off)), n);
    g_status->notify();
    if (off + n < len) delay(5);
  }
}

// Sends one notification, waiting briefly while the controller's buffers are full.
bool notifyAudio(BLECharacteristic* c, const uint8_t* data, size_t len) {
  for (int attempt = 0; attempt <= kNotifyRetries; ++attempt) {
    if (!app::bleConnected()) return false;
    g_notify_failed = false;
    c->setValue(const_cast<uint8_t*>(data), len);
    c->notify();
    if (!g_notify_failed) return true;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return false;
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    app::setBleConnected(true);
    (void)server;
  }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onConnect(BLEServer* server, esp_ble_gatts_cb_param_t* param) override {
    app::setBleConnected(true);
    // Ask for a short connection interval for audio throughput.
    server->requestConnParams(param->connect.remote_bda, kConnIntervalMin, kConnIntervalMax, 0,
                              kSupervisionTimeout);
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onConnect(BLEServer* server, ble_gap_conn_desc* desc) override {
    app::setBleConnected(true);
    server->requestConnParams(desc->conn_handle, kConnIntervalMin, kConnIntervalMax, 0,
                              kSupervisionTimeout);
  }
#endif
  void onDisconnect(BLEServer* server) override {
    app::setBleConnected(false);
    g_rf_subscribed = false;
    delay(50);
    server->startAdvertising();
  }
};

class AudioCallbacks : public BLECharacteristicCallbacks {
  void onStatus(BLECharacteristic* c, Status s, uint32_t code) override {
    (void)c;
    (void)code;
    if (s == Status::ERROR_GATT) g_notify_failed = true;
  }
#if defined(CONFIG_NIMBLE_ENABLED)
  void onSubscribe(BLECharacteristic* c, ble_gap_conn_desc* desc, uint16_t sub_value) override {
    (void)desc;
    if (c == g_audio_rf) g_rf_subscribed = (sub_value & 0x0001) != 0;
  }
#endif
};

class ControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    Cmd cmd;
    String v = c->getValue();
    strncpy(cmd.text, v.c_str(), sizeof(cmd.text) - 1);
    cmd.text[sizeof(cmd.text) - 1] = '\0';
    xQueueSend(g_cmd_q, &cmd, 0);  // executed in bleTask (bigger stack)
  }
};

// Raw + filtered: [u32 index][int16 raw[n]][int16 filt[n]]. 32 samples per packet
// (132 bytes) when the MTU allows, so a 64-sample block takes exactly 2 packets.
void sendRawFiltered(const BleBlock& blk) {
  static uint8_t pkt[4 + BLOCK_SAMPLES * 4];
  uint16_t per_pkt = (peerMtu() - 3 - 4) / 4;
  if (per_pkt > 32) per_pkt = 32;
  if (per_pkt > blk.n) per_pkt = blk.n;
  for (uint16_t off = 0; off < blk.n; off += per_pkt) {
    const uint16_t n = (blk.n - off) < per_pkt ? (blk.n - off) : per_pkt;
    const uint32_t idx = blk.first_index + off;
    memcpy(pkt, &idx, 4);
    memcpy(pkt + 4, &blk.raw[off], n * 2);
    memcpy(pkt + 4 + n * 2, &blk.filt[off], n * 2);
    if (!notifyAudio(g_audio_rf, pkt, 4 + n * 4)) ++app::counters().ble_drops;
  }
}

// Filtered only (legacy): [u32 index][int16 filt[n]].
void sendFiltered(const BleBlock& blk) {
  static uint8_t pkt[4 + BLOCK_SAMPLES * 2];
  uint16_t per_pkt = (peerMtu() - 3 - 4) / 2;
  if (per_pkt > blk.n) per_pkt = blk.n;
  for (uint16_t off = 0; off < blk.n; off += per_pkt) {
    const uint16_t n = (blk.n - off) < per_pkt ? (blk.n - off) : per_pkt;
    const uint32_t idx = blk.first_index + off;
    memcpy(pkt, &idx, 4);
    memcpy(pkt + 4, &blk.filt[off], n * 2);
    if (!notifyAudio(g_audio, pkt, 4 + n * 2)) ++app::counters().ble_drops;
  }
}

void bleTask(void*) {
  static BleBlock blk;
  static BufferPrint<1536> reply;
  Cmd cmd;
  for (;;) {
    while (xQueueReceive(g_cmd_q, &cmd, 0) == pdTRUE) {
      reply.clear();
      commands::execute(cmd.text, reply);
      notifyStatus(reply.c_str(), reply.length());
    }
    if (xQueueReceive(g_audio_q, &blk, pdMS_TO_TICKS(20)) != pdTRUE) continue;
    if (!app::bleConnected()) continue;

    if (rfSubscribed()) {
      sendRawFiltered(blk);
    } else {
      sendFiltered(blk);
    }
  }
}

}  // namespace

bool begin() {
  g_audio_q = xQueueCreate(BLE_QUEUE_LEN, sizeof(BleBlock));
  g_cmd_q = xQueueCreate(4, sizeof(Cmd));
  if (!g_audio_q || !g_cmd_q) return false;

  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(247);
  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());

  BLEService* svc = g_server->createService(BLE_SERVICE_UUID);  // 12 of the default 15 handles
  AudioCallbacks* audio_cb = new AudioCallbacks();
  g_audio = svc->createCharacteristic(BLE_AUDIO_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  g_audio_rf = svc->createCharacteristic(BLE_AUDIO_RF_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  g_status = svc->createCharacteristic(BLE_STATUS_UUID, BLECharacteristic::PROPERTY_NOTIFY |
                                                            BLECharacteristic::PROPERTY_READ);
  BLECharacteristic* ctrl = svc->createCharacteristic(
      BLE_CONTROL_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
#if defined(CONFIG_BLUEDROID_ENABLED)
  g_audio->addDescriptor(new BLE2902());
  g_audio_rf_cccd = new BLE2902();
  g_audio_rf->addDescriptor(g_audio_rf_cccd);
  g_status->addDescriptor(new BLE2902());
#endif
  g_audio->setCallbacks(audio_cb);
  g_audio_rf->setCallbacks(audio_cb);
  ctrl->setCallbacks(new ControlCallbacks());
  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  return xTaskCreatePinnedToCore(bleTask, "ble_tx", BLE_TASK_STACK, nullptr, BLE_TASK_PRIORITY,
                                 nullptr, BLE_TASK_CORE) == pdPASS;
}

void pushAudio(uint32_t first_index, const int16_t* raw, const int16_t* filt, uint16_t n) {
  if (!g_audio_q) return;
  BleBlock b;
  b.first_index = first_index;
  b.n = n > BLOCK_SAMPLES ? BLOCK_SAMPLES : n;
  memcpy(b.raw, raw, b.n * 2);
  memcpy(b.filt, filt, b.n * 2);
  if (xQueueSend(g_audio_q, &b, 0) != pdTRUE) ++app::counters().ble_drops;
}

void publishStatus(const char* text, size_t len) {
  if (!g_status) return;
  g_status->setValue(reinterpret_cast<uint8_t*>(const_cast<char*>(text)), len);
  notifyStatus(text, len);
}

}  // namespace ble_link
