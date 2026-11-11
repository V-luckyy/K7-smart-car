// Build with Arduino-ESP32 3.3.0 (ESP-IDF 5.5), board: ESP32 Dev Module.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <atomic>
#include "board_config.h"
#include "wire_protocol.h"

struct Received {
  k7::FleetFrame frame;
  uint32_t received_ms;
};

constexpr uint8_t BROADCAST[6] = {255, 255, 255, 255, 255, 255};
constexpr uint32_t TX_PERIOD_MS = 100;
constexpr uint32_t MAX_QUEUE_AGE_MS = 200;
QueueHandle_t rx_queue;
QueueHandle_t tx_done_queue;
k7::StreamParser usb_parser;
k7::FleetFrame pending;
uint32_t pending_ms = 0, last_send_ms = 0, last_status_ms = 0;
uint32_t usb_rx = 0, radio_tx_done = 0, radio_tx_error = 0;
std::atomic<uint32_t> radio_rx{0}, rejected{0};
bool has_pending = false, in_flight = false;
uint8_t local_mac[6];

void require_ok(esp_err_t result, const char* operation) {
  if (result == ESP_OK) return;
  // Startup failures stay readable in a serial monitor; the K7 bridge resyncs
  // past boot text and reports missing binary status as a link fault.
  Serial.printf("FATAL %s: %s\n", operation, esp_err_to_name(result));
  while (true) delay(1000);
}

// Wi-Fi callbacks only hand off bounded data. Serial I/O lives in loop().
void on_receive(const esp_now_recv_info_t* info, const uint8_t* data, int length) {
  if (memcmp(info->src_addr, LEADER_MAC, 6) != 0 ||
      !k7::valid(data, static_cast<size_t>(length)) || data[3] != k7::FLEET) {
    ++rejected;
    return;
  }
  Received received;
  memcpy(&received.frame, data, sizeof(received.frame));
  if (received.frame.header.sender_id != LEADER_ID) {
    ++rejected;
    return;
  }
  received.received_ms = millis();
  xQueueOverwrite(rx_queue, &received);  // State is latest-value data.
  ++radio_rx;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void on_sent(const wifi_tx_info_t*, esp_now_send_status_t status) {
#else
void on_sent(const uint8_t*, esp_now_send_status_t status) {
#endif
  xQueueOverwrite(tx_done_queue, &status);
}

void publish_status() {
  k7::StatusFrame status{};
  status.header = k7::header(k7::STATUS, VEHICLE_ID);
  status.uptime_ms = millis();
  memcpy(status.mac, local_mac, 6);
  status.role = IS_LEADER ? 1 : 2;
  status.channel = RADIO_CHANNEL;
  status.usb_rx = usb_rx;
  status.radio_tx_done = radio_tx_done;
  status.radio_tx_error = radio_tx_error;
  status.radio_rx = radio_rx.load();
  status.rejected = rejected.load();
  k7::seal(&status);
  Serial.write(reinterpret_cast<const uint8_t*>(&status), sizeof(status));
}

void setup() {
  Serial.setRxBufferSize(512);
  Serial.begin(USB_BAUD);
  if (!WiFi.mode(WIFI_STA)) require_ok(ESP_FAIL, "WiFi.mode");
  require_ok(esp_wifi_set_ps(WIFI_PS_NONE), "disable power save");
  require_ok(esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE), "set channel");
  require_ok(esp_wifi_get_mac(WIFI_IF_STA, local_mac), "get STA MAC");
  rx_queue = xQueueCreate(1, sizeof(Received));
  tx_done_queue = xQueueCreate(1, sizeof(esp_now_send_status_t));
  if (!rx_queue || !tx_done_queue) require_ok(ESP_ERR_NO_MEM, "create queues");
  require_ok(esp_now_init(), "esp_now_init");
  if (IS_LEADER) {
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, BROADCAST, 6);
    peer.channel = RADIO_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    require_ok(esp_now_add_peer(&peer), "add broadcast peer");
    require_ok(esp_now_register_send_cb(on_sent), "register send callback");
  } else {
    require_ok(esp_now_register_recv_cb(on_receive), "register receive callback");
  }
  publish_status();
  last_status_ms = millis();
}

void loop() {
  const uint32_t now = millis();
  if (IS_LEADER) {
    // Consume a bounded batch so corrupt/continuous USB input cannot starve radio.
    for (int i = 0; i < 512 && Serial.available(); ++i) {
      k7::FleetFrame frame;
      if (!usb_parser.push(static_cast<uint8_t>(Serial.read()), frame)) continue;
      if (frame.header.kind != k7::FLEET || frame.header.sender_id != LEADER_ID) {
        ++rejected;
        continue;
      }
      pending = frame;
      pending_ms = now;
      has_pending = true;
      ++usb_rx;
    }
    esp_now_send_status_t result;
    if (xQueueReceive(tx_done_queue, &result, 0) == pdTRUE) {
      in_flight = false;
      if (result == ESP_NOW_SEND_SUCCESS) ++radio_tx_done;
      else ++radio_tx_error;
    }
    if (has_pending && uint32_t(now - pending_ms) > MAX_QUEUE_AGE_MS)
      has_pending = false;
    if (has_pending && !in_flight && uint32_t(now - last_send_ms) >= TX_PERIOD_MS) {
      in_flight = true;
      const esp_err_t error = esp_now_send(BROADCAST,
          reinterpret_cast<const uint8_t*>(&pending), sizeof(pending));
      if (error != ESP_OK) {
        in_flight = false;
        ++radio_tx_error;
      }
      has_pending = false;  // No retransmission/repetition of a cached state.
      last_send_ms = now;
    }
  } else {
    Received received;
    if (xQueueReceive(rx_queue, &received, 0) == pdTRUE &&
        uint32_t(millis() - received.received_ms) <= MAX_QUEUE_AGE_MS) {
      Serial.write(reinterpret_cast<const uint8_t*>(&received.frame), sizeof(received.frame));
    }
  }
  if (uint32_t(now - last_status_ms) >= 1000) {
    publish_status();  // Local USB diagnostics only, never sent over the radio.
    last_status_ms = now;
  }
  delay(1);
}
