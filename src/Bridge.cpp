#include "Bridge.h"
#include "NVSUtils.h"
#include <hid_usage_keyboard.h>

uint8_t Bridge::_currentSlot = 0;
BLEManager Bridge::_bleManager;
Preferences Bridge::_preferences;

void Bridge::begin() {
  // 1. Load saved slot
  _preferences.begin("usb-ble", true);
  _currentSlot = _preferences.getUChar("slot", 0);
  if (_currentSlot >= NUM_DEVICE_SLOTS)
    _currentSlot = 0;
  _preferences.end();

  Serial.printf("[Config] Starting on device slot %d\n", _currentSlot + 1);

  // 2. Load bonds for this slot
  NVSUtils::loadSlotBonds(_currentSlot);

  // 3. Init BLE
  const char *deviceNames[NUM_DEVICE_SLOTS] = {DEVICE_NAME_1, DEVICE_NAME_2,
                                               DEVICE_NAME_3};
  _bleManager.begin(_currentSlot, deviceNames[_currentSlot]);

  // 4. Init USB
  USBManager::setKeyboardCallback(onKeyboardReport);
  USBManager::begin();
}

void Bridge::loop() {
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 5000) {
    lastStatus = millis();
    bool connected = _bleManager.isConnected();
    Serial.printf("[Status] Slot %d | BLE: %s\n", _currentSlot + 1,
                  connected ? "CONNECTED" : "waiting for pairing...");
  }
}

void Bridge::switchToSlot(uint8_t slot) {
  if (slot >= NUM_DEVICE_SLOTS)
    return;

  if (slot == _currentSlot) {
    Serial.printf("[BLE] Already on slot %d\n", slot + 1);
    if (LED_FEEDBACK_PIN >= 0) {
      for (int i = 0; i <= slot; i++) {
        digitalWrite(LED_FEEDBACK_PIN, HIGH);
        delay(150);
        digitalWrite(LED_FEEDBACK_PIN, LOW);
        delay(150);
      }
    }
    return;
  }

  Serial.printf("[BLE] Switching from slot %d to slot %d\n", _currentSlot + 1,
                slot + 1);

  // 1. Save current BLE bonds
  NVSUtils::saveSlotBonds(_currentSlot);

  // 2. Save new slot index
  _preferences.begin("usb-ble", false);
  _preferences.putUChar("slot", slot);
  _preferences.end();

  // 3. LED feedback
  if (LED_FEEDBACK_PIN >= 0) {
    for (int i = 0; i <= slot; i++) {
      digitalWrite(LED_FEEDBACK_PIN, HIGH);
      delay(150);
      digitalWrite(LED_FEEDBACK_PIN, LOW);
      delay(150);
    }
  }

  // 4. Cleanup USB
  usb_host_device_free_all();

  Serial.println("[System] Restarting to apply new slot settings...");
  delay(500);
  ESP.restart();
}

void Bridge::onKeyboardReport(const uint8_t *data, size_t length) {
  if (length < sizeof(hid_keyboard_input_report_boot_t))
    return;

  hid_keyboard_input_report_boot_t *kb_report =
      (hid_keyboard_input_report_boot_t *)data;

  // Check for device switching combo
  if (checkDeviceSwitchCombo(kb_report->key, kb_report->modifier.val)) {
    return;
  }

  // Debug output
  Serial.printf("[KB] mod:0x%02X keys:[%02X %02X %02X %02X %02X %02X]\n",
                kb_report->modifier.val, kb_report->key[0], kb_report->key[1],
                kb_report->key[2], kb_report->key[3], kb_report->key[4],
                kb_report->key[5]);

  // Forward to BLE
  _bleManager.sendKeyboardReport(kb_report->key, kb_report->modifier.val);
}

bool Bridge::checkDeviceSwitchCombo(const uint8_t *keys, uint8_t modifiers) {
  if (!ENABLE_DEVICE_SWITCHING)
    return false;

  bool hasRightAlt = (modifiers & 0x40) != 0;
  uint8_t numberKey = 0;

  for (int i = 0; i < 6; i++) {
    if (keys[i] >= HID_KEY_1 && keys[i] <= HID_KEY_3) {
      numberKey = keys[i] - HID_KEY_1 + 1;
      break;
    }
  }

  if (hasRightAlt && numberKey > 0 && numberKey <= NUM_DEVICE_SLOTS) {
    Serial.printf("[Switch] Right Alt + %d detected\n", numberKey);
    switchToSlot(numberKey - 1);
    return true;
  }

  return false;
}
