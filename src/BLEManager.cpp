#include "BLEManager.h"
#include "Config.h"
#include <esp_mac.h>

const uint8_t BLEManager::_hidReportMap[] = {
    0x05, 0x01,        
    0x09, 0x06,        
    0xA1, 0x01,        
    0x85, 0x01,        
    0x05, 0x07,        
    0x19, 0xE0,        
    0x29, 0xE7,        
    0x15, 0x00,        
    0x25, 0x01,        
    0x75, 0x01,        
    0x95, 0x08,        
    0x81, 0x02,        
    0x95, 0x01,        
    0x75, 0x08,        
    0x81, 0x01,        
    0x95, 0x05,        
    0x75, 0x01,        
    0x05, 0x08,        
    0x19, 0x01,        
    0x29, 0x05,        
    0x91, 0x02,        
    0x95, 0x01,        
    0x75, 0x03,        
    0x91, 0x01,        
    0x95, 0x06,        
    0x75, 0x08,        
    0x15, 0x00,        
    0x25, 0x65,        
    0x05, 0x07,        
    0x19, 0x00,        
    0x29, 0x65,        
    0x81, 0x00,        
    0xC0              
};

BLEManager::BLEManager() 
  : _pServer(nullptr)
  , _pHidService(nullptr)
  , _pInputReport(nullptr)
  , _connected(false)
  , _serverCallbacks(nullptr)
{
}

void BLEManager::begin(uint8_t slot, const char *deviceName) {
  setUniqueMac(slot);

  Serial.printf("[BLE] Initializing slot %d: '%s'\n", slot + 1, deviceName);

  NimBLEDevice::init(deviceName);
  
  _pServer = NimBLEDevice::createServer();
  _serverCallbacks = new ServerCallbacks(this);
  _pServer->setCallbacks(_serverCallbacks);

  _pHidService = _pServer->createService(HID_SERVICE_UUID);

  NimBLECharacteristic *pProtocolMode = _pHidService->createCharacteristic(
    HID_PROTOCOL_MODE_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  uint8_t protocolMode = 1;
  pProtocolMode->setValue(&protocolMode, 1);

  NimBLECharacteristic *pReportMap = _pHidService->createCharacteristic(
    HID_REPORT_MAP_UUID,
    NIMBLE_PROPERTY::READ
  );
  pReportMap->setValue((uint8_t*)_hidReportMap, sizeof(_hidReportMap));

  NimBLECharacteristic *pHidInfo = _pHidService->createCharacteristic(
    HID_HID_INFORMATION_UUID,
    NIMBLE_PROPERTY::READ
  );
  uint8_t hidInfo[] = {0x01, 0x01, 0x00, 0x02};
  pHidInfo->setValue(hidInfo, sizeof(hidInfo));

  NimBLECharacteristic *pControlPoint = _pHidService->createCharacteristic(
    HID_HID_CONTROL_POINT_UUID,
    NIMBLE_PROPERTY::WRITE_NR
  );
  uint8_t controlPoint = 0;
  pControlPoint->setValue(&controlPoint, 1);

  _pInputReport = _pHidService->createCharacteristic(
    HID_INPUT_REPORT_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  uint8_t reportRef[] = {0x01, 0x01};
  NimBLEDescriptor *pReportRefDesc = _pInputReport->createDescriptor(
    HID_REPORT_REFERENCE_UUID,
    NIMBLE_PROPERTY::READ,
    2
  );
  pReportRefDesc->setValue(reportRef, 2);

  _pHidService->start();
  _pServer->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(HID_SERVICE_UUID);
  pAdvertising->setName(deviceName);
  pAdvertising->setAppearance(0x03C1);
  pAdvertising->enableScanResponse(true);
  pAdvertising->setPreferredParams(0x06, 0x12);
  
  bool started = pAdvertising->start();
  Serial.printf("[BLE] Advertising as '%s' - %s\n", deviceName, started ? "STARTED" : "FAILED");
  
  if (!started) {
    Serial.println("[BLE] ERROR: Failed to start advertising!");
  } else {
    delay(100);
    Serial.println("[BLE] Device should now be discoverable. Look for it in Bluetooth settings.");
  }
}

bool BLEManager::isConnected() {
  return _connected && _pServer && _pServer->getConnectedCount() > 0;
}

void BLEManager::sendKeyboardReport(const uint8_t *keys, uint8_t modifiers) {
  if (!isConnected() || !_pInputReport)
    return;

  uint8_t report[8] = {0};
  report[0] = modifiers;
  memcpy(&report[2], keys, 6);

  _pInputReport->setValue(report, sizeof(report));
  _pInputReport->notify();
}

void BLEManager::setUniqueMac(uint8_t slot) {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  mac[5] = (mac[5] & 0xF0) | (slot & 0x0F);

  esp_err_t err = esp_base_mac_addr_set(mac);
  if (err != ESP_OK) {
    Serial.printf("[BLE] Failed to set MAC address: %s\n",
                  esp_err_to_name(err));
  } else {
    Serial.printf("[BLE] Set MAC address to: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
}

void BLEManager::ServerCallbacks::onConnect(NimBLEServer *pServer, NimBLEConnInfo& connInfo) {
  _manager->_connected = true;
  Serial.println("[BLE] Client connected");
}

void BLEManager::ServerCallbacks::onDisconnect(NimBLEServer *pServer, NimBLEConnInfo& connInfo, int reason) {
  _manager->_connected = false;
  Serial.printf("[BLE] Client disconnected, reason: %d\n", reason);
  NimBLEDevice::startAdvertising();
}
