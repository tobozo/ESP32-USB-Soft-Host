#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include "NimBLEDevice.h"

#define HID_SERVICE_UUID "1812"
#define HID_INPUT_REPORT_UUID "2A4D"
#define HID_REPORT_MAP_UUID "2A4B"
#define HID_HID_INFORMATION_UUID "2A4A"
#define HID_HID_CONTROL_POINT_UUID "2A4C"
#define HID_PROTOCOL_MODE_UUID "2A4E"
#define HID_REPORT_REFERENCE_UUID "2908"

class BLEManager {
public:
  BLEManager();

  void begin(uint8_t slot, const char *deviceName);

  bool isConnected();

  void sendKeyboardReport(const uint8_t *keys, uint8_t modifiers);

private:
  NimBLEServer *_pServer;
  NimBLEService *_pHidService;
  NimBLECharacteristic *_pInputReport;
  bool _connected;

  void setUniqueMac(uint8_t slot);
  
  class ServerCallbacks : public NimBLEServerCallbacks {
  public:
    ServerCallbacks(BLEManager *manager) : _manager(manager) {}
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo& connInfo) override;
    void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo& connInfo, int reason) override;
  private:
    BLEManager *_manager;
  };
  
  ServerCallbacks *_serverCallbacks;
  
  static const uint8_t _hidReportMap[];
};

#endif
