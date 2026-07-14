#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// Generate unique UUIDs for your custom service and characteristic
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

const char* commands[] = {"cmd1", "cmd2", "cmd3", "cmd4"};
int commandIndex = 0;
unsigned long lastTransmissionTime = 0;
const unsigned long interval = 10000; // 10 seconds in milliseconds

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      // Restart advertising so the Central can reconnect if dropped
      pServer->startAdvertising();
    }
};

void setup() {
  Serial.begin(115200);

  delay(4000);
  // Initialize BLE and set explicit device name
  BLEDevice::init("ESP32-CMD-SERVER");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic with Notify property
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Create a BLE Descriptor (required for Notifications)
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issues
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("ESP32 BLE Peripheral Active. Waiting for nRF53 Central...");
}

void loop() {
  if (deviceConnected) {
    unsigned long currentMillis = millis();
    
    // Check if 10 seconds have elapsed
    if (currentMillis - lastTransmissionTime >= interval) {
      lastTransmissionTime = currentMillis;
      
      // Select the current command
      String currentCmd = commands[commandIndex];
      
      // Update characteristic value and notify the connected central
      pCharacteristic->setValue(currentCmd.c_str());
      pCharacteristic->notify();
      
      Serial.print("Notified Central with: ");
      Serial.println(currentCmd);
      
      // Cycle through the 4 commands
      commandIndex = (commandIndex + 1) % 4;
    }
  }
}