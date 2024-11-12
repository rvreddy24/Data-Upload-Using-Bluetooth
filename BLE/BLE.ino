#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <DHT.h>

#define DHTPIN 14 // Pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11 (AM2302) sensor

DHT dht(DHTPIN, DHTTYPE);

// BLE UUIDs
#define ENVIRONMENTAL_SENSING_SERVICE_UUID "181A"
#define TEMPERATURE_CHARACTERISTIC_UUID "2A6E"
#define HUMIDITY_CHARACTERISTIC_UUID "2A6F"
#define BATTERY_SERVICE_UUID "180F"
#define BATTERY_LEVEL_CHARACTERISTIC_UUID "2A19"

BLECharacteristic *temperatureCharacteristic;
BLECharacteristic *humidityCharacteristic;
BLECharacteristic *batteryCharacteristic;

bool deviceConnected = false;
uint8_t batteryLevel = 100; // Simulate 100% battery level
unsigned long lastBatteryUpdate = 0;
unsigned long lastSensorUpdate = 0;

// Server callbacks to handle connection and disconnection
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      pServer->startAdvertising(); // Restart advertising
    }
};

void setup() {
  Serial.begin(115200);
  dht.begin();

  // Create the BLE Device
  BLEDevice::init("Team 7");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the Environmental Sensing Service
  BLEService *envService = pServer->createService(ENVIRONMENTAL_SENSING_SERVICE_UUID);

  // Create Temperature Characteristic
  temperatureCharacteristic = envService->createCharacteristic(
                                  TEMPERATURE_CHARACTERISTIC_UUID,
                                  BLECharacteristic::PROPERTY_NOTIFY
                              );

  // Create Humidity Characteristic
  humidityCharacteristic = envService->createCharacteristic(
                                HUMIDITY_CHARACTERISTIC_UUID,
                                BLECharacteristic::PROPERTY_NOTIFY
                            );

  // Start the Environmental Sensing Service
  envService->start();

  // Create the Battery Service
  BLEService *batteryService = pServer->createService(BATTERY_SERVICE_UUID);

  // Create Battery Level Characteristic
  batteryCharacteristic = batteryService->createCharacteristic(
                              BATTERY_LEVEL_CHARACTERISTIC_UUID,
                              BLECharacteristic::PROPERTY_NOTIFY
                          );

  // Start the Battery Service
  batteryService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");
}

void loop() {
  // Simulate temperature and humidity readings every 5 seconds
  if (millis() - lastSensorUpdate >= 5000) {
    lastSensorUpdate = millis();
    
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (!isnan(temperature) && !isnan(humidity)) {
      // Convert temperature and humidity to BLE-friendly format (x100)
      int16_t temperatureBLE = (int16_t)(temperature * 100);
      uint16_t humidityBLE = (uint16_t)(humidity * 100);

      // Update and notify temperature and humidity
      temperatureCharacteristic->setValue((uint8_t*)&temperatureBLE, sizeof(int16_t));
      humidityCharacteristic->setValue((uint8_t*)&humidityBLE, sizeof(uint16_t));

      if (deviceConnected) {
        temperatureCharacteristic->notify();
        humidityCharacteristic->notify();
        
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" °C, Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");
      }
    }
  }

  // Simulate battery drain every minute
  if (millis() - lastBatteryUpdate >= 60000) {
    lastBatteryUpdate = millis();

    if (batteryLevel > 0) {
      batteryLevel--;
      batteryCharacteristic->setValue(&batteryLevel, 1);
      if (deviceConnected) {
        batteryCharacteristic->notify();
        Serial.print("Battery level: ");
        Serial.print(batteryLevel);
        Serial.println(" %");
      }
    }
  }

  delay(100);
}
