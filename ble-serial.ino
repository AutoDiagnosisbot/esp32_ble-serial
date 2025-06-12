#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>

#define SERVICE_UUID           "93b1eb81-430d-4456-b97d-80ba7ac20fc4"
#define CHARACTERISTIC_SPEED_UUID "950ddf3e-050a-4cc3-b9bd-fea5c013b7e9"
#define CHARACTERISTIC_DATA_UUID "5a9b8290-0be7-4078-a503-c7a728cf2106"

// Вариант 1: GPIO13 (RX), GPIO14 (TX)
#define CUSTOM_RX_PIN 13
#define CUSTOM_TX_PIN 14

BLEServer* pServer = nullptr;
BLECharacteristic* pSpeedCharacteristic;
BLECharacteristic* pDataCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t currentSpeed = 115200;

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

class SpeedCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        uint8_t* data = pCharacteristic->getData();
        size_t length = pCharacteristic->getLength();
        if (length == 4) {
            memcpy(&currentSpeed, data, 4);
            Serial.end();
            Serial.begin(currentSpeed, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);
            //Serial.println("Speed changed to: " + String(currentSpeed));
            Serial1.println("Speed changed to: " + String(currentSpeed));
        }
    }
};

class DataCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        // Отправляем полученные данные в UART
        uint8_t* data = pCharacteristic->getData();
        size_t length = pCharacteristic->getLength();
        Serial.write(data, length);
    }
};


void setup() {
    //Serial.begin(115200);
    Serial.begin(115200, SERIAL_8N1, CUSTOM_RX_PIN, CUSTOM_TX_PIN);
    Serial1.begin(115200, SERIAL_8N1, 21, 22);
    BLEDevice::init("ESP32-BLE-Serial");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Характеристика для изменения скорости
    pSpeedCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_SPEED_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pSpeedCharacteristic->setCallbacks(new SpeedCallback());
    
    // Характеристика для данных UART
    pDataCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_DATA_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | 
        BLECharacteristic::PROPERTY_WRITE
    );
    pDataCharacteristic->addDescriptor(new BLE2902());
    pDataCharacteristic->setCallbacks(new DataCallback());

    pService->start();
    pServer->getAdvertising()->start();
}

void loop() {
    // Обработка подключения/отключения
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    // Отправка данных из Serial в BLE
    static String serialBuffer;
    while (Serial.available()) {
        char c = Serial.read();
        serialBuffer += c;
        
        if (c == '\n') {
            if (serialBuffer.length() > 0) {
                // Преобразуем String в C-строку
                pDataCharacteristic->setValue(serialBuffer.c_str());
                pDataCharacteristic->notify();
                serialBuffer = "";
            }
        }
    }
}