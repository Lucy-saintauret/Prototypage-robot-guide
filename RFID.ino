#include <SPI.h>
#include <MFRC522.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>  // Needed for BLE notifications

// --- RFID pins ---
#define SS_PIN   32
#define RST_PIN  22

// SPI pins
#define SCK_PIN  25
#define MOSI_PIN 26
#define MISO_PIN 27

// BLE UUIDs
#define SERVICE_UUID        "9A48F510-0912-4A17-8A49-D76BE128A222"
#define CHARACTERISTIC_UUID "67F63520-6139-4C4E-9A49-8C1D94A31BBB"

MFRC522 rfid(SS_PIN, RST_PIN);
TFT_eSPI tft;

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
String lastUID = "";

// BLE Callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("📱 Phone connected!");

    // Enable notifications
    pCharacteristic->setNotifyProperty(true);
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("❌ Phone disconnected — waiting...");
    pServer->getAdvertising()->start();
  }
};

void setup() {
  Serial.begin(115200);

  // Init SPI + RFID
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  rfid.PCD_Init();

  // Init TFT
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN);
  tft.setCursor(10, 20);
  tft.println("RFID BLE READY");

  // BLE Init
  BLEDevice::init("ESP32_RFID");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("WAITING");

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  Serial.println("📡 BLE Advertising — ready!");
}

void sendUID(String uid) {
  if (!deviceConnected) {
    Serial.println("⚠ BLE not connected — UID not sent.");
    return;
  }

  if (uid == lastUID) return; // avoid spamming

  lastUID = uid;

  pCharacteristic->setValue(uid.c_str());
  pCharacteristic->notify();

  Serial.print("📡 SENT: ");
  Serial.println(uid);
}

void loop() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) return;

  // Build UID
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
    if (i < rfid.uid.size - 1) uid += ":";
  }
  uid.toUpperCase();

  Serial.print(" Detected RFID: ");
  Serial.println(uid);

  // Display on screen
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 40);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(3);
  tft.println(uid);

  // Send to BLE
  sendUID(uid);

  delay(2000);

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 20);
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(2);
  tft.println("Scan next card...");
  
  rfid.PICC_HaltA();
}