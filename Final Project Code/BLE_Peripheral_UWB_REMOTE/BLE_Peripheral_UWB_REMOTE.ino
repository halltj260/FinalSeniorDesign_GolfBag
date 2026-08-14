#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>
#include <Arduino.h>

/* ================= BLE DEFINITIONS ================= */

#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-1234-1234-abcdefabcdef"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

/* ================= UWB DEFINITIONS ================= */

#define UWB_RX_PIN 16
#define UWB_TX_PIN 17

HardwareSerial UWB(2);

#define NUM_BASES 4
#define FRAME_LEN (3 + (NUM_BASES * 4))

byte buffer[FRAME_LEN];
int bufIndex = 0;

float uwbDistances[NUM_BASES] = { -1, -1, -1, -1 };

/* ================= BUTTON DEFINITIONS ================= */

#define AUTO_LED
#define MAN_LED
#define PWR_LED
#define BLE_LED


#define AUTO      27
#define MANUAL    14
#define FORWARD   26
#define BACKWARD  32
#define LEFT      33
#define RIGHT     25

enum Mode {
  MODE_MANUAL = 0,
  MODE_AUTO   = 1
};

Mode currentMode = MODE_MANUAL;

uint8_t buttonMask = 0;

/* ================= CONTROL PACKET STRUCT ================= */

struct ControlPacket {
  uint8_t header;              // 0xAA
  uint8_t mode;                // 0=manual, 1=auto
  uint8_t buttons;             // bitmask
  float distances[NUM_BASES];  // meters
};

/* ================= BLE CALLBACK ================= */

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    // Serial.print("Client connected. MTU: ");
    Serial.println(pServer->getPeerMTU(pServer->getConnId()));
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Client disconnected");
  }
};

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);
  delay(1000);

  /* ----- Buttons ----- */
  pinMode(AUTO, INPUT_PULLUP);
  pinMode(MANUAL, INPUT_PULLUP);
  pinMode(FORWARD, INPUT_PULLUP);
  pinMode(BACKWARD, INPUT_PULLUP);
  pinMode(LEFT, INPUT_PULLUP);
  pinMode(RIGHT, INPUT_PULLUP);

  /* ----- UWB UART ----- */
  UWB.begin(115200, SERIAL_8N1, UWB_RX_PIN, UWB_TX_PIN);

  /* ----- BLE ----- */
  BLEDevice::init("ESP32_Remote");
  BLEDevice::setMTU(247);  // request larger MTU (optional)

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("Remote advertising...");
}

/* ================= UWB FRAME DECODE ================= */

void decodeDistances(byte *data, int len) {
  if (len < FRAME_LEN) return;

  for (int i = 0; i < NUM_BASES; i++) {
    int offset = 3 + (i * 4);

    uint32_t distanceRaw =
      data[offset] |
      (data[offset+1] << 8) |
      (data[offset+2] << 16) |
      (data[offset+3] << 24);

    if (distanceRaw > 0) {
      uwbDistances[i] = distanceRaw / 1000.0;  // mm → meters
    } else {
      uwbDistances[i] = -1.0;
    }
  }
}

/* ================= SEND PACKET ================= */

void sendPacket() {
  if (!deviceConnected) return;

  ControlPacket packet;

  packet.header = 0xAA;
  packet.mode = currentMode;
  packet.buttons = buttonMask;

  for (int i = 0; i < NUM_BASES; i++) {
    packet.distances[i] = uwbDistances[i];
  }


Serial.print("TX: ");
uint8_t* raw = (uint8_t*)&packet;
for (int i = 0; i < sizeof(packet); i++) {
  if (raw[i] < 0x10) Serial.print("0");
  Serial.print(raw[i], HEX);
  Serial.print(" ");
}
Serial.println();
Serial.print("Mode: ");
Serial.print(packet.mode);
Serial.print("  Buttons: ");
Serial.print(packet.buttons, BIN);
Serial.print("  Distances: ");
for (int i = 0; i < NUM_BASES; i++) {
  Serial.print(packet.distances[i], 3);
  Serial.print(" ");
}
Serial.println();




  pCharacteristic->setValue((uint8_t*)&packet, sizeof(packet));
  pCharacteristic->notify();
}

/* ================= LOOP ================= */

void loop() {

  /* ----- READ UWB DATA ----- */
  while (UWB.available()) {
    byte b = UWB.read();

    if (bufIndex == 0 && b != 0xAA) continue;
    if (bufIndex == 1 && b != 0x25) { bufIndex = 0; continue; }
    if (bufIndex == 2 && b != 0x01) { bufIndex = 0; continue; }

    buffer[bufIndex++] = b;

    if (bufIndex >= FRAME_LEN) {
      decodeDistances(buffer, bufIndex);
      bufIndex = 0;
    }
  }

  /* ----- UPDATE MODE ----- */
  if (digitalRead(AUTO) == LOW) {
    currentMode = MODE_AUTO;
  }

  if (digitalRead(MANUAL) == LOW) {
    currentMode = MODE_MANUAL;
  }

  /* ----- UPDATE BUTTON MASK (ONLY ACTIVE IN MANUAL) ----- */
  buttonMask = 0;

  if (currentMode == MODE_MANUAL) {
    if (digitalRead(FORWARD)  == LOW) buttonMask |= (1 << 0);
    if (digitalRead(BACKWARD) == LOW) buttonMask |= (1 << 1);
    if (digitalRead(LEFT)     == LOW) buttonMask |= (1 << 2);
    if (digitalRead(RIGHT)    == LOW) buttonMask |= (1 << 3);
  }

  /* ----- SEND AT 20 Hz ----- */
  static unsigned long lastSend = 0;

  if (millis() - lastSend > 50) {   // 20Hz
    sendPacket();
    lastSend = millis();
  }
}