#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <esp_camera.h>

// Define GPIO pin for the PIR motion sensor
#define PIR_PIN 13

// BLE UUIDs
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012" // Example UUID
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-210987654321" // Example UUID

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;

bool deviceConnected = false;
bool motionDetected = false;
unsigned long motionStartTime = 0;
const long motionDelay = 5000; // Delay in milliseconds

// Camera configuration for ESP32-CAM (AI-Thinker model)
camera_config_t camera_config = {
  .pin_pwdn = -1,
  .pin_reset = -1,
  .pin_xclk = 0,
  .pin_sccb_sda = 26,
  .pin_sccb_scl = 27,
  .pin_d7 = 35,
  .pin_d6 = 34,
  .pin_d5 = 39,
  .pin_d4 = 36,
  .pin_d3 = 21,
  .pin_d2 = 19,
  .pin_d1 = 18,
  .pin_d0 = 5,
  .pin_vsync = 25,
  .pin_href = 23,
  .pin_pclk = 22,
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG,
  .frame_size = FRAMESIZE_SVGA,
  .jpeg_quality = 12,
  .fb_count = 1
};

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* server) {
      deviceConnected = true;
    }

    void onDisconnect(BLEServer* server) {
      deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);
  
  // Initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Initialize BLE
  BLEDevice::init("ESP32-CAM");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEDevice::startAdvertising();
  Serial.println("BLE Server started");
}

void loop() {
  // Check for motion
  if (digitalRead(PIR_PIN) == HIGH) {
    if (!motionDetected) {
      motionDetected = true;
      motionStartTime = millis();
    } else if ((millis() - motionStartTime) > motionDelay) {
      // Motion detected for more than 5 seconds, capture and send image
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb && deviceConnected) {
        pCharacteristic->setValue(fb->buf, fb->len);
        pCharacteristic->notify();
        Serial.println("Image sent over BLE");
      }
      if (fb) {
        esp_camera_fb_return(fb);
      }
      motionDetected = false; // Reset motion detection
    }
  } else {
    motionDetected = false;
  }
  delay(100); // Small delay to avoid excessive polling
}
