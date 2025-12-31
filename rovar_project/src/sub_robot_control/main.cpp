#include <WiFi.h>
#include <esp_now.h>

// -------- RECEIVER MAC (from your output) --------
uint8_t receiverMac[] = {
  0x68, 0xFE, 0x71, 0x81, 0x88, 0x18
};

// -------- Joystick pins --------
// Left Joystick (ADC1 pins for analog, digital pin for button)
#define JOY_X   34    // ADC1_CH6
#define JOY_Y   35    // ADC1_CH7
#define JOY_SW  27    // Digital pin (moved from 32 to free ADC1 pin)

// Right Joystick (ADC1 pins for analog, digital pin for button)
#define JOY2_X   33   // ADC1_CH5
#define JOY2_Y   32   // ADC1_CH4 (moved from 25 which is ADC2)
#define JOY2_SW  14   // Digital pin (ADC2 pins don't work with WiFi)

// -------- Data structure --------
typedef struct {
  int leftX;
  int leftY;
  int rightX;
  int rightY;
  bool button;
} JoystickData;

JoystickData txData;
bool espNowReady = false;
volatile bool lastSendOk = false;

void onSend(const uint8_t* mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
}

void setup() {
  Serial.begin(115200);

  pinMode(JOY_SW, INPUT_PULLUP);
  pinMode(JOY2_SW, INPUT_PULLUP);

  // Required for ESP-NOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Register peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMac, 6);
  peer.channel = 0;      // auto
  peer.encrypt = false; // no encryption for now

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_send_cb(onSend);
  espNowReady = true;
  Serial.println("ESP-NOW Sender Ready");
}

void loop() {
  if (!espNowReady) {
    Serial.println("ESP-NOW not ready");
    delay(500);
    return;
  }

  // Read left joystick
  txData.leftX = analogRead(JOY_X);
  txData.leftY = analogRead(JOY_Y);
  
  // Read right joystick
  txData.rightX = analogRead(JOY2_X);
  txData.rightY = analogRead(JOY2_Y);
  
  // Read buttons (using OR - either button triggers)
  txData.button = !digitalRead(JOY_SW) || !digitalRead(JOY2_SW);

  esp_now_send(receiverMac, (uint8_t *)&txData, sizeof(txData));

  Serial.print("Sent -> Left X:");
  Serial.print(txData.leftX);
  Serial.print(" Left Y:");
  Serial.print(txData.leftY);
  Serial.print(" | Right X:");
  Serial.print(txData.rightX);
  Serial.print(" Right Y:");
  Serial.print(txData.rightY);
  Serial.print(" | BTN:");
  Serial.println(txData.button);
  Serial.print("ESP-NOW status: ");
  Serial.println(lastSendOk ? "connected" : "not connected");


  delay(500); // ~20 Hz update rate
}
