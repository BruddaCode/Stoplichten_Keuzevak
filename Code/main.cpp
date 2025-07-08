#include <FastLED.h>
#include <Arduino.h>

#define LED_PIN 17
#define NUM_LEDS 18
#define button 23

#include <esp_now.h>
#include <WiFi.h>

// Replace with the MAC address of the *other* device
uint8_t peerAddress[] = {0x10, 0x06, 0x1c, 0xb4, 0xf7, 0x40}; // bright board
// uint8_t peerAddress[] = {0x10, 0x06, 0x1c, 0xb5, 0xb1, 0x08}; // faint board

bool currentState;

typedef struct struct_message {
  bool buttonState;
} struct_message;

struct_message incomingData;
struct_message outgoingData = {currentState};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingDataBytes, int len) {
  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));
  Serial.print(incomingData.buttonState);
}


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  // Important: STA mode is required

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(peerAddress)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  pinMode(button, INPUT_PULLUP);
}

void loop() {
  currentState = digitalRead(button);
  outgoingData.buttonState = currentState;
  Serial.print(currentState);
  esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));

  delay(100);
}

CRGB leds[NUM_LEDS];

struct RGB_struct{
  int r;
  int g;
  int b;
};

RGB_struct red = {255,0,0};
RGB_struct orange = {255,40,0}; 
RGB_struct green = {0,255,0};

void changeLeds(RGB_struct color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(color.r, color.g, color.b);
    FastLED.show();
  }
}

// void setup() {
//   FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
// }

// void loop() {
//   changeLeds(red);
//   delay(1000);

//   changeLeds(orange);
//   delay(1000);

//   changeLeds(green);
//   delay(1000);
// }