/*
This is the code for the esp32 which acts as the MASTER 
This code decides when the lights change and instructs the other esp32

Written by:
Corné Noorlander

Contribution of:
Fabio Wolthuis
Merel van der Leeden
Julian Bouman
*/

#include <FastLED.h>
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// 1 for on, 0 for off
#define DEBUG 1

// button pins
#define button 33
#define buttonLed 25

// led ring pins
#define greenBike 14         // (3) on the trafficlight
#define orangeBike 32        // (2) on the trafficlight
#define redBike 13           // (1) on the trafficlight
#define greenPedestrian 26   // (5) on the trafficlight
#define redPedestrian 27     // (4) on the trafficlight

// RGB ring amount of LEDs
#define NUM_LEDS 16

// MAC address of the Slave ESP32
static uint8_t peerAddress[] = {0xc8, 0xf0, 0x9e, 0xf3, 0xde, 0x24};

// struct for the outgoing data
// state is also used as global variable to keep track of the traffic light state
struct struct_outgoing {
  bool buttonState;
  int state;
};

// struct for the incoming data
struct struct_incoming {
  bool buttonState;
};

// struct to store RGB values
struct RGB_struct{
  int r;
  int g;
  int b;
};

// variables for messaging the other esp32
struct_incoming incomingData;
struct_outgoing outgoingData = {1, 1};

// setup the individual LED rings
CRGB ledRings[5][NUM_LEDS];
enum LedRingIndex {
  RED_BIKE,
  ORANGE_BIKE,
  GREEN_BIKE,
  RED_PEDESTRIAN,
  GREEN_PEDESTRIAN
};

// the different colours for the ledrings
RGB_struct red = {255,0,0};
RGB_struct orange = {255,40,0}; 
RGB_struct green = {0,255,0};
RGB_struct off = {0,0,0};

// variables to keep track of the button state
bool currentState;
bool previousState = 1;

// variables to keep track of initialization of case 1 and 4
bool case1Initialized;
bool case4Initialized;

// variables for the timer and timings
unsigned long startTime = 0;
unsigned long case1Duration = 25000;    // 25 seconds
unsigned long case2Duration = 5000;     // 5 seconds
unsigned long case3Duration = 2000;     // 2 seconds
unsigned long case4Duration = 10000;    // 10 seconds
unsigned long case5Duration = 10500;    // 10.5 seconds
unsigned long blinkingDuration = 3000; // 3 seconds for blinking
unsigned long blinkingInterval = 500; // 0.5 seconds for each blink
unsigned long startDebugTime = 0;
unsigned long debugInterval = 1000; // print every second in the loop

// function to send button state and traffic light state to the other esp32
void sendData(){
  esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
}

// function to be called after sending data that prints the status of the last sent packet
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (DEBUG) {
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }
}

// function to be called when data is received from the other esp32
// changes the state of the trafficlights and the button LED
// using ArduinoIDE: (const uint8_t *mac_addr, const uint8_t *incomingDataBytes, int len)
// using PlatformIO: (const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataBytes, int len)
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataBytes, int len) {
  outgoingData.state = 2;
  digitalWrite(buttonLed, HIGH);  
  if (DEBUG) {
    Serial.println("Data received from slave");
  }
}

// checks if the button is pressed
// if it is pressed the trafficlight state changes and the button led is turned on 
// also sends the data to the other esp32
void checkButton(){
  currentState = digitalRead(button);

  if(previousState != currentState && outgoingData.state == 1){
    previousState = currentState;
    outgoingData.buttonState = 0;
    outgoingData.state = 2;
    sendData();
    digitalWrite(buttonLed, HIGH);
  }
}

// function to change the color of the LEDs
void changeLeds(RGB_struct colors[]) {
  for (int ringIndex = 0; ringIndex < 5; ringIndex++) {
    CRGB *leds = ledRings[ringIndex];
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(colors[ringIndex].r, colors[ringIndex].g, colors[ringIndex].b);
    }
    FastLED.show();
    if (DEBUG) {
      Serial.println("-------------------");
      Serial.print("LED Ring ");
      Serial.print(ringIndex);
      Serial.print(": ");
      Serial.print(colors[ringIndex].r);
      Serial.print(", ");
      Serial.print(colors[ringIndex].g);
      Serial.print(", ");
      Serial.println(colors[ringIndex].b);
      Serial.println("-------------------");
    }
  }
}

void setup() {
  Serial.begin(115200); // Initialization for Serial talk on the Baudrate 115200
  WiFi.mode(WIFI_STA);  // Initialization of the WiFi mode

  // Adding all of the ledrings
  FastLED.addLeds<WS2812, redBike, GRB>(ledRings[RED_BIKE], NUM_LEDS);
  FastLED.addLeds<WS2812, orangeBike, GRB>(ledRings[ORANGE_BIKE], NUM_LEDS);
  FastLED.addLeds<WS2812, greenBike, GRB>(ledRings[GREEN_BIKE], NUM_LEDS);
  FastLED.addLeds<WS2812, redPedestrian, GRB>(ledRings[RED_PEDESTRIAN], NUM_LEDS);
  FastLED.addLeds<WS2812, greenPedestrian, GRB>(ledRings[GREEN_PEDESTRIAN], NUM_LEDS);

  // reset the LEDs on startup
  FastLED.clear();
  FastLED.show();

  // Initializing the pins of the button and its LED
  pinMode(button, INPUT_PULLUP);
  pinMode(buttonLed, OUTPUT);

  // Making sure the LED on the button is off on startup
  digitalWrite(buttonLed, LOW);

  // Initialization proces for the WiFi
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // initializing sending and receiving functions
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Gathering the info of the other esp32 in a single variable
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Making a connection with the other esp32
  if (!esp_now_is_peer_exist(peerAddress)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }
}

void loop() {
  checkButton();
  
  if (DEBUG) {
    if (millis() - startDebugTime > debugInterval) {
      startDebugTime = millis();
      Serial.println("-------------------");
      Serial.print("Button state: ");
      Serial.println(outgoingData.buttonState);
      Serial.print("Current state: ");
      Serial.println(outgoingData.state);
      Serial.println("-------------------");
    }
  }
  /*
  The switch case exists of the different states the trafficlight can be in

  1 bikes can go and pedestrians have a red light
  2 bikes get orange and need to stop when they can
  3 bikes get red and pedestrians can go
  4 starts the blinking fase which tells the pedestrians the light will turn red soon
  5 the trafficlight starts blinking the green pedestrian light
  */
  switch (outgoingData.state) {
    case 1:
      if(!case1Initialized){
        startTime = millis(); // This resets the timer back to 0
        RGB_struct colors[5] = {red, off, off, off, green}; // Set the colors for the LED rings
        changeLeds(colors);
        case1Initialized = true;
      }

      // enter state 2 after 25 seconds
      if(millis() - startTime > case1Duration){
        startTime = millis();
        outgoingData.state = 2;     
        case1Initialized = false;
      } 
      break;

    case 2:
      if(millis() - startTime > case2Duration){
        sendData(); // Telling the slave to change state
        RGB_struct colors[5] = {off, orange, off, off, red};
        changeLeds(colors);

        outgoingData.state = 3;
        outgoingData.buttonState = 1;
        startTime = millis();
      }
      break;

    case 3:
      if(millis() - startTime > case3Duration){
        sendData();
        RGB_struct colors[5] = {off, off, red, green, off};
        changeLeds(colors);
        digitalWrite(buttonLed, LOW);

        outgoingData.state = 4;
        startTime = millis();
      }
      break;

    case 4:
      if(millis() - startTime > case4Duration){
        if (!case4Initialized) {
          sendData();
          case4Initialized = true;
        } 
        RGB_struct colors[5] = {off, off, red, off, off};
        changeLeds(colors);
      }
      if(millis() - startTime > case5Duration){
        outgoingData.state = 5;
        sendData();
        case4Initialized = false;
        startTime = millis();
      }
      break;
    
    case 5:
      unsigned long elapsed = millis() - startTime;
      // for the next 3 seconds blink the green pedestrian light
      if (elapsed < blinkingDuration) {
        if ((elapsed / blinkingInterval) % 2 == 0) {
          RGB_struct colors[5] = {off, off, red, green, off};
          changeLeds(colors);
        } else {
          RGB_struct colors[5] = {off, off, red, off, off};
          changeLeds(colors);
        }
      } else {
        RGB_struct colors[5] = {off, off, red, off, off};
        changeLeds(colors);
        outgoingData.state = 1; // Reset to state 1 after blinking
        sendData();
      }
      break;
  }
}