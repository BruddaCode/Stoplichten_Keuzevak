/*
This is the code for the esp32 which acts as the slave
This code acts on the calls of the master
The only thing this code sends back to the master is when the pedestrian button has been pressed

Written by:
Corné Noorlander

Contribution of:
Fabio Wolthuis
Merel van der Leeden
Jullian Bouman
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

// MAC address of the Master ESP32
uint8_t peerAddress[] = {0x68, 0x25, 0xdd, 0xee, 0xd2, 0x84};

// struct for the outgoing data
// state is also used as global variable to keep track of the traffic light state
struct struct_outgoing {
  bool buttonState;
};

// struct for the incoming data
struct struct_incoming {
  bool buttonState;
  int state;
};

// struct to store RGB values
struct RGB_struct{
  int r;
  int g;
  int b;
};

// variables for messaging the other esp32
struct_incoming incomingData;
struct_outgoing outgoingData = {0};

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
bool case1Initialized = false;
bool case2Initialized = false;
bool case3Initialized = false;
bool case4Initialized = false;
bool case5Initialized = false;

// variable for the timer
unsigned long startTime = 0;
unsigned long startDebugTime = 0;
unsigned long debugInterval = 1000; // print every second in the loop

// function to be called after sending data that prints the status of the last sent packet
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// function to be called when data is received from the other esp32
// changes the state of the trafficlights and the button LED
// using ArduinoIDE: (const uint8_t *mac_addr, const uint8_t *incomingDataBytes, int len)
// using PlatformIO: (const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataBytes, int len)
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingDataBytes, int len) {
  if (len == sizeof(struct_incoming)) {
    memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));
    if (incomingData.buttonState == 0) {
      digitalWrite(buttonLed, HIGH);
    }
  }
}

// checks if the button is pressed
// if it is pressed the trafficlight state changes and the button led is turned on 
// also sends the data to the other esp32
void checkButton(){
  currentState = digitalRead(button);
  if(previousState != currentState && incomingData.state == 1){
    previousState = currentState;
    digitalWrite(buttonLed, HIGH);
    esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
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

  // Adding all of the ledrings that are used
  FastLED.addLeds<WS2812, redBike, GRB>(ledRings[RED_BIKE], NUM_LEDS);
  FastLED.addLeds<WS2812, orangeBike, GRB>(ledRings[ORANGE_BIKE], NUM_LEDS);
  FastLED.addLeds<WS2812, greenBike, GRB>(ledRings[GREEN_BIKE], NUM_LEDS);
  FastLED.addLeds<WS2812, redPedestrian, GRB>(ledRings[RED_PEDESTRIAN], NUM_LEDS);
  FastLED.addLeds<WS2812, greenPedestrian, GRB>(ledRings[GREEN_PEDESTRIAN], NUM_LEDS);
  
  // Making sure all of the LEDs are off at startup
  FastLED.clear();
  FastLED.show();

  // Initializing the pins of the button and its LED
  pinMode(button, INPUT_PULLUP);
  pinMode(buttonLed, OUTPUT);
  
  // Making sure the LED on the button is off after startup
  digitalWrite(buttonLed, LOW);

  // Initialization proces for the WiFi
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Making the functions which are called after sending and recieving data
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
      Serial.println(incomingData.buttonState);
      Serial.print("Current state: ");
      Serial.println(incomingData.state);
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
  switch (incomingData.state) {
    case 1:
      if(!case1Initialized){
        RGB_struct colors[5] = {red, off, off, off, green}; // Set the colors for the LED rings
        changeLeds(colors);
        case5Initialized = false;
        case1Initialized = true; 
      } 
      break;

    case 2:
      if(!case2Initialized){
        RGB_struct colors[5] = {red, off, off, off, green}; // Set the colors for the LED rings
        changeLeds(colors);
        case1Initialized = false;    
        case2Initialized = true;
      }
      break;

    case 3:
      if(!case3Initialized){
        RGB_struct colors[5] = {red, off, off, off, green}; // Set the colors for the LED rings
        changeLeds(colors);
        digitalWrite(buttonLed, LOW);
        case2Initialized = false;
        case3Initialized = true; 
      }    
      break;

    case 4:
      if(!case4Initialized){
        RGB_struct colors[5] = {red, off, off, off, green}; // Set the colors for the LED rings
        changeLeds(colors);
        case3Initialized = false;
        case4Initialized = true; 
      }
      break;
    
    case 5:
      if (!case5Initialized) {
        startTime = millis(); // This resets the timer back to 0
        case4Initialized = false;
        case5Initialized = true;
      }
      unsigned long elapsed = millis() - startTime;
      
      /*
      Makes the green pedestrian light blink 3 times
      
      The light turns off for 500 miliseconds and then on again for another 500 miliseconds
      */
      if (elapsed < 3000) {
        if ((elapsed / 500) % 2 == 0) {
          RGB_struct colors[5] = {red, off, off, off, green}; // Set the colors for the LED rings
        changeLeds(colors);
        } else {
          RGB_struct colors[5] = {red, off, off, off, green}; // Set the colors for the LED rings
        changeLeds(colors);
        }
      } else {
        RGB_struct colors[5] = {red, off, off, off, green}; // Set the colors for the LED rings
        changeLeds(colors);
      }
      break;
  }
}