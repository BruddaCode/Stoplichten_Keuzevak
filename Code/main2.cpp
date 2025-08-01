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

// RGB ring amount of LEDs
#define NUM_LEDS 16

// button pins
#define button 33
#define buttonLed 25

// pins for the bike lights
#define greenb 14
#define orangeb 32
#define redb 13

// pins for the pedestrian lights
#define greenp 26
#define redp 27

// variables to keep track of initialization of all the cases
bool case1Initialized = false;
bool case2Initialized = false;
bool case3Initialized = false;
bool case4Initialized = false;
bool case5Initialized = false;

// variable for the timer
unsigned long startTime = 0;

// variable for the switch case
int state = 1;

// variables to keep track of the button state
bool currentState;
bool previousState = 1;

// MAC address off the other esp32
uint8_t peerAddress[] = {0x68, 0x25, 0xdd, 0xee, 0xd2, 0x84}; // normal (master)

// making new datatypes for the messages
typedef struct struct_outgoing {
  bool buttonState;
} struct_outgoing;

typedef struct struct_incoming {
  bool buttonState;
  int state;
} struct_incoming;

// variables for messaging the other esp32
struct_incoming incomingData;
struct_outgoing outgoingData = {0};

// arrays for the ledrings each 1 array
CRGB ledsgb[NUM_LEDS];
CRGB ledsob[NUM_LEDS];
CRGB ledsrb[NUM_LEDS];
CRGB ledsgp[NUM_LEDS];
CRGB ledsrp[NUM_LEDS];

// makes a datatype to mix different colours for the ledrings
struct RGB_struct{
  int r;
  int g;
  int b;
};

// the different colours for the ledrings
RGB_struct red = {255,0,0};
RGB_struct orange = {255,40,0}; 
RGB_struct green = {0,255,0};
RGB_struct blank = {0,0,0};

/*
The checkButton() function checks if the pedestrian button is pressed

If the pedestrian button has been pressed and the bikes currently have a green light
the function will turn the button LED on and message the master about the button press
*/
void checkButton(){
  currentState = digitalRead(button);
  if(previousState != currentState && state == 1){
    previousState = currentState;
    digitalWrite(buttonLed, HIGH);
    esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
  }
}

/*
The function OnDataSent() is called after the sendData function is done
All this does is check if the package has been delivered succesfully and report this to the terminal
*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/*
The OnDataRecv() function will be called whenever the other esp32 sends us data

First it checks if the message is the size it expects
When it is it will store the data in incomingData

Then it will change its state to the one it just recieved

Lastly it checks if the buttonState is low and if it is it turns on the button LED
*/
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingDataBytes, int len) {
  if (len == sizeof(struct_incoming)) {
    memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));
    state = incomingData.state;
    if (incomingData.buttonState == 0) {
      digitalWrite(buttonLed, HIGH);
    }
  }
}

/*
The changeLeds() function takes a RGB_struct and an int
it uses the int to decide which ring to change colour of

After getting to the right ring it changes every datalocation in the array to the right rgb mix.
When it is done with that the function calls the show() function which makes all the LEDs switch to the colour in their array spot
*/
void changeLeds(RGB_struct color, int ring) {
  switch(ring){
    case 1:
      for (int i = 0; i < NUM_LEDS; i++) {
        ledsgb[i] = CRGB(color.r, color.g, color.b);
      }
      FastLED.show();
      break;

    case 2:
      for (int i = 0; i < NUM_LEDS; i++) {
        ledsob[i] = CRGB(color.r, color.g, color.b);
      }
      FastLED.show();
      break;

    case 3:
      for (int i = 0; i < NUM_LEDS; i++) {
        ledsrb[i] = CRGB(color.r, color.g, color.b);
      }
      FastLED.show();
      break;

    case 4:
      for (int i = 0; i < NUM_LEDS; i++) {
        ledsgp[i] = CRGB(color.r, color.g, color.b);
      }
      FastLED.show();
      break;

    case 5:
      for (int i = 0; i < NUM_LEDS; i++) {
        ledsrp[i] = CRGB(color.r, color.g, color.b);
      }
      FastLED.show();
      break;
  }
}

/*
The setup() is where the code starts running

In the setup all the initializations take place
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Initialization for Serial talk on the Baudrate 115200
  WiFi.mode(WIFI_STA);  // Initialization of the WiFi mode

  // Initializing the pins of the button and its LED
  pinMode(button, INPUT_PULLUP);
  pinMode(buttonLed, OUTPUT);
  
  // Adding all of the ledrings that are used
  FastLED.addLeds<WS2812, redb, GRB>(ledsrb, NUM_LEDS);
  FastLED.addLeds<WS2812, orangeb, GRB>(ledsob, NUM_LEDS);
  FastLED.addLeds<WS2812, greenb, GRB>(ledsgb, NUM_LEDS);
  FastLED.addLeds<WS2812, redp, GRB>(ledsrp, NUM_LEDS);
  FastLED.addLeds<WS2812, greenp, GRB>(ledsgp, NUM_LEDS);

  // Making sure all of the LEDs are off at startup
  FastLED.clear();
  FastLED.show();

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

/*
In the loop the code is run which decides how the trafficlight should act
*/
void loop() {
  checkButton();  // calling checkButton each loop to see if there are pedestrians who want to cross
  
  /*
  The switch case exists of the different states the trafficlight can be in

  1 bikes can go and pedestrians have a red light
  2 bikes get orange and need to stop when they can
  3 bikes get red and pedestrians can go
  4 starts the blinking fase which tells the pedestrians the light will turn red soon
  5 the trafficlight starts blinking the green pedestrian light
  */
  switch (state) {
    case 1:
      if(!case1Initialized){
        changeLeds(green, 1);
        changeLeds(red, 5);
        changeLeds(blank, 2);
        changeLeds(blank, 3);
        changeLeds(blank, 4); 
        case5Initialized = false;
        case1Initialized = true; 
      } 
      break;

    case 2:
      if(!case2Initialized){
        changeLeds(red, 5);
        changeLeds(orange, 2);
        changeLeds(blank, 1);
        changeLeds(blank, 3);
        changeLeds(blank, 4);
        case1Initialized = false;    
        case2Initialized = true;
      }
      break;

    case 3:
      if(!case3Initialized){
        changeLeds(red, 3);
        changeLeds(green, 4);
        digitalWrite(buttonLed, LOW);
        changeLeds(blank, 1);
        changeLeds(blank, 2);
        changeLeds(blank, 5); 
        case2Initialized = false;
        case3Initialized = true; 
      }    
      break;

    case 4:
      if(!case4Initialized){
        changeLeds(red, 3);
        changeLeds(blank, 1);
        changeLeds(blank, 2);
        changeLeds(blank, 4);
        changeLeds(blank, 5);
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
          changeLeds(green, 4);
        } else {
          changeLeds(blank, 4);
        }
      } else {
        changeLeds(blank, 4);
      }
      break;
  }
}