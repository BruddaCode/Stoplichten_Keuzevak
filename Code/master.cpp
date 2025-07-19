/*
This is the code for the esp32 which acts as the master
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
#define DEBUG 0

// RGB ring amount of LEDs
#define NUM_LEDS 16

// button pins
#define button 33
#define buttonLed 25

// pins for the bike lights
#define greenBike 14   // (3) on the trafficlight
#define orangeBike 32  // (2) on the trafficlight
#define redBike 13     // (1) on the trafficlight

// pins for the pedestrian lights
#define greenPedestrian 26   // (5) on the trafficlight
#define redPedestrian 27     // (4) on the trafficlight

//! look into not needing these
// variables to keep track of initialization of case 1 and 4
bool case1Initialized;
bool case4Initialized;

// variable for the timer
unsigned long startTime = 0;

// variable for the switch case
int state = 1;

// MAC address off the other esp32
static uint8_t peerAddress[] = {0xc8, 0xf0, 0x9e, 0xf3, 0xde, 0x24};

// making new datatypes for the messages
struct struct_incoming {
  bool buttonState;
};

struct struct_outgoing {
  bool buttonState;
  int state;
};

// variables for messaging the other esp32
// struct_incoming incomingData;
struct_outgoing outgoingData = {1, 1};

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

// variables to keep track of the button state
bool currentState;
bool previousState = 1;

// the different colours for the ledrings
RGB_struct red = {255,0,0};
RGB_struct orange = {255,40,0}; 
RGB_struct green = {0,255,0};
RGB_struct blank = {0,0,0};

/*
The sendData() function sends the data stored in outgoingData to the other esp32
After doing so the esp_now_send() calls the OnDataSent() function
*/
void sendData(){
  esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
}

/*
The checkButton() function checks if the pedestrian button is pressed

If the pedestrian button has been pressed and the bikes currently have a green light
the function will change the state of the trafficlight so that pedestrians can cross after waiting a few seconds

After changing it's own state it changes the buttonState for the outgoing data and sends this data to the other esp32

Then it changes the state in the outgoing data so this can be sent when the time is there

Lastly it writes the LED on the button high showcasing the pedestrians that there request has been taken
*/
void checkButton(){
  currentState = digitalRead(button);

  if(previousState != currentState && state == 1){
    previousState = currentState;
    state = 2;
    outgoingData.buttonState = 0;
    sendData();
    outgoingData.state = 2;
    digitalWrite(buttonLed, HIGH);
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
When it does so, it will always be to say that his button has been pressed

Upon being called this function will change the state of the trafficlight so that pedestrians can cross after waiting a few seconds

Then it changes the state in the outgoing data so this can be sent when the time is there

Lastly it writes the LED on the button high showcasing the pedestrians that there request has been taken
*/
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingDataBytes, int len) {
  state = 2;
  outgoingData.state = 2;
  digitalWrite(buttonLed, HIGH);  
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
  Serial.begin(115200); // Initialization for Serial talk on the Baudrate 115200
  WiFi.mode(WIFI_STA);  // Initialization of the WiFi mode

  // Adding all of the ledrings that are used
  FastLED.addLeds<WS2812, redBike, GRB>(ledsrb, NUM_LEDS);
  FastLED.addLeds<WS2812, orangeBike, GRB>(ledsob, NUM_LEDS);
  FastLED.addLeds<WS2812, greenBike, GRB>(ledsgb, NUM_LEDS);
  FastLED.addLeds<WS2812, redPedestrian, GRB>(ledsrp, NUM_LEDS);
  FastLED.addLeds<WS2812, greenPedestrian, GRB>(ledsgp, NUM_LEDS);

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
        startTime = millis(); // This resets the timer back to 0
        changeLeds(green, 1);
        changeLeds(red, 5);
        changeLeds(blank, 2);
        changeLeds(blank, 3);
        changeLeds(blank, 4);
        case1Initialized = true;
      }

      // Going to state 2 after 25 seconds
      if(millis() - startTime > 25000){
        startTime = millis(); // This resets the timer back to 0
        state = 2;
        outgoingData.state = 2;     
        case1Initialized = false;
      } 
      break;

    case 2:
      if(millis() - startTime > 5000){
        sendData(); // Telling the other esp32 it needs to enter state 2
        changeLeds(red, 5);
        changeLeds(orange, 2);
        changeLeds(blank, 1);
        changeLeds(blank, 3);
        changeLeds(blank, 4);

        // All of the changes to enter state 3
        state = 3;
        outgoingData.state = 3;
        outgoingData.buttonState = 1;
        startTime = millis(); // This resets the timer back to 0      
      }
      break;

    case 3:
      if(millis() - startTime > 2000){
        sendData(); // Telling the other esp32 to enter state 3
        changeLeds(red, 3);
        changeLeds(green, 4);
        digitalWrite(buttonLed, LOW);
        changeLeds(blank, 1);
        changeLeds(blank, 2);
        changeLeds(blank, 5);

        // All of the changes to enter state 4 
        state = 4;
        outgoingData.state = 4;
        startTime = millis(); // This resets the timer back to 0
      }
      break;

    case 4:
      if(millis() - startTime > 10000){
        if (!case4Initialized) {
          sendData(); // Telling the other esp32 to enter state 4
          case4Initialized = true;
        } 
        changeLeds(red, 3);
        changeLeds(blank, 1);
        changeLeds(blank, 2);
        changeLeds(blank, 4);
        changeLeds(blank, 5);
      }
      if(millis() - startTime > 10500){
        // All of the changes to enter state 5
        state = 5;
        outgoingData.state = 5;
        sendData(); // Telling the other esp32 to enter state 4
        case4Initialized = false;
        startTime = millis(); // This resets the timer back to 0
      }
      break;
    
    case 5:
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
        // All of the changes to enter state 1
        state = 1;
        outgoingData.state = 1;
        sendData(); // Telling the other esp32 to enter state 3
      }
      break;
  }
}