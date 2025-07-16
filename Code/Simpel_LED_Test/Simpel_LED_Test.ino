#include <FastLED.h>
#define LED_PIN 13
#define NUM_LEDS 16

CRGB leds[NUM_LEDS];

struct RGB_struct{
  int r;
  int g;
  int b;
};

RGB_struct red = {255,0,0};
RGB_struct orange = {255,40,0}; 
RGB_struct green = {0,255,0};

void setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
}

void loop() {
  changeLeds(red);
  delay(1000);

  changeLeds(orange);
  delay(1000);

  changeLeds(green);
  delay(1000);
}

void changeLeds(RGB_struct color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(color.r, color.g, color.b);
    FastLED.show();
  }
}
