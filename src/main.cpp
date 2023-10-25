#include <Arduino.h>

// Only include debug for platformIO.
#ifdef PLATFORMIO
#include "avr8-stub.h"
#include "app_api.h"
#endif

void setup()
{

// Only include debug for platformIO.
#ifdef PLATFORMIO
  debug_init();
#endif

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop()
{
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(1000);                     // wait for a second
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  delay(1000);                     // wait for a second
}
