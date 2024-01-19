#include <Arduino.h>

// Only include debug for platformIO.
// #ifdef PLATFORMIO
// #include "avr8-stub.h"
// #include "app_api.h"
// #endif

void setup()
{

// // Only include debug for platformIO.
// #ifdef PLATFORMIO
//   debug_init();
// #endif

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial1.begin(9600);
}

// the loop function runs over and over again forever
void loop()
{
  // read from port 0, send to port 1:
  if (Serial.available())
  {
    int inByte = Serial.read();
    if (inByte == 100) {
      Serial.write(21);
      Serial1.println(String((uint8_t)inByte));
    }
  }
}
