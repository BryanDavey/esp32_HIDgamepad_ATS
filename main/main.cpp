#include "Arduino.h"

extern "C" void app_main()
{
    initArduino();
    pinMode(4, OUTPUT);
    while (true) {
        digitalWrite(4, HIGH);
        delay(1000);
        digitalWrite(4, LOW);
        delay(1000);
    }
    // Do your own thing
}