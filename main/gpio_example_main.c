/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "pins.h"
#include "define.h"

#include "Arduino.h"


#include "rom/ets_sys.h"
#include "esp_hidd.h"

#include "USBHIDGamepad.h"


/*
 * --------------------------------------
 * Debug
 * --------------------------------------
 */

#define FEATURE_DISABLED          0
#define FEATURE_ENABLED           1

#define ENABLE_HATCH              FEATURE_DISABLED
#define ENABLE_REAR_POD_LIGHTS    FEATURE_DISABLED
#define ENABLE_STEERING           FEATURE_ENABLED

#define DEBUG_MODE                FEATURE_ENABLED

/// WARNING: Do not run primitive driver while DEBUG_MODE is enabled! 
//   DEBUG_MODE enables debug messages to be printed to UART (USB) so that they can be human readable in a serial monitor window.
//   DEBUG_MODE should only be used independently of the primitive driver and the rest of the ASV system software.

// Disable/enable log messages depending on DEBUG_MODE
#if DEBUG_MODE
#define error_logger(str) debug_serial_write_string("ERROR: ",str)
#define debug_logger(str) debug_serial_write_string("DEBUG: ",str)
#define trace_logger(str) debug_serial_write_string("TRACE: ",str)
#else
#define error_logger(str)
#define debug_logger(str)
#define trace_logger(str)
#endif

void serial_write_string(char* src) {
  uart_write_bytes(UART_PORT_NUM, (uint8_t*)src, strlen(src));
}
void debug_serial_write_string(char* log_level, char* src) {
  char log_buf[256];
  serial_write_string(strcat(strcat(strcpy(log_buf,log_level),src), "\n"));
}

/*
 * --------------------------------------
 * Variables
 * --------------------------------------
 */

bool pinStates[8];
uint8_t outPins[] = {0,OUT1,OUT2,OUT3,OUT4,0,0,0};

/*
 * --------------------------------------
 * Functions
 * --------------------------------------
 */

void delayBlocking(uint32_t us) {
    ets_delay_us(us);
}

void myShiftOut(uint8_t bit_order, uint8_t val) {
    for (uint8_t i=0;i<8;i++) {
        gpio_set_level(CLOCK_PIN, LOW);
        if (bit_order == LSBFIRST) {
            gpio_set_level(DATA_PIN, val & 1);
            val >>= 1;
        } else {
            gpio_set_level(DATA_PIN, (val & 128) != 0);
            val <<= 1;
        }
        gpio_set_level(CLOCK_PIN, HIGH);
        delayBlocking(10);
    }
}

/*
 * --------------------------------------
 * Threads
 * --------------------------------------
 */

void app_main() {
    initArduino();
    init_gpio();
    configure_uart();
    bool level = false;
    portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
    while(true) {
        taskENTER_CRITICAL(&myMutex);
        // for (int i=0;i<100;i++) {
        //     gpio_set_level(OUT1, level);
        //     level = !level;
        //     delayMicroseconds(3);
        // }
        for (int i=0;i<8;i++) {
            // Shift the bits out
            myShiftOut(MSBFIRST, 1 << i);
            gpio_set_level(LATCH_PIN, HIGH);
            delayBlocking(20);
            gpio_set_level(LATCH_PIN, LOW);
            delayBlocking(20);
            pinStates[i] = gpio_get_level(READ_PIN);
            if (outPins[i]) gpio_set_level(outPins[i],pinStates[i]);
        }
        taskEXIT_CRITICAL(&myMutex);
        char str[1024];
        sprintf(str, "PinStates: ");
        for (int i=0;i<sizeof(pinStates)/sizeof(pinStates[0]);i++) {
            sprintf(str + strlen(str), "%d,",pinStates[i]);
        }
        debug_logger(str);
        delayNonBlocking(100);
    }
}