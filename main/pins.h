#ifndef PINS_H
#define PINS_H

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_adc/adc_oneshot.h>

/*
 *
 * Definitions and configurations for all gpio pins used in this project
 * 
 * To add a new pin, #define the name and assign a GPIO_NUM, then go to pins.c and add it to the corresponding pins array (o_pins or i_pins)
 * NOTE: pins with unique configurations need to be initialized seperately (do not add to o_pins/i_pins array)
 *
 *
*/


/*
 * Output pins.
 */
#define LATCH_PIN   15
#define CLOCK_PIN   16
#define DATA_PIN    17
#define READ_PIN    18

#define OUT1        40
#define OUT2        39
#define OUT3        38
#define OUT4        37

/*
 * Input pins.
 */

// UART parameters
#define BUF_SIZE                  (2048)
#define UART_PORT_NUM             UART_NUM_0
#define UART_BAUD_RATE            115200
#define TX_PIN                    GPIO_NUM_43 // UART pins mapped to USB interface
#define RX_PIN                    GPIO_NUM_44 // UART pins mapped to USB interface

// PWM parameters
#define PWM_CHANNEL               LEDC_CHANNEL_0
#define PWM_RESOLUTION            LEDC_TIMER_8_BIT
#define PWM_OUTPUT_FREQUENCY      (3921) // To match Arduino MEGA PWM frequency. Actual hardware constraint/limitation is unknown.  

// Arrays for organizing output and input pins: defined in "pins.c"
extern const int8_t o_pins[];
extern const int8_t i_pins[];

// Ananlog-digital converters
extern adc_oneshot_unit_handle_t adc_unit1;

// PWM channel config objects: defined in "pins.c"
extern ledc_channel_config_t throttle_channel;
extern ledc_channel_config_t braking_channel;
extern ledc_channel_config_t steering_channel;

void init_gpio();
void configure_uart();
void configure_pwm();

#endif // PINS_H