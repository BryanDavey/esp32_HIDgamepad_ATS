#ifndef PINS_H
#include "pins.h"
#endif // PINS_H

// Extern variables from pins.h that need to be defined in a .c file to stop multiple definition errors
const int8_t o_pins[] = {
  LATCH_PIN,
  CLOCK_PIN,
  DATA_PIN,
  OUT1,
  OUT2,
  OUT3,
  OUT4,
};

const int8_t i_pins[] = {
  READ_PIN,
};

adc_oneshot_unit_handle_t adc_unit1;

// PWM config
// ledc_channel_config_t throttle_channel = {
//   .gpio_num = THROTTLE_PWM_PIN,
//   .speed_mode = LEDC_LOW_SPEED_MODE,
//   .channel = LEDC_CHANNEL_0,
//   .intr_type = LEDC_INTR_DISABLE,
//   .timer_sel = LEDC_TIMER_1,
//   .duty = 0,
// };


void init_gpio() {
  // Initialize generic output pins
  gpio_config_t o_conf, i_conf;
  o_conf.intr_type =      GPIO_INTR_DISABLE;
  o_conf.mode =           GPIO_MODE_OUTPUT;
  o_conf.pin_bit_mask =   0;
  o_conf.pull_down_en =   GPIO_PULLDOWN_ENABLE;
  o_conf.pull_up_en =     GPIO_PULLUP_DISABLE;
  for (int i = 0;i < sizeof(o_pins) / sizeof(o_pins[0]);i++) {
    if (o_pins[i] == GPIO_NUM_NC) continue; // Ignore any pins that are not connected.
    o_conf.pin_bit_mask |= (1ULL << o_pins[i]);
  }
  ESP_ERROR_CHECK(gpio_config(&o_conf));

  // Initialize generic input pins
  i_conf.intr_type =      GPIO_INTR_DISABLE;
  i_conf.mode =           GPIO_MODE_OUTPUT;
  i_conf.pin_bit_mask =   0;
  i_conf.pull_down_en =   GPIO_PULLDOWN_DISABLE;
  i_conf.pull_up_en =     GPIO_PULLUP_DISABLE;
  for (int i = 0;i < sizeof(i_pins) / sizeof(i_pins[0]);i++) {
    if (i_pins[i] == GPIO_NUM_NC) continue;
    i_conf.pin_bit_mask |= (1ULL << i_pins[i]);
  }
  i_conf.mode = GPIO_MODE_INPUT;
  if (sizeof(i_pins) > 0) ESP_ERROR_CHECK(gpio_config(&i_conf));
}

void configure_uart() {
  uart_config_t uart_config;
  /* Configure parameters of an UART driver,
   * communication pins and install the driver */
  uart_config.baud_rate = UART_BAUD_RATE;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.rx_flow_ctrl_thresh = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_DEFAULT;

  ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TX_PIN, RX_PIN, GPIO_NUM_NC, GPIO_NUM_NC));
  uart_flush(UART_PORT_NUM);
}

void configure_pwm() {
  // Initialize PWM channels
  ledc_timer_config_t ledc_timer = {
  .speed_mode = LEDC_LOW_SPEED_MODE,
  .duty_resolution = PWM_RESOLUTION,
  .timer_num = LEDC_TIMER_1,
  .freq_hz = PWM_OUTPUT_FREQUENCY
  };

  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
  ESP_ERROR_CHECK(ledc_channel_config(&throttle_channel));
  ESP_ERROR_CHECK(ledc_channel_config(&braking_channel));
  ESP_ERROR_CHECK(ledc_channel_config(&steering_channel));
}