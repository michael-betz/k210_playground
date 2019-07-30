// Horrible hack to place a custom firmware on the ESP8285 on the Sipeed M1 module
//
// I forward the ESP serial port pins through the gpio interface of the K210.
// I'm not using UART hardware but just sample the RX pin levels and set the
// TX pins accordingly in a very tight loop. Turns out the K210 is fast enough
// for this to work up to serveral MBaud/s.
//
// Unfortunately GPIO0 (pin 15) on the ESP is not connected on the M1 board. This
// pin needs to be pulled low to enter the ESP bootloader. Hence a botch wire
// is required between Pin 9 on the M1 board and Pin 15 on the ESP8285.
//
// to connect I need to open miniterm.py and make sure DTR is __active__
// by toggling it with `CTRL + T, CTRL + D`
// Then I can run succesfully:
// esptool.py --before no_reset --after soft_reset chip_id
//
// Note: ESP bootloader runs initially at 74880 baud/s

#include <Arduino.h>
#include "gpiohs.h"

#define PIN_LED_B 12
#define PIN_LED_G 13
#define PIN_LED_R 14

#define PIN_BTN_BOOT 16

#define ISP_RX 4
#define ISP_TX 5
#define ESP_RX 6
#define ESP_TX 7

#define PIN_ESP_EN 8
#define PIN_ESP_GPIO0 9

void setup()
{
    pinMode(ESP_RX, INPUT);       // 0
    pinMode(ISP_TX, OUTPUT);      // 1

    pinMode(ISP_RX, INPUT);       // 2
    pinMode(ESP_TX, OUTPUT);      // 3

    pinMode(PIN_BTN_BOOT, INPUT); // 4
    pinMode(PIN_LED_R, OUTPUT);   // 5

    pinMode(PIN_ESP_EN, OUTPUT);    // 6
    pinMode(PIN_ESP_GPIO0, OUTPUT); // 7

    volatile unsigned *gpio_o = gpiohs->output_val.u32;
    volatile unsigned *gpio_i = gpiohs->input_val.u32;

    // disable ESP, set gpio0 low for bootloader mode
    *gpio_o = 0;
    delay(10);

    // enable ESP
    *gpio_o = (1 << 6);

    while (1) {
      // forward RX and TX pins. ESP enable signal is controlled
      // by serial port RTS line in parallel with BOOT button on Dan Dock
      *gpio_o = ((*gpio_i) << 1) | ((*gpio_i & (1 << 4)) << 2);
    }
}

void loop () {}
