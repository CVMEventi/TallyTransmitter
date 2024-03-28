#ifndef CONSTANTS_H
#define CONSTANTS_H
/* Radio Setup */

#define RF69_FREQ_1 865.0
#define RF69_FREQ_2 866.2
#define RFM69_CS 8
#define RFM69_INT 7
#define RFM69_RST 4

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#define OLED_CLOCK 3
#define OLED_DATA 2
#define OLED_RESET 4         // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define SERVER_ADDRESS 120

#define TALLY_PINS {A0, 0, 1, A1, A2, A3, A4, A5}
#define SERIAL_PINS {0, 1}

/* DIP switches configuration */
#define SERIAL_DIP_PIN 13
#define PREVIEW_DIP_PIN 12
#define CHANNEL_DIP_PIN 11

#define TALLY_ON 1
#define TALLY_OFF 0
#define TALLY_TYPE_PROGRAM 0
#define TALLY_TYPE_PREVIEW 1

#define COMM_MODE_SERIAL 0
#define COMM_MODE_IO_PREVIEW_PROGRAM 1
#define COMM_MODE_IO_PROGRAM 2

#endif