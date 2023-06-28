#include <Arduino.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <U8g2lib.h>
#include "constants.h"
#include "utils.h"
#include "tally-read.h"

/* Devices Setup */
//U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, OLED_CLOCK, OLED_DATA, OLED_RESET);
U8G2_SSD1309_128X64_NONAME0_2_HW_I2C u8g2(U8G2_R0, OLED_CLOCK, OLED_DATA, OLED_RESET);
RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rfManager(rf69, SERVER_ADDRESS);

uint8_t communicationMode = COMM_MODE_IO_PREVIEW_PROGRAM;
uint8_t tallyPins[] = TALLY_PINS;
float selectedFrequency = RF69_FREQ_1;
bool programStatuses[8] = {0,0,0,0,0,0,0,0};
bool previewStatuses[8] = {0,0,0,0,0,0,0,0};
int8_t batteries[8] = {-1,-1,-1,-1,-1,-1,-1,-1};
unsigned long savedTime = 0;
uint8_t timeouts[8] = {0,0,0,0,0,0,0,0};

bool statusChanged = false;

void sendTallyStatus(uint8_t to, uint8_t mode, uint8_t status) {
  uint8_t data[3] = {
          mode,
          status
  };

  Serial.print("Sending to tally n: ");
  Serial.print(to);
  Serial.print(" value: ");
  Serial.println(status);

  if (!rfManager.sendtoWait(data, sizeof(data), to)) {
    Serial.println("Send failed");
    batteries[to - 1] = 0;
  }

  if(!rfManager.waitPacketSent(200)) {
    Serial.println("Send failed: timeout");
  }
}

void readConfiguration() {
  pinMode(SERIAL_DIP_PIN, INPUT_PULLUP);
  pinMode(PREVIEW_DIP_PIN, INPUT_PULLUP);
  pinMode(CHANNEL_DIP_PIN, INPUT_PULLUP);

  if (!digitalRead(SERIAL_DIP_PIN)) {
    communicationMode = COMM_MODE_SERIAL;
  } else {
    if (!digitalRead(PREVIEW_DIP_PIN)) {
      communicationMode = COMM_MODE_IO_PREVIEW_PROGRAM;
    } else {
      communicationMode = COMM_MODE_IO_PROGRAM;
    }
  }

  if (!digitalRead(CHANNEL_DIP_PIN)) {
    selectedFrequency = RF69_FREQ_2;
  }
}

void setup() {
  Serial.begin(9600);
  while(!Serial) {}

  readConfiguration();

  Serial.println(communicationMode);

  switch (communicationMode)
  {
  case COMM_MODE_IO_PREVIEW_PROGRAM:
  case COMM_MODE_IO_PROGRAM:
    for (uint8_t pin : tallyPins) {
      pinMode(pin, INPUT_PULLUP);
    }
    break;
  case COMM_MODE_SERIAL:
    Serial1.begin(9600);
    break;
  
  default:
    break;
  }

  Serial.println("Init");

  u8g2.begin();

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset of RF
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rfManager.init())
    Serial.println("init failed");

  rf69.setFrequency(selectedFrequency);
  rf69.setTxPower(20, true);

  rfManager.setTimeout(5);
  rfManager.setRetries(3);

  statusChanged = true;

  for (uint8_t index = 0; index < 8; index++) {
    sendTallyStatus(index + 1, 0, TALLY_OFF);
    sendTallyStatus(index + 1, 1, TALLY_OFF);
  }
}

void readAndUpdateStatuses(bool * statuses, bool * newStatuses, uint8_t mode) {
  for (uint8_t index = 0; index < 8; index++) {
    uint8_t newPinValue = newStatuses[index];
    uint8_t oldPinValue = statuses[index];

    statuses[index] = newPinValue;

    if (newPinValue == oldPinValue)
      continue;

    statusChanged = true;
    sendTallyStatus(index + 1, mode, newPinValue == HIGH ? TALLY_ON : TALLY_OFF);
  }
}

void loop() {
  if (rfManager.available()) {
    Serial.println("Message available");
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rfManager.recvfrom(buf, &len, &from)) {
      timeouts[from - 1] = 0;
      if (buf[0] == 120) {
        sendTallyStatus(from, 0, programStatuses[from - 1]);
        sendTallyStatus(from, 1, previewStatuses[from - 1]);
      } else {
        if (batteries[from - 1] != buf[0]) {
          batteries[from - 1] = buf[0];

          Serial.print("Got battery: ");
          Serial.print(buf[0]);
          Serial.print(" from: ");
          Serial.println(from);

          statusChanged = true;
        }
      }
    }
  }

  if (millis() - savedTime > 1000) {
    for (uint8_t index = 0; index < 8; index++) {
      timeouts[index] = timeouts[index] + 1;

      if (timeouts[index] > 5) {
        int8_t oldValue = batteries[index];
        batteries[index] = -1;

        if (oldValue != batteries[index]) {
            statusChanged = true;
        }
      }
    }

    savedTime = millis();
  }

  bool newProgramStatuses[8];
  bool newPreviewStatuses[8];

  memcpy(newProgramStatuses, programStatuses, sizeof(programStatuses));
  memcpy(newPreviewStatuses, previewStatuses, sizeof(previewStatuses));

  if (communicationMode != COMM_MODE_SERIAL) {
    readFromPins(tallyPins, newProgramStatuses, newPreviewStatuses, 1);
  } else {
    readFromSerial(newProgramStatuses, newPreviewStatuses);
  }

  readAndUpdateStatuses(programStatuses, newProgramStatuses, TALLY_TYPE_PROGRAM);
  readAndUpdateStatuses(previewStatuses, newPreviewStatuses, TALLY_TYPE_PREVIEW);

  if (statusChanged) {
    Serial.println("Set monitor");

    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_t0_11_tf);
      
      uint8_t receiver = 0;
      while (receiver < 8) {
        uint8_t x = receiver > 3 ? 64 : 0;
        uint8_t y = receiver > 3 ? 12 * (receiver - 4) : 12 * receiver;

        char title[11] = {};
        char battery[4] = {};
        dtostrf(batteries[receiver], 3, 0, battery);

        char statusChar = ' ';
        if (programStatuses[receiver]) {
            statusChar = 'P';
        } else if (previewStatuses[receiver]) {
            statusChar = 'w';
        }

        char formattedBattery[5] = {};

        if (batteries[receiver] > 0) {
          snprintf(formattedBattery, 5, "%s%%", battery);
        } else {
          snprintf(formattedBattery, 5, " OFF");
        }

        snprintf(title, 11, "T%d:%c %s", receiver + 1, statusChar, formattedBattery);
        u8g2.drawStr(x, 28 + y, title);
        receiver++;
      }
    } while (u8g2.nextPage());

    statusChanged = false;
  }
}