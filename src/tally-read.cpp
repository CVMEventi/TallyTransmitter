#include "tally-read.h"
#include "constants.h"

bool readFromPins(uint8_t * tallyPins, bool * programStatuses, bool * previewStatuses, uint8_t mode) {
  bool statusChanged = false;

  switch (mode)
  {
  case COMM_MODE_IO_PREVIEW_PROGRAM: // 4 Preview/4 Program
    for (int i = 0; i < 4; i++) {
      uint8_t newPinValue = digitalRead(tallyPins[i]);
      programStatuses[i] = newPinValue != 1;
      statusChanged = true;
    }

    for (int x = 4; x < 8; x++) {
      uint8_t newPinValue = digitalRead(tallyPins[x]);
      previewStatuses[x - 4] = newPinValue != 1;
      statusChanged = true;
    }

    return statusChanged;

    break;
  case COMM_MODE_IO_PROGRAM: // 8 Programs
    for (int i = 0; i < 8; i++) {
      uint8_t newPinValue = digitalRead(tallyPins[i]);
      programStatuses[i] = newPinValue != 1;
      statusChanged = true;
    }
    break;
  default:
    break;
  }

  return true;
}

bool readFromSerial(bool * programStatuses, bool * previewStatuses) {
  String update = "";
  if (Serial1.available()) {
      update = Serial1.readStringUntil('S');
  } else {
      return false;
  }

  /*if (update.length() > 0) {
    Serial.println(update);
  }*/

  /*if (!update.endsWith("\0") || update.length() != 4) {
    return false;
  }*/

  int mode = update.charAt(0) - '0';
  int tallyNumber = update.charAt(1) - '0';
  int value = update.charAt(2) - '0';

  if (mode == 0) { // Program
    programStatuses[tallyNumber - 1] = value != 0;
  } else { // Preview
    previewStatuses[tallyNumber - 1] = value != 0;
  }

  return true;
}