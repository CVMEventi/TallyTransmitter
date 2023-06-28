#include <Arduino.h>

#ifndef TALLY_READ_H
#define TALLY_READ_H

bool readFromPins(uint8_t * tallyPins, bool * previewStatuses, bool * programStatuses, uint8_t mode);
bool readFromSerial(bool * programStatuses, bool * previewStatuses);

#endif