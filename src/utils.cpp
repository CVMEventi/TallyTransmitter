#include <Arduino.h>

float calculateBatteryVoltage(uint8_t rawValue[]) {
    float batValue = ((rawValue[0] - 48) * 1000) + ((rawValue[1] - 48) * 100) + ((rawValue[2] - 48) * 10) + (rawValue[3] - 48);
    batValue *= 2;     // we divided by 2, so multiply back
    batValue *= 3.3;   // Multiply by 3.3V, our reference voltage
    batValue /= 1024;  // convert to voltage
    return batValue;
}