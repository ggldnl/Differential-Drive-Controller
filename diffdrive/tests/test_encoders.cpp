#include "libs/encoder/encoder.hpp"


// N20 Motor -> High resolution, motor shaft, magnetic encoder
const uint8_t encoder1A = 2;
const uint8_t encoder1B = 7; // not used

// Low resolution, output shaft, infrared encoder
const uint8_t encoder2A = 3;
const uint8_t encoder2B = 8; // not used

// Instantiate components
Encoder encoder1(encoder1A, Encoder::COUNT_MODE);
Encoder encoder2(encoder2A, Encoder::PERIOD_MODE);

void setup() {

  Serial.begin(115200);
  Serial.println("Encoder test starting...");

}

void loop() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint >= 200) {  // print every 200 ms
    lastPrint = now;

    EncoderData data1 = encoder1.getData();
    EncoderData data2 = encoder2.getData();

    Serial.print("Encoder1 ticks: ");
    Serial.print(data1.ticks);
    Serial.print(" | Encoder2 ticks: ");
    Serial.println(data2.ticks);
  }
}
