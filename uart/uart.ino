// Define Connections to 74HC595
const int latchPin = A2;
const int clockPin = A1;
const int dataPin = A0;

void setup() {
  // Setup pins as Outputs
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  // Start Serial on pins 0 (RX) / 1 (TX)
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for Serial ready
  }

  // Serial.println("Arduino ready!");

  displayOnLeds(0);
}


void displayOnLeds(uint8_t value) {
  // Mask to ensure only lower 6 bits are used
  value = value & 0x3F;  // 0b0011_1111

  // Align the 6 bits to the shift register (you may choose left/right alignment)
  // uint8_t shiftedValue = value << 2; // Shift left by 2 bits if LEDs are connected to Q2-Q7
  uint8_t shiftedValue = value;

  // Latch LOW to prepare for data shift
  digitalWrite(latchPin, LOW);

  // Shift out the bits
  shiftOut(dataPin, clockPin, MSBFIRST, shiftedValue);

  // Latch HIGH to apply output
  digitalWrite(latchPin, HIGH);
}


void loop() {

  // Wait until data arrives from Raspberry Pi
  if (Serial.available()) {
    byte received = Serial.read();

    /*
    // Display the byte on shift register (LEDs)
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, received); // send whole byte
    digitalWrite(latchPin, HIGH);

    Serial.print("Displayed: ");
    Serial.println(received); // Debug print over USB
    */
    displayOnLeds(received);

    delay(500); // Small delay to make it visible

    // Send back a response (incremented value for fun)
    byte response = (received + 1) & 0x3F; // Keep in 0–63 range
    Serial.write(response);
  }
}
