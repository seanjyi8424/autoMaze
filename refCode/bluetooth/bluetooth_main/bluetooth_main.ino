/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-bluetooth
 */

// NOTE: change the Serial to other Serial/Software Serial if you connects Bluetooth module to other pins
// #define LED_PIN 8
// #include <avr/sleep.h>

int light = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); // set the digital pin as output:
  digitalWrite(LED_BUILTIN, HIGH);  // turn off LED
  delay(1500);
  digitalWrite(LED_BUILTIN, LOW);  // turn off LED
  delay(1500);
  Serial.println("Bluetooth is ready to receive commands...");
}

void loop() {
  if (Serial.available() > 0) { // if there is data comming
    light = Serial.read(); 
  }

  Serial.print(static_cast<char>(light));
  Serial.print("\n");

  if (light == 0) {
    digitalWrite(LED_BUILTIN, LOW);  // turn off LED
    Serial.println("LED is OFF");
  } else if (light == 1) {
    digitalWrite(LED_BUILTIN, HIGH); // turn on LED
    Serial.println("LED is ON");
  } else {
    Serial.println("Waiting for valid command...");
  }
  delay(3000);
}
