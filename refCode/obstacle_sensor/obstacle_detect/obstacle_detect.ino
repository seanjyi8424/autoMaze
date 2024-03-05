/*
 * Original code from ArduinoGetStarted.com
 * This example code is in the public domain
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-infrared-obstacle-avoidance-sensor
 *
 * Modified by Artip Nakchinda
 *
 */

// Arduino's pin connected to OUT pin of IR obstacle avoidance sensor
const int O_SENSOR_RIGHT = 11;
const int O_SENSOR_LEFT = 12;

int lastState = HIGH;  // the previous state from the input pin
int currentState;      // the current reading from the input pin

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  // initialize the Arduino's pin as aninput
  pinMode(O_SENSOR_LEFT, INPUT);
}

void loop() {
  // read the state of the the input pin:
  currentState = digitalRead(O_SENSOR_LEFT);

  if (lastState == HIGH && currentState == LOW)
    Serial.println("Obstacle not detected");
  else if (lastState == LOW && currentState == HIGH)
    Serial.println("Obstacle detected");

  delay(50);
  // save the the last state
  lastState = currentState;
}
