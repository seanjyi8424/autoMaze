// Bluetooth Helper
// This is the remote code to control a arduino bluetooth connected device's LED.
// Send a blink command to the arduino to turn on/off the LED.
//
// This code is meant to be used with the bluetooth_main.ino file.
bool lightOn = false;
int light = 0;

void setup() {
  Serial.begin(115200);
  // pinMode(LED_BUILTIN, OUTPUT); // set the digital pin as output:
}

void loop() {
  // This is remote, so send commands to the arduino
  // if (Serial.available() > 0) { // if there is data comming

  if (lightOn == false) {
    light = 0;
    Serial.write(static_cast<int>(light));
    lightOn = true;
  } 
  else if (lightOn == true) {
    light = 1;
    Serial.write(static_cast<int>(light));
    lightOn = false;
  }
  delay(3000);
}
