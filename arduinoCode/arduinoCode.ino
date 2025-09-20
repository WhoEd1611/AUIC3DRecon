#include <Servo.h>

Servo myServo;
const int servoPin = 9;
int lastAngle = -1;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  Serial.println("Enter angle (0-180):");
}

void loop() {
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();         // read integer (e.g. 90)
    while (Serial.available() > 0) Serial.read(); // flush leftover chars (CR/LF)
    angle = constrain(angle, 0, 180);

    // only write if it's a new value (prevents repeated writes)
    if (angle != lastAngle) {
      myServo.write(angle);
      lastAngle = angle;
      Serial.print("Moved to: ");
      Serial.println(angle);
    }
  }
}
