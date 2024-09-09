#include <Servo.h>

Servo servoX; // Servo for X axis
Servo servoY; // Servo for Y axis

int x = 90; // Initial angle for servo X
int y = 90; // Initial angle for servo Y

void setup() {
  Serial.begin(9600);
  
  servoX.attach(10);
  servoY.attach(9);
  
  servoX.write(x);
  servoY.write(y);
  
  delay(1000); // Initial delay to allow servos to reach the starting position
}

void loop() {
  if (Serial.available()) { // Check if data is available in the serial buffer
    char input = Serial.read(); // Read the incoming data

    // Control servoY
    if (input == 'U') {
      if (y < 180) { // Ensure the angle is within bounds
        servoY.write(++y);
        delay(15); // Short delay for servo to move
      }
    } else if (input == 'D') {
      if (y > 0) { // Ensure the angle is within bounds
        servoY.write(--y);
        delay(15); // Short delay for servo to move
      }
    }

    // Control servoX
    if (input == 'L') {
      if (x > 0) { // Ensure the angle is within bounds
        servoX.write(--x);
        delay(15); // Short delay for servo to move
      }
    } else if (input == 'R') {
      if (x < 180) { // Ensure the angle is within bounds
        servoX.write(++x);
        delay(15); // Short delay for servo to move
      }
    }
  }
}
