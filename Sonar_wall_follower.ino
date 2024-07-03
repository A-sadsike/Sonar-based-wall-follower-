#include <PWM.h> // Include the PWM library
#include <NewPing.h>

int MotorL = 3;
int MotorLe1 = 10;
int MotorLe2 = 11;
int MotorRe1 = 6;
int MotorRe2 = 9;
int MotorR = 5;
int TRIGGER_PIN_STRAIGHT = A2;
int ECHO_PIN_STRAIGHT = A3;
int TRIGGER_PIN_LEFT = A4;
int ECHO_PIN_LEFT = A5;
int TRIGGER_PIN_RIGHT = A1;
int ECHO_PIN_RIGHT = A0;

#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).

NewPing sonarStraight(TRIGGER_PIN_STRAIGHT, ECHO_PIN_STRAIGHT);
NewPing sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);
NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT);

int Junction = 0;

void setup() {
  Serial.begin(9600);
  Junction = 0;
  
  pinMode(MotorLe1, OUTPUT);
  pinMode(MotorLe2, OUTPUT);
  pinMode(MotorRe1, OUTPUT);
  pinMode(MotorRe2, OUTPUT);
}

void Movefrwd() {
  pwmWrite(MotorLe1, 200); // Left motor enable 1
  digitalWrite(MotorLe2, LOW); // Left motor enable 2 (set LOW for forward)
  pwmWrite(MotorRe1, 200); // Right motor enable 1
  digitalWrite(MotorRe2, LOW); // Right motor enable 2 (set LOW for forward)
}

void Moveback() {
  pwmWrite(MotorLe1, 200); // Left motor enable 1
  digitalWrite(MotorLe2, HIGH); // Left motor enable 2 (set HIGH for reverse)
  pwmWrite(MotorRe1, 200); // Right motor enable 1
  digitalWrite(MotorRe2, HIGH); // Right motor enable 2 (set HIGH for reverse)
}

void Moveleft(int y) {
  pwmWrite(MotorLe1, 0); // Stop left motor
  digitalWrite(MotorLe2, LOW); // Left motor enable 2 (set LOW for forward)
  pwmWrite(MotorRe1, y); // Right motor enable 1
  digitalWrite(MotorRe2, LOW); // Right motor enable 2 (set LOW for forward)
}

void Moveright(int x) {
  pwmWrite(MotorLe1, x); // Left motor enable 1
  digitalWrite(MotorLe2, LOW); // Left motor enable 2 (set LOW for forward)
  pwmWrite(MotorRe1, 0); // Stop right motor
  digitalWrite(MotorRe2, LOW); // Right motor enable 2 (set LOW for forward)
}

void RotateLeft() {
  pwmWrite(MotorLe1, 0); // Stop left motor
  digitalWrite(MotorLe2, HIGH); // Left motor enable 2 (set LOW for forward)
  pwmWrite(MotorRe1, 200); // Right motor enable 1
  digitalWrite(MotorRe2, LOW); // Right motor enable 2 (set LOW for forward)
}

void RotateRight() {
  pwmWrite(MotorLe1, 200); // Left motor enable 1
  digitalWrite(MotorLe2, LOW); // Left motor enable 2 (set LOW for forward)
  pwmWrite(MotorRe1, 0); // Stop right motor
  digitalWrite(MotorRe2, HIGH); // Right motor enable 2 (set LOW for forward)
}

void stop() {
  pwmWrite(MotorLe1, 0); // Stop left motor
  digitalWrite(MotorLe2, LOW); // Set left motor enable 2 LOW
  pwmWrite(MotorRe1, 0); // Stop right motor
  digitalWrite(MotorRe2, LOW); // Set right motor enable 2 LOW
}

void CenterRobot(int desiredDistance)
 {
  int leftDistance = sonarLeft.ping_cm();
  int rightDistance = sonarRight.ping_cm();

  int difference = rightDistance - leftDistance;
  int correctionFactor = 2; // Adjust this value based on your robot's characteristics
 
  if (difference > desiredDistance) {
    // Turn slightly left to center the robot
    int leftSpeed = 200 - (correctionFactor * difference);
    Moveleft(leftSpeed);
  } else if (difference < -desiredDistance) {
    // Turn slightly right to center the robot
    int rightSpeed = 200 + (correctionFactor * (-difference));
    Moveright(rightSpeed);
  } else {
    // Robot is centered, move forward
    Movefrwd();
  }
}

void loop() {
  int frontDistance = sonarStraight.ping_cm();
  int leftDistance = sonarLeft.ping_cm();
  int rightDistance = sonarRight.ping_cm();

  Serial.print("Left Distance: ");
  Serial.print(leftDistance);
  Serial.println(" cm");

  Serial.print("Right Distance: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  // Check conditions for adjusting the robot's position
  CenterRobot(10);

  if (leftDistance >= 15 || rightDistance >= 15 || frontDistance <= 20) {
    delay(150);
    Junction++;
    Serial.println(Junction);
  }

  // The rest of your code remains the same...


  

  if (Junction == 1) {
    do {
      RotateRight();
      delay(35);
      stop();
      delay(35);
    } while (leftDistance <= 15 && rightDistance <= 15);
    Junction++;
  }

  if (Junction == 5) {
    do {
      RotateLeft();
      delay(35);
      stop();
      delay(35);
    } while (leftDistance <= 15 && rightDistance <= 15);
    Junction++;
  }

  if (Junction == 7) {
    do {
      RotateLeft();
      delay(150);
      stop();
      delay(150);
      RotateRight();
      delay(35);
      stop();
      delay(35);
    } while (leftDistance <= 15 && rightDistance <= 15);
    Junction++;
  }

  if (Junction == 9) {
    do {
      RotateRight();
      delay(150);
      stop();
      delay(150);
      RotateLeft();
      delay(35);
      stop();
      delay(35);
      RotateRight();
      delay(35);
      stop();
      delay(35);
    } while (leftDistance <= 15 && rightDistance <= 15);
    Junction++;
  }

  if (Junction == 11) {
    do {
      RotateRight();
      delay(35);
      stop();
      delay(35);
    } while (leftDistance <= 15 && rightDistance <= 15);
    Junction++;
  }

  if (Junction == 14) {
    do {
      RotateLeft();
      delay(35);
      stop();
      delay(35);
    } while (leftDistance <= 15 && rightDistance <= 15);
    Junction++;
  }

  if (Junction == 16) {
    do {
      RotateLeft();
      delay(35);
      stop();
      delay(35);
    } while (leftDistance <= 15 && rightDistance <= 15);
    Junction++;
  }

  if (Junction == 19) {
    do {
      RotateRight();
      delay(35);
      stop();
      delay(35);
    } while (leftDistance <= 15 && rightDistance <= 15);
    Junction++;
  }

  if (Junction == 20) {
    delay(500);
    stop();
  }
}
