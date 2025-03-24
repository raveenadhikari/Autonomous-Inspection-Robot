#include <SPI.h>
#include <LoRa.h>
#include <ESP32Servo.h>

// Define the pins used by the LoRa transceiver
#define ss 5
#define rst 14
#define dio0 2

// Motor controller pins
const int IN1 = 25;
const int IN2 = 26;
const int ENA = 32;
const int IN3 = 27;
const int IN4 = 12;
//const int ENB = 33;  // <-- NOTE: This pin conflicts with the servo below. Change one if needed.

// Servo motor pins
const int servoPinX = 33;  // <-- Consider reassigning if ENB remains on 33.
const int servoPinY = 13;

Servo servoX;
Servo servoY;

// Ultrasonic sensor pins
const int trigFront = 4, echoFront = 15;
const int trigLeft  = 16, echoLeft  = 17;
const int trigRight = 21, echoRight = 22;

// Motor speeds and obstacle threshold
int motorSpeed = 180;       // Manual mode speed
int autoMotorSpeed = 120;   // Autonomous mode speed
int thresholdDistance = 40; // in cm

// Mode flag: false = manual, true = autonomous
bool autoMode = false;

// Function prototypes
void stopMotors();
void goForward(int speed);
void goBackward(int speed);
void goLeft(int speed);
void goRight(int speed);
void controlServoX(int angle);
void controlServoY(int angle);
long getDistance(int trigPin, int echoPin);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Receiver Initializing...");

  // Initialize LoRa transceiver
  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(433E6)) {  // Change frequency if needed (433E6, 868E6, or 915E6)
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xFF);  // Sync word should match your sender

  Serial.println("LoRa Receiver Initialized!");

  // Initialize motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  //pinMode(ENB, OUTPUT);

  // Initialize servo motors
  servoX.attach(servoPinX);
  servoY.attach(servoPinY);

  // Initialize ultrasonic sensor pins
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  // Start with motors stopped
  stopMotors();
}

void loop() {
  // Check for LoRa commands
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String command = "";
    while (LoRa.available()) {
      command += (char)LoRa.read();
    }
    command.trim(); // remove extra whitespace/newlines
    Serial.println("Received command: " + command);

    // Handle mode switching commands
    if (command == "auto") {
      autoMode = true;
      Serial.println("Switched to Autonomous Mode");
    }
    else if (command == "manual") {
      autoMode = false;
      stopMotors();
      Serial.println("Switched to Manual Mode");
    }
    
    // If in manual mode, process control commands
    if (!autoMode) {
      if      (command == "forward")   { goForward(motorSpeed); }
      else if (command == "backward")  { goBackward(motorSpeed); }
      else if (command == "left")      { goLeft(motorSpeed); }
      else if (command == "right")     { goRight(motorSpeed); }
      else if (command == "stop")      { stopMotors(); }
      else if (command == "cam_up")    { controlServoY(90); }  // Adjust angles as needed
      else if (command == "cam_down")  { controlServoY(0); }
      else if (command == "cam_left")  { controlServoX(90); }
      else if (command == "cam_right") { controlServoX(180); }
      else if (command == "cam_center") {
        controlServoX(90);   // Center X-axis (adjust if necessary)
        controlServoY(90);   // Center Y-axis (adjust if necessary)
        Serial.println("Centering camera");
      }
    }
  }

  // Autonomous mode loop
  if (autoMode) {
    long frontDistance = getDistance(trigFront, echoFront);
    long leftDistance  = getDistance(trigLeft, echoLeft);
    long rightDistance = getDistance(trigRight, echoRight);

    Serial.print("Front: "); Serial.print(frontDistance); Serial.print(" cm, ");
    Serial.print("Left: "); Serial.print(leftDistance); Serial.print(" cm, ");
    Serial.print("Right: "); Serial.print(rightDistance); Serial.println(" cm");

    if (frontDistance < thresholdDistance) {
      stopMotors();
      delay(500);
      // If both sides are blocked, back up; otherwise, turn toward the side with more space.
      if (leftDistance < thresholdDistance && rightDistance < thresholdDistance) {
        goBackward(autoMotorSpeed);
        delay(1000);
      } else if (leftDistance > rightDistance) {
        Serial.println("Turning Left");
        goLeft(motorSpeed);
        delay(700);
      } else if (leftDistance < rightDistance) {
        Serial.println("Turning Right");
        goRight(motorSpeed);
        delay(700);
      }
      stopMotors();
      delay(500);
    } else {
      goForward(autoMotorSpeed);
      delay(1000);
      stopMotors();
      delay(500);
    }
    delay(100);
  }
  
  delay(10); // Small delay to allow LoRa polling
}

void stopMotors() {
  // Stop Motor 1
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  // Stop Motor 2
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  Serial.println("Motors stopped");
}

void goForward(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Moving forward");
}

void goBackward(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Moving backward");
}

void goLeft(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Turning left");
}

void goRight(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Turning right");
}

void controlServoX(int angle) {
  servoX.write(angle);
  Serial.println("Moving camera to X-axis angle: " + String(angle));
}

void controlServoY(int angle) {
  servoY.write(angle);
  Serial.println("Moving camera to Y-axis angle: " + String(angle));
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Convert to centimeters
}
