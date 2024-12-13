#include "BluetoothSerial.h" // Include BluetoothSerial library

String device_name = "JUDHI T-BEAM"; // Bluetooth device name
BluetoothSerial SerialBT; // Bluetooth Serial object

int motionSensor = 2; // Pin for motion sensor
int motionState = LOW; // State of motion sensor
int motionVal = 0; // Value read from motion sensor

int soundSensor = 14; // Pin for sound sensor
unsigned long lastSoundToggleTime = 0; // Last time sound was detected
const unsigned long soundDebounceDelay = 1000; // Debounce delay for sound sensor

int echoPin = 23; // Echo pin for ultrasonic sensor
int trigPin = 33; // Trigger pin for ultrasonic sensor

#define LEFT_MOTOR_FORWARD 21 // Left motor forward pin
#define LEFT_MOTOR_BACKWARD 22 // Left motor backward pin
#define RIGHT_MOTOR_FORWARD 0 // Right motor forward pin
#define RIGHT_MOTOR_BACKWARD 4 // Right motor backward pin

int buzzerPin = 13; // Pin for buzzer
#define DISTANCE_THRESHOLD 15 // Distance threshold for obstacle avoidance

String currentMotorDirection = "STOPPED"; // Current motor direction
bool motorsEnabled = true; // Flag to enable or disable motors

unsigned long lastDirectionMessageTime = 0; // Last time direction was sent
const unsigned long directionMessageInterval = 60000; // Interval to send direction

unsigned long lastOperationTime = 0; // Last time robot operation changed
const unsigned long operationInterval = 15000; // Interval for operation changes
bool isStopped = false; // Flag for stopped state

void setup() {
  Serial.begin(9600); // Initialize serial communication
  SerialBT.begin(device_name); // Initialize Bluetooth
  Serial.printf("Device \"%s\" is ready for pairing.\n", device_name.c_str());

  pinMode(motionSensor, INPUT); // Set motion sensor pin as input
  pinMode(soundSensor, INPUT_PULLUP); // Set sound sensor pin as input with pull-up

  pinMode(echoPin, INPUT); // Set echo pin as input
  pinMode(trigPin, OUTPUT); // Set trigger pin as output

  pinMode(LEFT_MOTOR_FORWARD, OUTPUT); // Set left motor forward pin as output
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT); // Set left motor backward pin as output
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT); // Set right motor forward pin as output
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT); // Set right motor backward pin as output

  pinMode(buzzerPin, OUTPUT); // Set buzzer pin as output
  digitalWrite(buzzerPin, LOW); // Turn off buzzer initially
}

// Function to measure distance using ultrasonic sensor
int measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // Measure pulse duration

  if (duration == 0) return -1; // Return -1 if no echo detected

  return duration / 58; // Convert duration to distance in cm
}

// Move robot forward
void moveForward() {
  if (!motorsEnabled) return;
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  currentMotorDirection = "FORWARD";
}

// Turn robot left
void turnLeft() {
  if (!motorsEnabled) return;
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  currentMotorDirection = "LEFT";
}

// Turn robot right
void turnRight() {
  if (!motorsEnabled) return;
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  currentMotorDirection = "RIGHT";
}

// Stop the robot's motors
void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  currentMotorDirection = "STOPPED";
}

// Send current motor direction over Bluetooth
void sendMotorDirection() {
  String directionMessage = "Motor Direction: " + currentMotorDirection;
  SerialBT.println(directionMessage);
  Serial.println(directionMessage);
}

// Handle motion sensor events
void handleMotionSensor() {
  motionVal = digitalRead(motionSensor);

  if (motionVal == HIGH && motionState == LOW) {
    Serial.println("Motion detected!");
    SerialBT.println("Motion detected!");
    motionState = HIGH;
  } else if (motionVal == LOW && motionState == HIGH) {
    Serial.println("No motion detected!");
    SerialBT.println("No motion detected!");
    motionState = LOW;
  }
}

// Handle sound sensor events
void handleSoundSensor() {
  unsigned long currentMillis = millis();
  int soundVal = digitalRead(soundSensor);
  if (soundVal == HIGH && (currentMillis - lastSoundToggleTime > soundDebounceDelay)) {
    Serial.println("Sound detected!");
    SerialBT.println("Sound detected!");
    lastSoundToggleTime = currentMillis;
  }
}

// Check if both motion and sound are detected and activate buzzer
void checkMotionAndSound() {
  if (motionVal == HIGH && digitalRead(soundSensor) == HIGH) {
    Serial.println("Both motion and sound detected! Activating buzzer.");
    SerialBT.println("Both motion and sound detected! Activating buzzer.");
    digitalWrite(buzzerPin, HIGH);
    delay(1000);
    digitalWrite(buzzerPin, LOW);
  }
}

// Handle obstacle avoidance logic using ultrasonic sensor
void handleObstacleAvoidance() {
  if (!motorsEnabled || isStopped) return;

  int distance = measureDistance();
  if (distance >= 0) {
    Serial.printf("Ultrasonic Distance: %d cm\n", distance);
    SerialBT.printf("Ultrasonic Distance: %d cm\n", distance);

    if (distance < DISTANCE_THRESHOLD) {
      stopMotors(); // Stop if obstacle is too close

      turnLeft(); // Check left direction
      delay(500);
      int leftDistance = measureDistance();

      turnRight(); // Check right direction
      delay(1000);
      int rightDistance = measureDistance();

      if (leftDistance > DISTANCE_THRESHOLD && leftDistance >= rightDistance) {
        turnLeft();
        delay(500);
        moveForward();
      } else if (rightDistance > DISTANCE_THRESHOLD) {
        turnRight();
        delay(500);
        moveForward();
      }
    } else {
      moveForward(); // Move forward if no obstacle
    }
  }
}

// Handle Bluetooth commands to start or stop the robot
void handleBluetoothCommands() {
  if (SerialBT.available()) {
    String dataFromBluetooth = SerialBT.readString();
    Serial.println("Received via Bluetooth: " + dataFromBluetooth);

    if (dataFromBluetooth.indexOf("STOP") != -1) {
      Serial.println("Motors stopped by Bluetooth command.");
      motorsEnabled = false;
      stopMotors();
    } else if (dataFromBluetooth.indexOf("START") != -1) {
      Serial.println("Motors enabled by Bluetooth command.");
      motorsEnabled = true;
    }
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Periodically stop the robot and activate sensors
  if (currentMillis - lastOperationTime >= operationInterval) {
    lastOperationTime = currentMillis;

    if (!isStopped) {
      stopMotors();
      Serial.println("Robot stopped. Activating sensors.");
      SerialBT.println("Robot stopped. Activating sensors.");
      unsigned long stopStartTime = millis();
      while (millis() - stopStartTime < operationInterval) {
        handleMotionSensor();
        handleSoundSensor();
        checkMotionAndSound();
        delay(300);
      }
      isStopped = true;
    } else {
      Serial.println("Resuming robot operation.");
      SerialBT.println("Resuming robot operation.");
      isStopped = false;
    }
  }

  if (!isStopped) {
    handleObstacleAvoidance();
    handleBluetoothCommands();

    // Periodically send motor direction
    if (currentMillis - lastDirectionMessageTime >= directionMessageInterval) {
      sendMotorDirection();
      lastDirectionMessageTime = currentMillis;
    }
  }

  delay(50); // Small delay for stability
}
