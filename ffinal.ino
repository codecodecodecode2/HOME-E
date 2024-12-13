#include "BluetoothSerial.h"

String device_name = "JUDHI T-BEAM";
BluetoothSerial SerialBT;

int motionSensor = 2;
int motionState = LOW;
int motionVal = 0;

int soundSensor = 14;
unsigned long lastSoundToggleTime = 0;
const unsigned long soundDebounceDelay = 1000;

int echoPin = 23;
int trigPin = 33;

#define LEFT_MOTOR_FORWARD 21
#define LEFT_MOTOR_BACKWARD 22
#define RIGHT_MOTOR_FORWARD 0
#define RIGHT_MOTOR_BACKWARD 4

int buzzerPin = 13;
#define DISTANCE_THRESHOLD 15

String currentMotorDirection = "STOPPED";
bool motorsEnabled = true;

unsigned long lastDirectionMessageTime = 0;
const unsigned long directionMessageInterval = 60000;

unsigned long lastOperationTime = 0;
const unsigned long operationInterval = 15000;
bool isStopped = false;

void setup() {
  Serial.begin(9600);
  SerialBT.begin(device_name);
  Serial.printf("Device \"%s\" is ready for pairing.\n", device_name.c_str());

  pinMode(motionSensor, INPUT);
  pinMode(soundSensor, INPUT_PULLUP);

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
}

int measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) return -1;

  return duration / 58;
}

void moveForward() {
  if (!motorsEnabled) return;
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  currentMotorDirection = "FORWARD";
}

void turnLeft() {
  if (!motorsEnabled) return;
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  currentMotorDirection = "LEFT";
}

void turnRight() {
  if (!motorsEnabled) return;
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  currentMotorDirection = "RIGHT";
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  currentMotorDirection = "STOPPED";
}

void sendMotorDirection() {
  String directionMessage = "Motor Direction: " + currentMotorDirection;
  SerialBT.println(directionMessage);
  Serial.println(directionMessage);
}

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

void handleSoundSensor() {
  unsigned long currentMillis = millis();
  int soundVal = digitalRead(soundSensor);
  if (soundVal == HIGH && (currentMillis - lastSoundToggleTime > soundDebounceDelay)) {
    Serial.println("Sound detected!");
    SerialBT.println("Sound detected!");
    lastSoundToggleTime = currentMillis;
  }
}

void checkMotionAndSound() {
  if (motionVal == HIGH && digitalRead(soundSensor) == HIGH) {
    Serial.println("Both motion and sound detected! Activating buzzer.");
    SerialBT.println("Both motion and sound detected! Activating buzzer.");
    digitalWrite(buzzerPin, HIGH);
    delay(1000);
    digitalWrite(buzzerPin, LOW);
  }
}

void handleObstacleAvoidance() {
  if (!motorsEnabled || isStopped) return;

  int distance = measureDistance();
  if (distance >= 0) {
    Serial.printf("Ultrasonic Distance: %d cm\n", distance);
    SerialBT.printf("Ultrasonic Distance: %d cm\n", distance);

    if (distance < DISTANCE_THRESHOLD) {
      stopMotors();

      turnLeft();
      delay(500);
      int leftDistance = measureDistance();

      turnRight();
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
      moveForward();
    }
  }
}

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

    if (currentMillis - lastDirectionMessageTime >= directionMessageInterval) {
      sendMotorDirection();
      lastDirectionMessageTime = currentMillis;
    }
  }

  delay(50);
}
