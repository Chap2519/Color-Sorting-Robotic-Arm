#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  150   
#define SERVOMAX  600  

// Arm lengths
#define L1  15.0  
#define L2  12.0  
#define L3  5.0  

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Color Sensor (TCS3200)
#define S0 5
#define S1 18
#define S2 19
#define S3 23
#define OUT 4

// IR Sensor
#define IR_SENSOR 15  

// Motor Driver (L298N)
#define ENA 27
#define IN1 12
#define IN2 
#define ENB 26
#define IN3 25
#define IN4 14

// Servo Channels
#define BASE_SERVO 0
#define ELBOW_SERVO 1
#define WRIST_SERVO 2
#define GRIPPER_SERVO 3

// LED Indicators (New Pins)
#define RED_LED 2
#define BLUE_LED 13

int redFrequency, greenFrequency, blueFrequency;

void setup() {
    Serial.begin(115200);
    pwm.begin();
    pwm.setPWMFreq(50);

    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(OUT, INPUT);

    pinMode(IR_SENSOR, INPUT);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
  //  pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // LED Setup
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    turnOffLEDs();  // Ensure all LEDs start off

    // Set motors OFF
    stopRobot();

    // Set TCS3200 scaling to 20%
    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);
}

void loop() {
    if (digitalRead(IR_SENSOR) == LOW) {
        stopRobot();
        Serial.println("Object Detected!");
        delay(500);

        readColor();
        determineColor();
    } else {
        moveForward();  // Keep moving when no object is detected
    }
    delay(100);
}

void readColor() {
    digitalWrite(S2, LOW); digitalWrite(S3, LOW);
    redFrequency = pulseIn(OUT, LOW);

    digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
    greenFrequency = pulseIn(OUT, LOW);

    digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
    blueFrequency = pulseIn(OUT, LOW);

    Serial.print("Red: "); Serial.print(redFrequency);
    Serial.print(" Green: "); Serial.print(greenFrequency);
    Serial.print(" Blue: "); Serial.println(blueFrequency);
}

void determineColor() {
    turnOffLEDs();  // Ensure only one LED is active at a time

    if (redFrequency < greenFrequency && redFrequency < blueFrequency && redFrequency < 70) {
        Serial.println("Detected: RED");
        digitalWrite(RED_LED, HIGH);  

        placeRedObject();
    } else if (greenFrequency < redFrequency && greenFrequency < blueFrequency && greenFrequency < 270) {
        Serial.println("Detected: GREEN");
       // digitalWrite(GREEN_LED, HIGH);
      
    } else if (blueFrequency < redFrequency && blueFrequency < greenFrequency && blueFrequency < 270) {
        Serial.println("Detected: BLUE");
        digitalWrite(BLUE_LED, HIGH);
        pickAndPlaceBlue();
    } else {
        Serial.println("No Valid Color Detected");
        pickAndPlaceInvalid(); 
    }
}

void pickAndPlaceInvalid() {
    moveServo(ELBOW_SERVO, 30, 40);
    moveServo(GRIPPER_SERVO, 60, 40);  
    moveServo(ELBOW_SERVO, 30, 40);
    moveServo(BASE_SERVO, 30, 40);
    moveServo(WRIST_SERVO, 0, 40);
    moveServo(GRIPPER_SERVO, 90, 40);
    resetArm();
}

void pickAndPlaceBlue() {
    moveServo(ELBOW_SERVO, 30, 40);
    moveServo(GRIPPER_SERVO, 70, 40); 
     moveServo(WRIST_SERVO, 0, 40); 
    moveServo(ELBOW_SERVO, 90, 40);
    moveServo(BASE_SERVO, 200, 40);
   
    moveServo(GRIPPER_SERVO, 90, 40);
    resetArm();
}

void placeRedObject() {
    moveServo(ELBOW_SERVO, 30, 40);
    moveServo(GRIPPER_SERVO, 70, 40);  
    moveServo(ELBOW_SERVO, 50, 40);
    moveServo(BASE_SERVO, 90, 40);
    moveServo(WRIST_SERVO, 0, 40);
    moveServo(GRIPPER_SERVO, 90, 40);
    resetArm();
}

void resetArm() {
  digitalWrite(RED_LED, LOW);  
  digitalWrite(GREEN_LED, LOW);  
  digitalWrite(BLUE_LED, LOW);  
    moveServo(BASE_SERVO, 0, 40);
    moveServo(ELBOW_SERVO, 30, 40);
    moveServo(WRIST_SERVO, 30, 40);
    moveServo(GRIPPER_SERVO, 90, 40);
    delay(1000);
}

// Move servo gradually
void moveServo(int channel, int targetAngle, int stepDelay) {
    static int currentAngles[4] = {90, 90, 90, 90}; // Store last known positions, initialized to 90°

    int startAngle = currentAngles[channel];  
    if (startAngle > targetAngle) {  // Move downward
        for (int angle = startAngle; angle >= targetAngle; angle--) {
            pwm.setPWM(channel, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
            delay(stepDelay);
        }
    } else {  // Move upward
        for (int angle = startAngle; angle <= targetAngle; angle++) {
            pwm.setPWM(channel, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
            delay(stepDelay);
        }
    }
    currentAngles[channel] = targetAngle;  // Save new position
}

void moveForward() {
    analogWrite(ENA, 100);
    analogWrite(ENB, 100);
    digitalWrite(IN1, HIGH);
   // digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void stopRobot() {
    digitalWrite(IN1, LOW);
   // digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// Function to turn off all LEDs
void turnOffLEDs() {
    digitalWrite(RED_LED, LOW);
    //digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
}
