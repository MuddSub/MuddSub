/**
 * MuddSub Teensy Servo Test
 * Controls a servo motor using digital input pins. Set custom ranges by changing the degree_diff variables.
 * 
 * Pin Configuration:
 * - Servo Control: Pin 8
 * - Positive Direction: Pin 14 (adjustable degree change from 1 to +90 degrees when HIGH)
 * - Negative Direction: Pin 15 (adjustable degree change from -1 to -90 degrees when HIGH)
 */

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

#define SERVO_PIN 8 // pin 8 is the pin that the servo is attached to

// Driving these pins to a 3.3V signal will move the servo
#define SERVO_PIN_POSITIVE 14  // Pin for +1 to +90 degrees
#define SERVO_PIN_NEGATIVE 15  // Pin for -1 to -90 degrees
#define SERVO_PIN_RESET 16

// Define servo PWM limits in microseconds
#define SERVO_MIN_PWM 900     // -90 degrees
#define SERVO_MAX_PWM 2100    // +90 degrees
#define SERVO_CENTER_PWM 1500 // 0 degrees

// delay period in ms
#define DELAY_PERIOD 30

Servo extraServo;

// Change these values to move the servo
float degree_diff_1 =  10; // attached to pin 14
float degree_diff_2 = -10; // attached to pin 15

float pos = 0;

/**
 * Converts angle in degrees to PWM value in microseconds
 * @param degrees Angle in degrees (-90 to +90)
 * @return PWM value in microseconds (900-2100)
 */
int degreesToPWM(float degrees) {
    // Constrain input to valid range
    degrees = constrain(degrees, -90.0, 90.0);
    
    // Linear mapping from degrees to PWM
    // -90 degrees = 900 µs
    //   0 degrees = 1500 µs
    // +90 degrees = 2100 µs
    return map(degrees * 10, -900, 900, SERVO_MIN_PWM, SERVO_MAX_PWM);
}


// TODO: use protocol buffers?
void setup() {
  // Configure control pins
  pinMode(SERVO_PIN_POSITIVE, INPUT_PULLDOWN);
  pinMode(SERVO_PIN_NEGATIVE, INPUT_PULLDOWN);
  pinMode(SERVO_PIN_RESET, INPUT_PULLDOWN);
  pinMode(13, OUTPUT);


  Serial.begin(115200);

  Serial.println("Starting");

  Wire.begin();
  
  extraServo.attach(SERVO_PIN);
  if (!extraServo.attached()) {
    Serial.println("Failed to attach servo!");
    while(1) {
      // Halt program if servo fails to attach
      delay(1000);
    }
  }
  Serial.println("Servo attached successfully");

  extraServo.writeMicroseconds(SERVO_CENTER_PWM);
  delay(500);
}

void loop() {

  // Check control pins and set servo position
  bool toggle_1 = false;
  if (digitalRead(SERVO_PIN_POSITIVE) == HIGH && !toggle_1) {
    toggle_1 = true; 
    pos = 90.0;
    digitalWrite(13, HIGH);
  } else if (digitalRead(SERVO_PIN_NEGATIVE) == HIGH && !toggle_1) {
      pos = -90.0;
      digitalWrite(13, LOW);
      toggle_1 = true;
  } else if (digitalRead(SERVO_PIN_RESET) == HIGH) {
    pos = 0;
    toggle_1 = false;
  }

  extraServo.writeMicroseconds(degreesToPWM(pos));
  Serial.printf("Moving to %.1f degrees\n", pos);

  delay(DELAY_PERIOD);


}