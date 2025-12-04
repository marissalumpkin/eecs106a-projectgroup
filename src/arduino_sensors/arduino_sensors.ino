#include "HX711.h"
#include <ESP32Servo.h>

// --- WIRING CONFIGURATION ---
// Left Load Cell
const int LEFT_DOUT_PIN = 7;
const int LEFT_SCK_PIN = 4;

// Right Load Cell
const int RIGHT_DOUT_PIN = 9;
const int RIGHT_SCK_PIN = 12;

// Pitot Tube (Analog)
const int PITOT_PIN = A0;

// Servo
const int SERVO_PIN = 5;

// --- OBJECTS ---
HX711 scale_left;
HX711 scale_right;
Servo myServo;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to wake up

  // Initialize Left Scale
  scale_left.begin(LEFT_DOUT_PIN, LEFT_SCK_PIN);
  scale_left.set_scale(-37.2); // Calibrated Factor
  scale_left.tare(); 

  // Initialize Right Scale
  scale_right.begin(RIGHT_DOUT_PIN, RIGHT_SCK_PIN);
  scale_right.set_scale(-37.2); // Assuming same factor
  scale_right.tare(); 
  
  // Initialize Servo
  myServo.attach(SERVO_PIN);
  myServo.write(0); // Start at 0 degrees 
}

void loop() {
  // 1. Check for Incoming Serial Commands (Servo)
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove whitespace
    
    // Format: "S:90"
    if (command.startsWith("S:")) {
      int angle = command.substring(2).toInt();
      // Constrain angle to safe limits (0-180)
      angle = constrain(angle, 0, 180);
      myServo.write(angle);
    }
  }

  // 2. Read Lift (Left & Right)
  float lift_left = scale_left.get_units(1); 
  float lift_right = scale_right.get_units(1); 

  // 3. Read Airspeed (Pitot Tube)
  int adc_val = analogRead(PITOT_PIN);
  float voltage = (adc_val / 4095.0) * 3.3; 
  float airspeed_approx = voltage * 10.0; 

  // 4. Send over Serial
  // Format: "LL:5.23,LR:4.80,A:12.50"
  Serial.print("LL:");
  Serial.print(lift_left, 2);
  Serial.print(",LR:");
  Serial.print(lift_right, 2);
  Serial.print(",A:");
  Serial.println(airspeed_approx, 2);

  delay(20); // ~50Hz
}
