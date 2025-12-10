#include "HX711.h"
#include <ESP32Servo.h>

// --- WIRING CONFIGURATION ---
// Load Cell
const int DOUT_PIN = 7;
const int SCK_PIN = 4;



// Servo
const int SERVO_PIN = 5;

// --- OBJECTS ---
HX711 scale;
Servo myServo;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to wake up

  // Initialize Scale
  scale.begin(DOUT_PIN, SCK_PIN);
  scale.set_scale(-37.2); // Calibrated Factor
  scale.tare(); 
  
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

  // 2. Read Lift
  float lift = scale.get_units(1); 

  // 3. Send over Serial
  // Format: "L:5.23"
  Serial.print("L:");
  Serial.println(lift, 2);

  delay(20); // ~50Hz
}
