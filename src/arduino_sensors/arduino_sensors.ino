#include "HX711.h"

// --- WIRING CONFIGURATION ---
// HX711 Circuit
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

// Pitot Tube (Analog)
const int PITOT_PIN = A0;

// --- OBJECTS ---
HX711 scale;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to wake up

  // Initialize Load Cell
  // Serial.println("Initializing Scale...");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Taring (Assuming no load at startup)
  scale.set_scale();
  scale.tare(); 
  
  // TODO: You must calibrate this factor!
  // scale.set_scale(CALIBRATION_FACTOR); 
}

void loop() {
  // 1. Read Lift (Load Cell)
  // get_units(5) returns average of 5 readings
  float lift_raw = scale.get_units(1); 

  // 2. Read Airspeed (Pitot Tube)
  // ESP32 ADC is 12-bit (0-4095) usually, but Arduino core might map to 10-bit.
  // analogRead returns integer.
  int adc_val = analogRead(PITOT_PIN);
  
  // Convert to Voltage (Assuming 3.3V logic for ESP32)
  float voltage = (adc_val / 4095.0) * 3.3; 
  
  // TODO: Convert voltage to airspeed using Bernoulli's equation
  // For now, sending raw voltage or simple scaling
  float airspeed_approx = voltage * 10.0; 

  // 3. Send over Serial
  // Format: "L:5.23,A:12.50"
  Serial.print("L:");
  Serial.print(lift_raw, 2);
  Serial.print(",A:");
  Serial.println(airspeed_approx, 2);

  delay(20); // ~50Hz
}
