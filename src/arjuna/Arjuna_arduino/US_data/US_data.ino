#include <NewPing.h>

// Sensor 1 pins
#define TRIGGER_PIN_1  2
#define ECHO_PIN_1     3
#define MAX_DISTANCE_1 200  // Maximum distance in cm

// Sensor 2 pins
#define TRIGGER_PIN_2  4
#define ECHO_PIN_2     5
#define MAX_DISTANCE_2 200  // Maximum distance in cm

// Create NewPing objects for both sensors
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE_1);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE_2);

void setup() {
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  // Read distance from both sensors
  unsigned int distance1 = sonar1.ping_cm();
  unsigned int distance2 = sonar2.ping_cm();
  
  // Handle zero readings (out of range)
  if (distance1 == 0) distance1 = MAX_DISTANCE_1;
  if (distance2 == 0) distance2 = MAX_DISTANCE_2;
  
  // Send data in JSON format for easy parsing
  Serial.print("{\"sensor1\":");
  Serial.print(distance1);
  Serial.print(",\"sensor2\":");
  Serial.print(distance2);
  Serial.println("}");
  
  delay(100);  // 10Hz update rate
}
