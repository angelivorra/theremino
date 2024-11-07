#include <Arduino.h>

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

// Configuration parameters
const int NUM_ZONES = 6;
const int MIN_DISTANCE = 10;
const int SAMPLES_FOR_MAX = 20;
const float MARGIN_FACTOR = 1.1;
const unsigned long SETUP_DELAY = 5000;
const int DETECTION_THRESHOLD = 100;  // Increased threshold for more definitive detection
const int READINGS_AVERAGE = 3;      // Average multiple readings for stability

// Variables for zone detection
int maxDistance;
int baselineDistance;
int zoneSize;
int currentZone = 0;
bool objectPresent = false;
int lastReadings[READINGS_AVERAGE];  // Array to store last readings
int readingIndex = 0;

// Reading interval control
const unsigned long READ_INTERVAL = 20;  // Slightly increased for stability
unsigned long lastReadTime = 0;

int getAverageReading() {
  int reading = sensor.read();
  if (sensor.timeoutOccurred()) {
    Serial.println("Sensor timeout occurred during reading!");
    return -1;
  }

  lastReadings[readingIndex] = reading;
  readingIndex = (readingIndex + 1) % READINGS_AVERAGE;

  long sum = 0;
  for(int i = 0; i < READINGS_AVERAGE; i++) {
    sum += lastReadings[i];
  }
  int average = sum / READINGS_AVERAGE;

  // Log the readings
  Serial.print("Raw Reading: ");
  Serial.print(reading);
  Serial.print(" | Average Reading: ");
  Serial.println(average);

  return average;
}

void calculateZones() {
  zoneSize = (maxDistance - MIN_DISTANCE) / NUM_ZONES;

  Serial.println("\nZone Configuration:");
  Serial.print("Baseline Distance: ");
  Serial.println(baselineDistance);
  Serial.print("Max Detection Distance: ");
  Serial.println(maxDistance);
  Serial.print("Zone Size: ");
  Serial.println(zoneSize);

  for (int i = 1; i <= NUM_ZONES; i++) {
    int zoneStart = maxDistance - (i * zoneSize);
    int zoneEnd = maxDistance - ((i-1) * zoneSize);
    Serial.print("Zone ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(zoneStart);
    Serial.print(" - ");
    Serial.println(zoneEnd);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize averaging array
  for(int i = 0; i < READINGS_AVERAGE; i++) {
    lastReadings[i] = 0;
  }

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect sensor!");
    while (1);
  }

  // Adjust distance mode and timing budget if necessary
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(50000);  // Increased timing budget for better accuracy
  sensor.startContinuous(50);  // Increased inter-measurement period to match timing budget

  Serial.println("Waiting 5 seconds before calibration...");
  Serial.println("Please ensure no objects are in front of the sensor");
  delay(SETUP_DELAY);

  // Determine baseline with better averaging
  long totalDistance = 0;
  Serial.println("Starting calibration readings...");
  for (int i = 0; i < SAMPLES_FOR_MAX; i++) {
    int reading = sensor.read();
    if (sensor.timeoutOccurred()) {
      Serial.println("Sensor timeout during calibration!");
      continue;
    }
    Serial.print("Calibration reading ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(reading);
    totalDistance += reading;
    delay(50);
  }

  baselineDistance = totalDistance / SAMPLES_FOR_MAX;
  maxDistance = baselineDistance - DETECTION_THRESHOLD;

  calculateZones();
  Serial.println("Calibration complete. Ready for detection.");
}

int getZone(int distance) {
  if (distance == -1) return currentZone; // Keep last zone on timeout
  if (distance > (baselineDistance - DETECTION_THRESHOLD) || distance < MIN_DISTANCE) {
    return 0;
  }

  int zone = NUM_ZONES - ((distance - MIN_DISTANCE) / zoneSize);
  zone = constrain(zone, 1, NUM_ZONES);

  // Log the computed zone
  Serial.print("Computed Zone: ");
  Serial.println(zone);

  return zone;
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;

    int distance = getAverageReading();
    if (distance == -1) {
      Serial.println("Sensor timeout!");
      return;
    }

    // Log the distance
    Serial.print("Distance: ");
    Serial.println(distance);

    int newZone = getZone(distance);

    if (newZone == 0 && objectPresent) {
      objectPresent = false;
      currentZone = 0;
      Serial.println("Object left detection range");
    }
    else if (newZone > 0) {
      if (!objectPresent) {
        objectPresent = true;
        Serial.println("Object entered detection range");
      }

      if (newZone != currentZone) {
        currentZone = newZone;
        Serial.print("Zone: ");
        Serial.print(currentZone);
        Serial.print(" | Distance: ");
        Serial.println(distance);
      }
    }
  }
}