#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

// Configuration parameters
const int NUM_ZONES = 6;
const int MIN_DISTANCE = 10;
const unsigned long SETUP_DELAY = 5000;
const unsigned long MEASUREMENT_PERIOD = 5000;  // Período de medición en milisegundos (reducido a la mitad)
const int OBJECT_DETECTION_THRESHOLD = 10;  // Umbral para detectar un objeto (ajustado)
const int SAMPLES_FOR_BASELINE = 50;

// Variables for zone detection
int currentZone = 0;
bool objectPresent = false;
int baselineDistance;

// Reading interval control
const unsigned long READ_INTERVAL = 50; // Intervalo de lectura en milisegundos
unsigned long lastReadTime = 0; // Último tiempo de lectura

void calculateZones(int centerDistance) {

  Serial.println("\nConfiguración de zonas:");
  Serial.print("Distancia mínima de detección: ");
  Serial.println(MIN_DISTANCE);
  Serial.print("Distancia base (sin objeto): ");
  Serial.println(centerDistance);
  Serial.print("Tamaño de la zona: ");

}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.startContinuous(50);

  // Calibrate baseline distance
  Serial.println("Iniciando lecturas de calibración...");
  long sum = 0;
  for (int i = 0; i < SAMPLES_FOR_BASELINE; i++) {
    sum += sensor.readRangeContinuousMillimeters();
    delay(READ_INTERVAL);
  }
  baselineDistance = sum / SAMPLES_FOR_BASELINE;
  Serial.print("Distancia base (sin objeto): ");
  Serial.println(baselineDistance);

  calculateZones(baselineDistance);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;

    int distance = sensor.readRangeContinuousMillimeters();
    Serial.print("Lectura cruda: ");
    Serial.print(distance);
  }
}