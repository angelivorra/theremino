#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

// Configuration parameters
const int NUM_ZONES = 6;
const int MIN_DISTANCE = 10;
const int SAMPLES_FOR_MAX = 20;
const unsigned long SETUP_DELAY = 5000;
const int OBJECT_DETECTION_THRESHOLD = 15;  // Prueba con valores mayores

// Variables for zone detection
int maxDistance;
int baselineDistance;
int zoneSize;
int currentZone = 0;
bool objectPresent = false;

// Reading interval control
const unsigned long READ_INTERVAL = 50;
unsigned long lastReadTime = 0;

void calculateZones() {
  zoneSize = (baselineDistance - MIN_DISTANCE) / NUM_ZONES;

  Serial.println("\nConfiguración de zonas:");
  Serial.print("Distancia mínima de detección: ");
  Serial.println(MIN_DISTANCE);
  Serial.print("Distancia base (sin objeto): ");
  Serial.println(baselineDistance);
  Serial.print("Tamaño de la zona: ");
  Serial.println(zoneSize);

  for (int i = 1; i <= NUM_ZONES; i++) {
    int zoneStart = baselineDistance - ((i - 1) * zoneSize);
    int zoneEnd = baselineDistance - (i * zoneSize);
    Serial.print("Zona ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(zoneEnd);
    Serial.print(" - ");
    Serial.println(zoneStart);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("¡Error al detectar el sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);

  // Incrementa el presupuesto a 100 ms
  sensor.setMeasurementTimingBudget(100000);

  Serial.println("Esperando 5 segundos antes de la calibración...");
  Serial.println("Por favor, asegúrese de que no haya objetos frente al sensor");
  delay(SETUP_DELAY);

  // Calibración inicial
  long totalDistance = 0;
  int maxReading = 0;
  Serial.println("Iniciando lecturas de calibración...");
  for (int i = 0; i < SAMPLES_FOR_MAX; i++) {
    int reading = sensor.read();
    if (sensor.timeoutOccurred()) {
      Serial.println("¡Tiempo de espera del sensor durante la calibración!");
      continue;
    }
    totalDistance += reading;
    if (reading > maxReading) {
      maxReading = reading;
    }
    Serial.print("Lectura de calibración ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(reading);
  }

  baselineDistance = totalDistance / SAMPLES_FOR_MAX;
  maxDistance = maxReading;  // Usar la lectura máxima obtenida durante la calibración

  calculateZones();
}

void loop() {
  if (millis() - lastReadTime >= READ_INTERVAL) {
    lastReadTime = millis();

    int reading = sensor.read();
    if (sensor.timeoutOccurred()) {
      Serial.println("¡Tiempo de espera del sensor durante la lectura!");
      return;
    }

    Serial.print("Lectura cruda: ");
    Serial.println(reading);

    // Calcular la diferencia respecto a la distancia base
    int distanceDifference = baselineDistance - reading;

    // Verificar si el objeto está dentro del rango de detección con umbral
    if (distanceDifference > OBJECT_DETECTION_THRESHOLD && reading > MIN_DISTANCE) {
      int zone = distanceDifference / zoneSize + 1;
      if (zone > NUM_ZONES) {
        zone = NUM_ZONES;
      }
      if (!objectPresent) {
        objectPresent = true;
        currentZone = zone;
        Serial.print("Objeto detectado en la Zona ");
        Serial.println(zone);
      } else if (zone != currentZone) {
        currentZone = zone;
        Serial.print("Cambio a Zona ");
        Serial.println(zone);
      }
    } else {
      if (objectPresent) {
        objectPresent = false;
        currentZone = 0;
        Serial.println("Objeto salió del rango de detección");
      }
    }
  }
}