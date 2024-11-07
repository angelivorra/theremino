#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

// Configuration parameters
const int NUM_ZONES = 6;
const int MIN_DISTANCE = 10;
const int SAMPLES_FOR_BASELINE = 50;  // Número de muestras para calcular la distancia base (reducido a la mitad)
const unsigned long SETUP_DELAY = 5000;
const unsigned long MEASUREMENT_PERIOD = 5000;  // Período de medición en milisegundos (reducido a la mitad)
const int OBJECT_DETECTION_THRESHOLD = 20;  // Umbral para detectar un objeto (ajustado)
const int HYSTERESIS = 5;  // Histéresis para evitar cambios de zona con fluctuaciones menores

const int FILTER_SIZE = 5;  // Tamaño del filtro para suavizar lecturas
int readings[FILTER_SIZE];  // Array para almacenar las lecturas
int readIndex = 0;          // Índice de la lectura actual
long total = 0;             // Suma de las lecturas
int average = 0;            // Promedio de las lecturas

// Variables for zone detection
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
  sensor.setMeasurementTimingBudget(100000);  // Incrementa el presupuesto a 100 ms para mayor precisión
  sensor.startContinuous(50);

  Serial.println("Esperando 5 segundos antes de la calibración...");
  Serial.println("Por favor, asegúrese de que no haya objetos frente al sensor");
  delay(SETUP_DELAY);

  // Calibración inicial
  long totalDistance = 0;
  Serial.println("Iniciando lecturas de calibración...");
  unsigned long startTime = millis();
  int sampleCount = 0;

  while (millis() - startTime < MEASUREMENT_PERIOD) {
    int reading = sensor.read();
    if (sensor.timeoutOccurred()) {
      continue;
    }
    totalDistance += reading;
    sampleCount++;
    if (sampleCount % (SAMPLES_FOR_BASELINE / 10) == 0) {
      Serial.print("Calibrando... ");
      Serial.print(sampleCount * 100 / SAMPLES_FOR_BASELINE);
      Serial.println("%");
    }
  }

  baselineDistance = totalDistance / sampleCount;
  Serial.println("Calibración completada.");
  Serial.print("Distancia base (sin objeto): ");
  Serial.println(baselineDistance);

  // Calcular zonas
  calculateZones();

  // Inicializar el array de lecturas
  for (int i = 0; i < FILTER_SIZE; i++) {
    readings[i] = baselineDistance;
    total += baselineDistance;
  }
}

void loop() {
  if (millis() - lastReadTime >= READ_INTERVAL) {
    lastReadTime = millis();

    int reading = sensor.read();
    if (sensor.timeoutOccurred()) {
      Serial.println("¡Tiempo de espera del sensor durante la lectura!");
      return;
    }

    // Actualizar el filtro de lecturas
    total = total - readings[readIndex];
    readings[readIndex] = reading;
    total = total + readings[readIndex];
    readIndex = (readIndex + 1) % FILTER_SIZE;
    average = total / FILTER_SIZE;

    Serial.print("Lectura cruda: ");
    Serial.print(reading);
    Serial.print(" | Lectura filtrada: ");
    Serial.println(average);

    // Calcular la diferencia respecto a la distancia base
    int distanceDifference = baselineDistance - average;

    Serial.print("Distancia base: ");
    Serial.println(baselineDistance);
    Serial.print("Diferencia de distancia: ");
    Serial.println(distanceDifference);
    Serial.print("Umbral de detección: ");
    Serial.println(OBJECT_DETECTION_THRESHOLD);

    // Verificar si el objeto está dentro del rango de detección con umbral
    if (distanceDifference > OBJECT_DETECTION_THRESHOLD && average > MIN_DISTANCE) {
      int zone = distanceDifference / zoneSize + 1;
      if (zone > NUM_ZONES) {
        zone = NUM_ZONES;
      }
      Serial.print("Zona calculada: ");
      Serial.println(zone);
      if (!objectPresent) {
        objectPresent = true;
        currentZone = zone;
        Serial.print("Objeto detectado en la Zona ");
        Serial.println(zone);
      } else if (zone != currentZone && abs(zone - currentZone) > HYSTERESIS) {
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