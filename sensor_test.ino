#include <Wire.h>
#include <OPT3101.h>

// Skapa ett sensorobjekt
OPT3101 sensor;

void setup() {
  // Starta seriell kommunikation för debug-utskrifter
  Serial.begin(115200);

  // Initiera I²C på Arduino Nano ESP32 med A4 (SDA) och A5 (SCL)
  Wire.begin(A4, A5);

  // Initiera sensorn och konfigurera den med standardinställningar
  sensor.init();
  sensor.configureDefault();

  // Om du behöver ändra I²C-adressen (standard är 0x58) kan du använda:
  // sensor.setAddress(0x58);
}

void loop() {
  // Starta en mätning och hämta resultatet
  sensor.sample();

  // Skriv ut vilken kanal som användes, avståndet (i millimeter) och amplituden
  Serial.print("Kanal: ");
  Serial.print(sensor.channelUsed);
  Serial.print(" | Avstånd: ");
  Serial.print(sensor.distanceMillimeters);
  Serial.print(" mm");
  Serial.print(" | Amplitud: ");
  Serial.println(sensor.amplitude);

  // Kort paus mellan mätningarna
  delay(1000);
}
