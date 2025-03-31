#include <Wire.h>
#include <OPT3101.h>
#include <ESP32Servo.h>

#define SERVO_PIN       9    // Servopinne
#define MOTOR_SPEED_PIN A1   // PWM-pin för motorhastighet (enable-ingång, M1A)
#define MOTOR_DIR_PIN   B1   // Digital utgång för motorriktning (M1B)

Servo myservo;             // Använd Servo-typen från ESP32Servo-biblioteket
OPT3101 sensor;

// --- Servo-inställningar ---
const int servoStraight = 90;    // Centervinkel när bilen är centrerad
const int servoLeftMax  = 135;    // Max vänstersväng (om hinder på höger)
const int servoRightMax = 45;     // Max högersväng (om hinder på vänster)
int servoPosition = servoStraight;

// --- Sensorvariabler (i mm) ---
// Kanal 0: Höger, Kanal 1: Fram, Kanal 2: Vänster
int sensorRight = 800;
int sensorFront = 800;
int sensorLeft  = 800;

// --- Tröskelvärden för mapping (justera efter behov) ---
const int Safe    = 400;   // Övre gräns för "säkert" avstånd
const int NotSafe = 300;   // Nedre gräns för att trigga sväng

// --- State machine för körning ---
enum DriveState { FORWARD, STOPPED, REVERSING };
DriveState driveState = FORWARD;
unsigned long stateChangeTime = 0;
const unsigned long stopDuration = 1000; // 2 sekunder i stoppat läge innan backning

// --- Motorinställningar ---
int motorSpeedVal = 200;  // PWM-värde (0-255); justera efter behov

void setup() {
  Serial.begin(115200);
  
  Serial.println("🔄 Startar systemet...");

  // Initiera I2C (om din Nano ESP32 använder A4 och A5 som SDA/SCL)
  Wire.begin(A4, A5);

  // Initiera OPT3101-sensorn
  sensor.init();
  if (sensor.getLastError()) {
    Serial.print("❌ Sensorfel: ");
    Serial.println(sensor.getLastError());
    while (1) {}
  }
  sensor.configureDefault();
  sensor.setFrameTiming(32);
  sensor.setBrightness(OPT3101Brightness::High);
  sensor.enableTimingGenerator();
  sensor.setChannel(0);
  sensor.startSample();

  // Initiera servot
  myservo.attach(SERVO_PIN);
  myservo.write(servoStraight);

  // Initiera motorpinnar
  pinMode(MOTOR_SPEED_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);

  // Starta motorn i framåtriktning: (fram = digital LOW)
  digitalWrite(MOTOR_DIR_PIN, LOW);
  analogWrite(MOTOR_SPEED_PIN, motorSpeedVal);

  Serial.println("✅ System klart!");
}

void loop() {
  unsigned long currentTime = millis();

  if (sensor.isSampleDone()) {
    sensor.readOutputRegs();
    int distance = sensor.distanceMillimeters;
    int channel = sensor.channelUsed;

    // Hoppa över ogiltiga mätningar
    if (distance == 0) {
      sensor.startSample();
      return;
    }

    // Uppdatera sensorvariabler
    if (channel == 0) {
      sensorRight = distance;
    } else if (channel == 1) {
      sensorFront = distance;
    } else if (channel == 2) {
      sensorLeft = distance;
    }

    // Debugutskrift
    Serial.print("Kanal ");
    Serial.print(channel);
    Serial.print(": ");
    Serial.print(distance);
    Serial.println(" mm");

    // Växla till nästa kanal (0 -> 1 -> 2 -> 0)
    if (channel == 0)
      sensor.setChannel(1);
    else if (channel == 1)
      sensor.setChannel(2);
    else
      sensor.setChannel(0);
    sensor.startSample();

    // --- State Machine ---
    if (driveState == FORWARD) {
      // Om frontavståndet är under 50 mm, stanna bilen
      if (sensorFront < 50) {
        Serial.println("🚨 Hinder FRAMFÖR! Stoppar bilen.");
        driveState = STOPPED;
        stateChangeTime = currentTime;
        digitalWrite(MOTOR_DIR_PIN, LOW);
        analogWrite(MOTOR_SPEED_PIN, 0);
      }
      else {
        int newServo = servoStraight;  // Standard: rakt fram

        // Om det finns ett hinder på vänster sida (lägre sensorvärde) än på höger, sväng höger
        if (sensorLeft < Safe && sensorLeft < sensorRight) {
          // Mappa från avståndet (NotSafe till Safe) till servo-vinkel (servoRightMax till servoStraight)
          newServo = map(sensorLeft, NotSafe, Safe, servoRightMax, servoStraight);
          Serial.println("🔄 Hinder på vänster sida, svänger höger.");
        }
        // Om det finns ett hinder på höger sida än på vänster, sväng vänster
        else if (sensorRight < Safe && sensorRight < sensorLeft) {
          // Mappa från avståndet (NotSafe till Safe) till servo-vinkel (servoLeftMax till servoStraight)
          newServo = map(sensorRight, NotSafe, Safe, servoLeftMax, servoStraight);
          Serial.println("🔄 Hinder på höger sida, svänger vänster.");
        }
        else {
          // Annars använd proportionell styrning med felberäkning (sensorLeft - sensorRight)
          int error = sensorLeft - sensorRight;
          if (abs(error) < 30) error = 0;  // Deadband
          float Kp = 0.10;
          newServo = servoStraight + (int)(Kp * error);
        }

        // Se till att servovinkeln håller sig inom gränserna
        if (newServo < servoRightMax) newServo = servoRightMax;
        if (newServo > servoLeftMax) newServo = servoLeftMax;

        if (servoPosition != newServo) {
          servoPosition = newServo;
          myservo.write(servoPosition);
          Serial.print("Servo justerat till: ");
          Serial.println(servoPosition);
        }
        // Kör framåt: sätt motorriktning till LOW (fram)
        digitalWrite(MOTOR_DIR_PIN, LOW);
        analogWrite(MOTOR_SPEED_PIN, motorSpeedVal);
      }
    }
    else if (driveState == STOPPED) {
      // Vänta 2 sekunder innan vi börjar backa
      if (currentTime - stateChangeTime >= stopDuration) {
        Serial.println("➡️ 2 sekunder passerade. Börjar backa.");
        driveState = REVERSING;
        servoPosition = servoStraight;
        myservo.write(servoPosition);
        // För backning: sätt motorriktning till HIGH (back)
        digitalWrite(MOTOR_DIR_PIN, HIGH);
        analogWrite(MOTOR_SPEED_PIN, motorSpeedVal);
      }
    }
    else if (driveState == REVERSING) {
      // Om frontavståndet ökar (över 100 mm) återgå till FORWARD
      if (sensorFront > 100) {
        Serial.println("✅ Hinder borta, återgår till framåtkörning.");
        driveState = FORWARD;
        digitalWrite(MOTOR_DIR_PIN, LOW);
        analogWrite(MOTOR_SPEED_PIN, motorSpeedVal);
      }
      // Under backning håll servot rakt
      myservo.write(servoStraight);
    }
  }
}