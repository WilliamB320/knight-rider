#include <Wire.h>
#include <ESP32Servo.h>    // Use the ESP32-compatible servo library
#include <OPT3101.h>       // Sensor library

// Pin definitions
#define servoPin 9        // Servo control pin
#define enA 8             // (Optional) Enable pin for motor driver (if used)
#define in1 A1         // Motor driver input 1 (MDD3A)
#define in2 A2            // Motor driver input 2 (MDD3A)
#define remotePin 4       // Pin for start module (remote)
const uint8_t dataReadyPin = A6;  // Pin for sensor data ready interrupt

// Servo steering angles
int Straight = 58;       // Center (straight) position
int TurnAngle = 17;      // How much the servo should deviate for turns
int TurnLeft = Straight - TurnAngle;
int TurnRight = Straight + TurnAngle;

// Distance thresholds (in millimeters)
int Near = 400;
int NotSafe = 600;
int Safe = 800;

// Global variables
int remoteVal = 0;       // Value read from the remote start module
int leftsensor = 0;
int middlesensor = 1;
int rightsensor = 2;
int angle = 90;          // Variable for calculated servo angle

OPT3101 sensor;          // Create sensor object
int16_t distances[3];    // Array to store sensor readings from three channels
volatile bool dataReady = false;  // Flag set by the data-ready interrupt

// Interrupt service routine for sensor data ready signal
void IRAM_ATTR setDataReadyFlag() {
  dataReady = true;
}

// Create the servo object using ESP32Servo
Servo myservo;

void setup() {
  Serial.begin(9600);
  // Wait for serial port connection (useful for debugging)
  while (!Serial) {}

  // Initialize and attach the servo; set it to the straight (center) position
  myservo.attach(servoPin);
  myservo.write(Straight);

  // Initialize I²C
  Wire.begin(A4,A5,A6);

  // Setup motor control pins
  pinMode(remotePin, INPUT);
  pinMode(enA, INPUT);  // If your MDD3A needs an enable pin, adjust as required
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Initialize the sensor
  sensor.init();
  if (sensor.getLastError()) {
    Serial.print(F("Failed to initialize OPT3101: error "));
    Serial.println(sensor.getLastError());
    while (1) {}  // Halt if sensor initialization fails
  }
  
  // Configure sensor settings
  sensor.setContinuousMode();
  sensor.enableDataReadyOutput(1);
  sensor.setFrameTiming(32);
  sensor.setChannel(OPT3101ChannelAutoSwitch);
  sensor.setBrightness(OPT3101Brightness::Adaptive);
  
  // Attach the data-ready interrupt for the sensor
  attachInterrupt(digitalPinToInterrupt(dataReadyPin), setDataReadyFlag, RISING);
  
  sensor.enableTimingGenerator();
}

void loop() {
  // Update sensor readings from the global distances array
  leftsensor = distances[0];
  middlesensor = distances[1];
  rightsensor = distances[2];

  // Read the start module (remote) value
  remoteVal = digitalRead(remotePin);

  if (remoteVal == HIGH) {
    // If the remote is active, run the motor forward.
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    // Process sensor reading if a sample is complete
    if (sensor.isSampleDone()) {
      sensor.readOutputRegs();
      // Save the current channel's distance measurement
      distances[sensor.channelUsed] = sensor.distanceMillimeters;
      
      // When channel 2 (for example) is done, adjust steering based on sensor data.
      if (sensor.channelUsed == 2) {
        // Optionally loop to smooth out or re-check sensor data (this loop runs three times).
        for (uint8_t i = 0; i < 3; i++) {
          // Adjust steering based on sensor comparisons.
          // If the left sensor reading is lower (i.e. an obstacle is closer) than the right sensor...
          if (leftsensor < Safe && leftsensor < rightsensor) {
            angle = map(leftsensor, Safe, NotSafe, Straight, TurnRight);
            myservo.write(angle);
          }
          // If the right sensor reading is lower than the left sensor...
          else if (rightsensor < Safe && rightsensor < leftsensor) {
            angle = map(rightsensor, Safe, NotSafe, Straight, TurnLeft);
            myservo.write(angle);
          }
          // If the middle sensor reading is below the NotSafe threshold and left is less than right...
          else if (middlesensor < NotSafe && leftsensor < rightsensor) {
            angle = map(rightsensor, Safe, Near, Straight, TurnRight);
            myservo.write(angle);
          }
          // If the middle sensor reading is below the NotSafe threshold and right is less than left...
          else if (middlesensor < NotSafe && rightsensor < leftsensor) {
            angle = map(rightsensor, Safe, Near, Straight, TurnLeft);
            myservo.write(angle);
          }
          else {
            // Otherwise, keep steering straight.
            myservo.write(Straight);
          }
        }
      }
      // Move to the next sensor channel and start a new sample
      sensor.nextChannel();
      sensor.startSample();
    }
  }
  else if (remoteVal == LOW) {
    // If the remote is inactive, stop the motor.
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
