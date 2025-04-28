#define BLYNK_AUTH_TOKEN "libqM3DsniS2Fd4R_XieQMpGfM4ZOBlx"
#define BLYNK_TEMPLATE_ID "TMPL3jXvEKOlp"
#define BLYNK_TEMPLATE_NAME "Fish Feeder"

#define V1 1  // Virtual pin for motion status
#define V2 2  // Virtual pin for ultrasonic distance
#define V6 6  // Virtual pin for temperature in Fahrenheit
#define V4 4  // Virtual pin for controlling servo position (slider)
#define V5 5  // Virtual pin for controlling water pump (button)
#define V7 7  // Virtual pin for turbidity level
#define V8 8  // Virtual pin for pH Value

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// LCD Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Temperature Sensor Setup
const int ONE_WIRE_BUS = 25;  // Pin for temperature sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float tempC;  // Celsius temperature variable

Servo myServo;

// WiFi credentials
const char* ssid = "Vivo";  // Replace with your WiFi name
const char* password = "12345678"; // Replace with your WiFi password

// Ultrasonic pins
const int trigPin = 12;  // GPIO 12 for Trigger
const int echoPin = 14;  // GPIO 14 for Echo

// Servo motor pin
const int servoPin = 26; // GPIO 26 for Servo Motor

// Motion sensor pin
const int sensorPin = 27; // GPIO 27 for Motion Sensor

// Water pump control pin
const int pumpPin = 33;  // GPIO 33 for Water Pump (Relay or Transistor)

// Turbidity sensor pin
const int turbidityPin = 34; // GPIO 34 (Analog pin for turbidity sensor)
const int pH_Sensorpin = 35;

// Variables for ultrasonic distance and motion detection
long duration;
float distanceCm;
float distanceInch;

unsigned long previousMillis = 0;
const long interval = 2000;  // 2 seconds for each parameter display
int displayState = 0;

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  pinMode(trigPin, OUTPUT);  // Sets trigPin as Output
  pinMode(echoPin, INPUT);   // Sets echoPin as Input
  pinMode(sensorPin, INPUT); // Motion sensor input pin
  pinMode(pumpPin, OUTPUT);  // Water pump pin (set to output)

  myServo.attach(servoPin);  // Attach the servo to the specified pin

  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();

  // Set ADC width (default is 12 bits, range 0-4095)
  analogReadResolution(12);  // Increase resolution to 12-bit (0-4095)
}

void loop() {
  // Blynk run method
  Blynk.run();

  // Motion sensor check
  long state = digitalRead(sensorPin);
  if (state == HIGH) {
    Blynk.virtualWrite(V1, HIGH);
    Serial.println("Motion detected!");
  } else {
    Blynk.virtualWrite(V1, LOW);
    Serial.println("Motion absent!");
  }

  // Ultrasonic distance measurement
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * 0.034 / 2;
  distanceInch = distanceCm * 0.393701;
  Blynk.virtualWrite(V2, distanceCm);  // Send distance to Blynk app
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);

  // Read and send temperature in Celsius
  sensors.requestTemperatures();
  tempC = sensors.getTempCByIndex(0);  // Get temperature in Celsius

  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data");
  } else {
    Serial.print("Temperature (C): ");
    Serial.print(tempC);
    Serial.println((char)176); // Show degrees symbol

    // Send Celsius temperature to Blynk Virtual Pin V6
    Blynk.virtualWrite(V6, tempC);
  }

  // Turbidity evaluation
  int rawValue = analogRead(turbidityPin);  // Read raw sensor value
  float voltage = rawValue * (3.3 / 4095.0);  // Convert raw value to voltage
  int turbidity = map(rawValue, 0, 4095, 0, 3000);  // Map raw value to turbidity (0 to 3000)

  Blynk.virtualWrite(V7, turbidity);  // Send turbidity data to Blynk app

  Serial.print("Turbidity (NTU): ");
  Serial.println(turbidity);  // Print turbidity value

 int pH_Value = analogRead(pH_Sensorpin); 
 int Voltage = pH_Value * (3.3 / 4095.0); 
  Serial.println(Voltage); 
  Serial.println(pH_Value);
  delay(500); 

  Blynk.virtualWrite(V8, pH_Value);  // Send pH Value data to Blynk app
  Serial.print("pH : ");
  Serial.println(pH_Value);  // Print pH value


  // Update LCD display every interval
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    displayState = (displayState + 1) % 5; // Cycle through 5 display states
  }

  // Display the current parameter based on displayState
  lcd.clear();
  switch (displayState) {
    case 0:
      // Display motion status
      lcd.setCursor(0, 0);
      lcd.print("Motion: ");
      if (state == HIGH) {
        lcd.print("Detected");
      } else {
        lcd.print("Absent");
      }
      break;

    case 1:
      // Display distance
      lcd.setCursor(0, 0);
      lcd.print("Distance: ");
      lcd.print(distanceCm);
      lcd.print(" cm");
      break;

    case 2:
      // Display temperature
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(tempC);
      break;

    case 3:
      // Display turbidity
      lcd.setCursor(0, 0);
      lcd.print("Turbidity: ");
      lcd.print(turbidity);
      // Display water condition based on voltage
      lcd.setCursor(0, 1);
      lcd.print("Water: ");
      if (voltage >= 3.0) {
        lcd.print("CLEAN");
      } else {
        lcd.print("NOT CLEAN");
      }
      break;

    case 4:
      // Display turbidity
      lcd.setCursor(0, 0);
      lcd.print("pH Value: ");
      lcd.print(pH_Value);
      // Display water condition based on voltage
      lcd.setCursor(0, 1);
      lcd.print("pH: ");
      if (pH_Value >= 5.5 && pH_Value >= 8) {
        lcd.print("In Range");
      } else {
        lcd.print("Out of Range");
      }
      break;

  }

  delay(1000);  // Delay for stability
}

// Blynk virtualWrite for servo control (Slider control on V4)
BLYNK_WRITE(V4) {
  int servoPos = param.asInt();  // Get value from slider (0 to 180)
  myServo.write(servoPos);  // Set the servo position
  Serial.print("Servo Position: ");
  Serial.println(servoPos);
}

// Blynk virtualWrite for water pump control (Button control on V5)
BLYNK_WRITE(V5) {
  int pumpState = param.asInt();  // Get button state (0 or 1)
  if (pumpState == 1) {
    digitalWrite(pumpPin, HIGH);  // Turn on the pump
    Serial.println("Water Pump ON");
  } else {
    digitalWrite(pumpPin, LOW);   // Turn off the pump
    Serial.println("Water Pump OFF");
  }
}