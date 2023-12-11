# 1. Detecting metallic and nonmetallic object in Arduino
# 2. Sound sensor
# 3. Temperature 27 and 25 
# 4. Sliding door of Shoping mall
# 5. Sliding door using RFID tag
# 6. Wavy moving robot using onstacle sensing
# 7. Automatic room light using LDR
# 8. pH value of orange juice if less than 6.7 add neutralizing agent till the ph reduce to 5.7
# 9. Display the room temperature on your mobile
# 10. Parking Automation
# 11. Automatic sump and overhead tank 
# 12. Temperature sensor interfacing 
#
#

# 1.	Detecting metallic and nonmetallic object in Arduino
const int sensorPin = 2;
const int buzzerPin = 3;

void setup() {
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  int sensorValue = digitalRead(sensorPin);

  if (sensorValue == HIGH) {
    Serial.println("Metallic object detected!");
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
  } else {
    Serial.println("Non-metallic object detected!");
  }

  delay(500);
}

------------------------------------------------------------------------------------------

#  2.Sound sensor
const int soundSensorPin = A0;
const int buzzerPin = 9;

void setup() {
  pinMode(soundSensorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int soundValue = analogRead(soundSensorPin);
  int threshold = 500;

  if (soundValue > threshold) {
    digitalWrite(buzzerPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
  }

  Serial.print("Sound Value: ");
  Serial.println(soundValue);

  delay(100);
}

-------------------------------------------------------------------------------------------------------
# 3.Temperature 27 and 25 
const int lm35Pin = A0;
const int ledPin = 13;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  int sensorValue = analogRead(lm35Pin);
  float temperature = (sensorValue * 5.0 / 1023.0) * 100.0;
  Serial.print("Temperature: ");
  Serial.print(temperature);
  
Serial.println(" °C");

  if (temperature > 25.0 && temperature < 27.0) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  delay(1000);
}

---------------------------------------------------------------------------------------------------

# 4. Sliding door of Shoping mall
#include <Servo.h>

const int trigPin = 7;
const int echoPin = 6;
const int servoPin = 9;

Servo myServo;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myServo.attach(servoPin);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < 20) {
    rotateServo();
  }

  delay(1000);
}

void rotateServo() {
  myServo.write(0);
  delay(2000);
  myServo.write(180);
  delay(1000);
}

---------------------------------------------------------------------------------------------------
# 5.Sliding door using RFID tag
#include <Servo.h>

#define RFID_D1_PIN 2   // Replace with the actual pin connected to D1 on the RFID module
#define RFID_D2_PIN 3   // Replace with the actual pin connected to D2 on the RFID module
#define RFID_OUT_PIN 4  // Replace with the actual pin connected to OUT on the RFID module

Servo myServo;
bool cardDetected = false;

void setup() {
  Serial.begin(9600);
  myServo.attach(6);

  pinMode(RFID_D1_PIN, INPUT);
  pinMode(RFID_D2_PIN, OUTPUT);
  pinMode(RFID_OUT_PIN, INPUT);
}

void loop() {
  if (digitalRead(RFID_OUT_PIN) == HIGH) {
    if (!cardDetected) {
      Serial.println("RFID card detected!");
      rotateServo();
      cardDetected = true;
    }
  } else {
    cardDetected = false;
  }
}

void rotateServo() {
  myServo.write(0);
  delay(2000);
  myServo.write(180);
  delay(2000);
  myServo.write(0);
}

---------------------------------------------------------------------------------------------------

# 6.	Wavy moving robot using onstacle sensing
const int irSensorPin = 2;
const int ledPin = 13;

void setup() {
  pinMode(irSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (digitalRead(irSensorPin) == LOW) {
    digitalWrite(ledPin, HIGH);
    Serial.println("IR sensor detected something - LED on");
  } else {
    digitalWrite(ledPin, LOW);
  }
}

---------------------------------------------------------------------------------------------------

# 7. Automatic room light using LDR
  const int ldrPin = A0;
const int ledPin = 13;

void setup() {
  pinMode(ldrPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}
void loop() {
  int ldrValue = analogRead(ldrPin);
  int threshold = 500;

  if (ldrValue > threshold) {
    digitalWrite(ledPin, LOW);
  } else {
    digitalWrite(ledPin, HIGH);
  }
  delay(500);
}

---------------------------------------------------------------------------------------------------

# 8. pH value of orange juice if less than 6.7 add neutralizing agent till the ph reduce to 5.7
const int sensor = A0;
const int ledPin = 13;
void setup() {
  Serial.begin(9600);
  pinMode(sensor, INPUT);
  pinMode(ledPin, OUTPUT);
}
void loop() {
  int analogValue = analogRead(sensor);
  int mappedValue = map(analogValue, 0, 1023, 0, 14);
  Serial.println(mappedValue);
 if (mappedValue < 6.7) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
  if (mappedValue <= 4.7) {
    // Additional actions or code can be added here
  }
 delay(1000);
}

---------------------------------------------------------------------------------------------------
# 9. Display the room temperature on your mobile
#include <SoftwareSerial.h>

const int lm35Pin = A0;
const int bluetoothTx = 1; 
const int bluetoothRx = 0; 

SoftwareSerial bluetoothSerial(bluetoothTx, bluetoothRx);

void setup() {
  Serial.begin(9600);
  Serial.println("Serial communication initialized");
  
  bluetoothSerial.begin(9600);
  Serial.println("Bluetooth communication initialized");
}

void loop() {
  int sensorValue = analogRead(lm35Pin);
  float temperature = (sensorValue * 5.0 / 1023.0) * 100.0;

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  bluetoothSerial.print("Temperature: ");
  bluetoothSerial.print(temperature);
  bluetoothSerial.println(" °C");

  delay(1000);
}

---------------------------------------------------------------------------------------------------

# 10. Parking Automation

#include <Servo.h>
#include <LiquidCrystal.h>

const int ledPins[] = {6, 10, 11, 12, 13, A1, A2, A3};
const int irSensorPin = A0;
const int exitButtonPin = 7; // Connect the push button to this digital pin
const int servoPin = 9;
const int rs = 0, en = 1, d4 = 2, d5 = 3, d6 = 4, d7 = 5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

Servo gateServo;
int vehicleCount = 0; // Start counting from 0
bool gateOpen = false;

const int irThresholdHigh = 600; // Adjust according to your setup
const int irThresholdLow = 400;  // Adjust according to your setup

void setup() {
  for (int i = 0; i < 8; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  gateServo.attach(servoPin);
  gateServo.write(0);

  lcd.begin(16, 2);
  lcd.print("Vehicle Count: 0"); // Start with count 0
  pinMode(exitButtonPin, INPUT_PULLUP); // Use INPUT_PULLUP to enable internal pull-up resistor
}

void loop() {
  int irValue = analogRead(irSensorPin);

  if (vehicleCount < 8) {
    if (irValue < irThresholdLow && !gateOpen) {
      gateServo.write(90);
      delay(2000);
      gateServo.write(0);
      gateOpen = true;

      digitalWrite(ledPins[vehicleCount], HIGH);

      vehicleCount++;
      updateDisplay();
    }
    else if (irValue >= irThresholdHigh && gateOpen) {
      gateOpen = false;
    }
  }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Parking Slots");

    if (gateOpen) {
      gateServo.write(0);
      gateOpen = false;
    }
  }

  // Check the exit button state
  if (digitalRead(exitButtonPin) == LOW && vehicleCount > 0) {
    gateServo.write(90);
    delay(2000);
    gateServo.write(0);
    gateOpen = true;

    digitalWrite(ledPins[vehicleCount - 1], LOW);
    vehicleCount--;
    updateDisplay();
  }
}

void updateDisplay() {
  lcd.setCursor(14, 0); // Position the cursor to update vehicle count
  lcd.print("   "); // Clear the previous count
  lcd.setCursor(14, 0);
  lcd.print(vehicleCount); // Display the updated count
}

---------------------------------------------------------------------------------------------------

# 11. Automatic sump and overhead tank 

const int lowLevelButtonPin = 2;       // Button to detect low water level
const int highLevelButtonPin = 3;      // Button to detect high water level
const int additionalButton1Pin = 4;    // Additional button 1
const int additionalButton2Pin = 5;    // Additional button 2
const int motorPin = 8;                 // Motor (buzzer) pin
const int lowLevelLedPin = 9;          // LED for low-level button
const int additionalLedPin = 10;       // LED for additional buttons

int lowLevelStatus = 0; // Variable to store low-level status (0 - off, 1 - on)
int additionalStatus = 0; // Variable to store additional buttons status (0 - off, 1 - on)

void setup() {
  pinMode(lowLevelButtonPin, INPUT);
  pinMode(highLevelButtonPin, INPUT);
  pinMode(additionalButton1Pin, INPUT);
  pinMode(additionalButton2Pin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(lowLevelLedPin, OUTPUT);
  pinMode(additionalLedPin, OUTPUT);

  // Initialize Serial communication if needed
  // Serial.begin(9600);
}

void loop() {
  // Check if the low-level button is pressed
  if (digitalRead(lowLevelButtonPin) == LOW && lowLevelStatus == 0) {
    lowLevelStatus = 1; // Set low-level status to on
    digitalWrite(motorPin, HIGH); // Turn on motor (buzzer)
    blinkLed(lowLevelLedPin); // Blink LED for low-level button
  }

  // Check if the high-level button is pressed to turn off low-level status
  if (digitalRead(highLevelButtonPin) == LOW && lowLevelStatus == 1) {
    lowLevelStatus = 0; // Set low-level status to off
    digitalWrite(motorPin, LOW); // Turn off motor (buzzer)
    digitalWrite(lowLevelLedPin, LOW); // Turn off LED for low-level button
  }

  // Check if additional button 1 is pressed
  if (digitalRead(additionalButton1Pin) == LOW && additionalStatus == 0) {
    additionalStatus = 1; // Set additional status to on
    digitalWrite(motorPin, HIGH); // Turn on motor (buzzer)
    blinkLed(additionalLedPin); // Blink LED for additional buttons
  }

  // Check if additional button 2 is pressed to turn off additional status
  if (digitalRead(additionalButton2Pin) == LOW && additionalStatus == 1) {
    additionalStatus = 0; // Set additional status to off
    digitalWrite(motorPin, LOW); // Turn off motor (buzzer)
    digitalWrite(additionalLedPin, LOW); // Turn off LED for additional buttons
  }

  // Print motor status for debugging if needed
  // Serial.print("Low Level Status: ");
  // Serial.print(lowLevelStatus);
  // Serial.print(" | Additional Status: ");
  // Serial.println(additionalStatus);
}

void blinkLed(int ledPin) {
  // Function to blink the LED
  for (int i = 0; i < 5; i++) {
    digitalWrite(ledPin, HIGH); // Turn on LED
    delay(500); // Blinking delay
    digitalWrite(ledPin, LOW); // Turn off LED
    delay(500); // Blinking delay
  }
}

---------------------------------------------------------------------------------------------------

# 11. Temperature sensor interfacing 
const int analogPin = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(analogPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float temperatureCelsius = (voltage - 0.5) * 100.0;

  Serial.print("Sensor Value: ");
  Serial.println(sensorValue);
  
  Serial.print("Temperature (Celsius): ");
  Serial.println(temperatureCelsius);

  delay(1000);
}


