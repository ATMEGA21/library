# Title 

 1. Detecting metallic and nonmetallic object in Arduino
 2. Sound sensor
 3. Temperature 27 and 25 
 4. Sliding door of Shoping mall
 5. Sliding door using RFID tag
 6. Wavy moving robot using onstacle sensing
 7. Automatic room light using LDR
 8. pH value of orange juice if less than 6.7 add neutralizing agent till the ph reduce to 5.7
 9. Display the room temperature on your mobile
 10. Parking Automation
 11. Automatic sump and overhead tank 
 12. Temperature sensor interfacing
 13. LDR
 14. POT
 15. SCRIPT 






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
const int exitButtonPin = 7;
const int servoPin = 9;
const int rs = 0, en = 1, d4 = 2, d5 = 3, d6 = 4, d7 = 5;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

Servo gateServo;
int vehicleCount = 0;
bool gateOpen = false;

const int irThresholdHigh = 600;
const int irThresholdLow = 400;

void setup() {
  for (int i = 0; i < 8; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  gateServo.attach(servoPin);
  gateServo.write(0);

  lcd.begin(16, 2);
  lcd.print("Vehicle Count: 0");
  pinMode(exitButtonPin, INPUT_PULLUP);
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
    } else if (irValue >= irThresholdHigh && gateOpen) {
      gateOpen = false;
    }
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Parking Slots");

    if (gateOpen) {
      gateServo.write(0);
      gateOpen = false;
    }
  }

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
  lcd.setCursor(14, 0);
  lcd.print(" ");
  lcd.setCursor(14, 0);
  lcd.print(vehicleCount);
}


---------------------------------------------------------------------------------------------------

# 11. Automatic sump and overhead tank 

const int lowLevelButtonPin = 2;
const int highLevelButtonPin = 3;
const int additionalButton1Pin = 4;
const int additionalButton2Pin = 5;
const int motorPin = 8;
const int lowLevelLedPin = 9;
const int additionalLedPin = 10;

int lowLevelStatus = 0;
int additionalStatus = 0;

void setup() {
  pinMode(lowLevelButtonPin, INPUT);
  pinMode(highLevelButtonPin, INPUT);
  pinMode(additionalButton1Pin, INPUT);
  pinMode(additionalButton2Pin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(lowLevelLedPin, OUTPUT);
  pinMode(additionalLedPin, OUTPUT);
}

void loop() {
  if (digitalRead(lowLevelButtonPin) == LOW && lowLevelStatus == 0) {
    lowLevelStatus = 1;
    digitalWrite(motorPin, HIGH);
    blinkLed(lowLevelLedPin);
  }

  if (digitalRead(highLevelButtonPin) == LOW && lowLevelStatus == 1) {
    lowLevelStatus = 0;
    digitalWrite(motorPin, LOW);
    digitalWrite(lowLevelLedPin, LOW);
  }

  if (digitalRead(additionalButton1Pin) == LOW && additionalStatus == 0) {
    additionalStatus = 1;
    digitalWrite(motorPin, HIGH);
    blinkLed(additionalLedPin);
  }

  if (digitalRead(additionalButton2Pin) == LOW && additionalStatus == 1) {
    additionalStatus = 0;
    digitalWrite(motorPin, LOW);
    digitalWrite(additionalLedPin, LOW);
  }
}

void blinkLed(int ledPin) {
  for (int i = 0; i < 5; i++) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
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

---------------------------------------------------------------------------------------------------


# 12 LDR 
const int ldrPin = A0;
const int ledPin = 13;

void setup() {
  pinMode(ldrPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  int ldrValue = analogRead(ldrPin);

  if (ldrValue < 500) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  delay(100);
}

---------------------------------------------------------------------------------------------------

# 13. POT to voltage values
const int potPin = A0;
const float referenceVoltage = 5.0;

void setup() {
  pinMode(potPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(potPin);

  float voltage = potValue * (referenceVoltage / 1023.0);

  Serial.println(voltage, 3);

  delay(100);
}

---------------------------------------------------------------------------------------------------

# 14. SCRIPT 
[w:GD200] =[w:GD200]+1;

if([w:GD200] ++50){

[w:GD200] =0;
}
