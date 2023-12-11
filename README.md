# 1. Detecting metallic and nonmetallic object in Arduino
# 2. Sound sensor
# 3. Temperature 27 and 25 
# 4. Sliding door of Shoping mall
# 5. Sliding door using RFID tag
# 6. Wavy moving robot using onstacle sensing
# 7. Automatic room light using LDR
# 8. pH value of orange juice if less than 6.7 add neutralizing agent till the ph reduce to 5.7
# 9.
# 10.
# 11.
#
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
