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

# Sound sensor
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
# Temperature 27 and 25 
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

# Sliding door of Shoping mall
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

#




