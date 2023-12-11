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
# 
