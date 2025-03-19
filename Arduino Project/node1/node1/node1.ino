
#include <DHT.h>
#include <ArduinoJson.h>
#include <Servo.h>

#define DHTPIN 21
#define DHTTYPE DHT11
#define SERVOMOTOR_PIN 9
#define LDR_PIN A0
#define POTENTIAL_PIN A1
#define MOTOR_IN1 7
#define MOTOR_IN2 6
#define MOTOR_PWM 5

Servo servo1;
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  dht.begin();
  servo1.attach(SERVOMOTOR_PIN);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
}

void loop() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int potentialValue = analogRead(POTENTIAL_PIN);
  int servoAngle = map(potentialValue, 0, 1023, 0, 180);
  int ldrValue = analogRead(LDR_PIN);

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("DHT sensor read failed!");
    return;
  }

  JsonDocument doc;
  doc["node"] = 1;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["servo_servoAngle"] = servoAngle;
  doc["ldr"] = ldrValue;

  servo1.write(servoAngle);
  char jsonOutput[2048];
  serializeJson(doc, jsonOutput);
  Serial1.println(jsonOutput);
  Serial.print("Node 1 Sent: ");
  Serial.println(jsonOutput);
  
  if (Serial1.available()) {
    String receivedData = Serial1.readStringUntil('\n');
    JsonDocument docReceived;
    DeserializationError error = deserializeJson(docReceived, receivedData);
    if (error) {
      Serial.print("Node 2 Parse Error: ");
      Serial.println(error.c_str());
    } else {
      int nodeID = docReceived["node"];
      float tempRecv = docReceived["temperature"];
      float humidRecv = docReceived["humidity"];
      bool motorStart = docReceived["start"];
      bool motorDir = docReceived["dir"];
      int motorSpeed = docReceived["speed"];

      controlMotor(motorStart, motorDir, motorSpeed);  

      Serial.print("Node 2 Data: ");
      Serial.print("Start="); Serial.print(motorStart);
      Serial.print(", Dir="); Serial.print(motorDir);
      Serial.print(", Speed="); Serial.println(motorSpeed);
    }
  }

  delay(200);
}


void controlMotor(bool motorStart, bool motorDir, int motorSpeed) {
  if (motorStart) {
    digitalWrite(MOTOR_IN1, motorDir ? HIGH : LOW);
    digitalWrite(MOTOR_IN2, motorDir ? LOW : HIGH);
    analogWrite(MOTOR_PWM, motorSpeed);
  } else {
    stopMotor();  
  }
}


void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
  delay(100);  
}
