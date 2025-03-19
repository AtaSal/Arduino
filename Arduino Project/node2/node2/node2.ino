#include <DHT.h>
#include <ArduinoJson.h>
#include <Servo.h>

#define DHTPIN 8
#define DHTTYPE DHT11
#define SWITCH_START 23
#define SWITCH_DIRECTION 22
#define POTENTIAL_SPEED_PIN A4
#define SERVOMOTOR_PIN 9
#define MOTOR_IN1 7
#define MOTOR_IN2 6
#define MOTOR_PWM 5

Servo servo2;
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  dht.begin();
  pinMode(SWITCH_START, INPUT_PULLUP);
  pinMode(SWITCH_DIRECTION, INPUT_PULLUP);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  servo2.attach(SERVOMOTOR_PIN);
}

void loop() {
 
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  bool startState = digitalRead(SWITCH_START);
  bool dirState = digitalRead(SWITCH_DIRECTION);
  int potSpeed = analogRead(POTENTIAL_SPEED_PIN);
  int motorSpeed = map(potSpeed, 0, 1023, 0, 255);

  
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("DHT sensor read failed!");
    return;
  }


  sendJsonData(temperature, humidity, startState, dirState, motorSpeed);

  
  if (Serial1.available()) {
    processReceivedData();
  }

  delay(150);
}

void sendJsonData(float temp, float hum, bool start, bool dir, int speed) {
  JsonDocument doc;
  doc["node"] = 2;
  doc["temperature"] = temp;
  doc["humidity"] = hum;
  doc["start"] = start;
  doc["dir"] = dir;
  doc["speed"] = speed;

  char jsonOutput[2048];
  serializeJson(doc, jsonOutput);
  Serial1.println(jsonOutput);
  Serial.print("Node 2 Sent: ");
  Serial.println(jsonOutput);
}

void processReceivedData() {
  String receivedData = Serial1.readStringUntil('\n');
  JsonDocument docReceived;
  DeserializationError error = deserializeJson(docReceived, receivedData);

  if (error) {
    Serial.print("Node 1 Parse Error: ");
    Serial.println(error.c_str());
  } else {
    int servoAngle = docReceived["servo_servoAngle"];  
    int ldrValue = docReceived["ldr"];

    
    servo2.write(servoAngle);

    
    bool motorStart = docReceived["start"];
    bool motorDir = docReceived["dir"];
    int motorSpeed = docReceived["speed"];
    controlMotor(motorStart, motorDir, motorSpeed);

    Serial.print("Node 1 Data: Angle=");
    Serial.print(servoAngle);
    Serial.print(", LDR=");
    Serial.print(ldrValue);
    Serial.print(", Motor Start=");
    Serial.print(motorStart);
    Serial.print(", Motor Dir=");
    Serial.print(motorDir);
    Serial.print(", Motor Speed=");
    Serial.println(motorSpeed);
  }
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
