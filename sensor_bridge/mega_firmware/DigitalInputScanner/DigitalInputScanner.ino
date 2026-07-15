const int sensorPin = 22;

void setup() {
  Serial.begin(115200);
  pinMode(sensorPin, INPUT_PULLUP);
}

void loop() {
  int sensorState = digitalRead(sensorPin);

  Serial.print("D22 raw = ");
  Serial.println(sensorState == HIGH ? "HIGH" : "LOW");

  delay(100);
}