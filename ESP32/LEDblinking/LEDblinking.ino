#define LED_PIN 4  // External LED on GPIO4

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH); // LED ON
  delay(50);
  digitalWrite(LED_PIN, LOW);  // LED OFF
  delay(50);
}