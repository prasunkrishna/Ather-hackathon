// Simple ESP32 Test Code

int ledPin = 2; // Most ESP32 boards have built-in LED on GPIO 2

void setup() {
  Serial.begin(115200);           // Start Serial Monitor
  pinMode(ledPin, OUTPUT);        // Set LED pin as output
  Serial.println("ESP32 is Running!");
}

void loop() {
  digitalWrite(ledPin, HIGH);     // LED ON
  Serial.println("LED ON");
  delay(1000);

  digitalWrite(ledPin, LOW);      // LED OFF
  Serial.println("LED OFF");
  delay(1000);
}
