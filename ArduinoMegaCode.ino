const int trigPins[] = {22, 24, 26, 28, 30, 32, 34, 36};
const int echoPins[] = {46, 48, 50, 52, 38, 40, 42, 44};
const int numSensors = 8;

void setup() {
  Serial.begin(9600);
  
  for (int i = 0; i < numSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

void loop() {
  for (int i = 0; i < numSensors; i++) {
    long duration, distance;
    
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);
    
    // Read the echo pin
    duration = pulseIn(echoPins[i], HIGH);

    // Calculate the distance (in cm)
    distance = duration * 0.034 / 2; // Speed of sound = 340 m/s = 0.034 cm/µs

    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(distance);
    Serial.println(" cm");
    
    delay(100);
  }
  
  delay(200);
}
