/*
  Ultrasonic Sensor HC-SR04 and Arduino Tutorial
  by Dejan Nedelkovski,
  www.HowToMechatronics.com
*/
// defines pins numbers
int trigPin[] = {2, 3, 4, 5};
int echoPin[] = {6, 7, 8, 9};
// defines variables: a 5x4 array to store the last 5 distance readings for each of the 4 sensors
int distances[5][4] = {
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0}
};

void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(trigPin[i], OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin[i], INPUT);  // Sets the echoPin as an Input
  }

  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  for (int i = 0; i < 4; i++) {
    // Clears the trigPin
    digitalWrite(trigPin[i], LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 microseconds
    digitalWrite(trigPin[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin[i], LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(echoPin[i], HIGH);
    
    // Calculating the distance
    int distance = (duration * 0.034) / 2;

    // Shift previous results and store the new distance (if valid)
    if (distance < 500) {
      transfer(i, distance);
    }

    delay(0);
  }


  Serial.print(0);
  Serial.print(", ");
  // Calculate smoothed average and print results
  for (int i = 0; i < 4; i++) {
    int avgDistance = average(i);
    Serial.print(avgDistance);
    Serial.print(", ");
  }
  Serial.println(200);
}

// Shifts previous distance measurements up and adds the new one
void transfer(int sensorIndex, int newDistance) {
  // Shift all rows up for the specific sensor
  for (int i = 0; i < 4; i++) {
    distances[i][sensorIndex] = distances[i + 1][sensorIndex];
  }
  // Add the new reading at the end
  distances[4][sensorIndex] = newDistance;
}

// Calculates the average distance for a specific sensor
int average(int sensorIndex) {
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += distances[i][sensorIndex];
  }
  return sum / 5; // Return the average
}
