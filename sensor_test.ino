const int echoPin[] = {6, 7, 8, 9};
const int triggerPin[] = {2, 3, 4 ,5};

void setup() {
  Serial.begin(9600); 
  Serial.write("Starting sensors");
  // put your setup code here, to run once:
  for (int i = 0; i < sizeof(echoPin) / sizeof(echoPin[0]); i++) {  
    pinMode(echoPin[i], INPUT);
    pinMode(triggerPin[i], OUTPUT);
  }
  Serial.println("\nStarting loop");
}

void loop() {
  // put your main code here, to run repeatedly:
  float sensors[] = {0, 0, 0, 0};
  for (int i = 0; i < sizeof(echoPin) / sizeof(echoPin[0]); i++) {  
    digitalWrite(triggerPin[i], LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin[i], LOW);
    float duration = pulseIn(echoPin[i], HIGH);
    float distance = (duration*.0343)/2;

    if (distance < 400) sensors[i] = distance;


  } 
  float average = (sensors[0] + sensors[1] + sensors[2] + sensors[3])/4;
  Serial.print(average);
  Serial.print(", ");
  Serial.print(200);
  Serial.print(", ");
  Serial.println(0);

  delay(200);
}
