#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>

// Ethernet configuration
byte mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDA, 0x02 };
EthernetClient client;

// Sendable tube data
int TubeNumber = 1;
int ActivationFloat = 0;
int valueToSend[] = { TubeNumber, ActivationFloat };

// Client information
IPAddress ip(192, 168, 0, 2);  // Arduino IP address
int NextResponseTime = 5; // In seconds
String ServerResponse = "";

// Server information
IPAddress server(192, 168, 0, 1);  // Raspberry Pi IP address (server)
int ServerPort = 8080;

// More connection information
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

unsigned long responseTimeout = 5000; // Timeout for server response in ms
unsigned long startMillis;  // To store millis at the start of waiting

#define MCP23017_ADDRESS_A 0x20 // Default I2C address
#define MCP23017_ADDRESS_B 0x21 // Default I2C address

// MCP23017 Register Addresses
#define IODIRA   0x00  // I/O direction register for Port A (1 = input, 0 = output)
#define IODIRB   0x01  // I/O direction register for Port B
#define IPOLA    0x02  // Input polarity register for Port A (1 = inverted, 0 = normal)
#define IPOLB    0x03  // Input polarity register for Port B
#define GPINTENA 0x04  // Interrupt-on-change control for Port A (1 = enable interrupt)
#define GPINTENB 0x05  // Interrupt-on-change control for Port B
#define DEFVALA  0x06  // Default compare register for interrupt on Port A
#define DEFVALB  0x07  // Default compare register for interrupt on Port B
#define INTCONA  0x08  // Interrupt control register for Port A (0 = previous, 1 = DEFVAL)
#define INTCONB  0x09  // Interrupt control register for Port B
#define IOCONA   0x0A  // I/O expander configuration register for Port A
#define IOCONB   0x0B  // I/O expander configuration register for Port B
#define GPPUA    0x0C  // Pull-up resistor configuration for Port A (1 = enable pull-up)
#define GPPUB    0x0D  // Pull-up resistor configuration for Port B
#define INTFA    0x0E  // Interrupt flag register for Port A (read-only, indicates the pin that caused the interrupt)
#define INTFB    0x0F  // Interrupt flag register for Port B
#define INTCAPA  0x10  // Interrupt capture register for Port A (state of the pin at the time of the interrupt)
#define INTCAPB  0x11  // Interrupt capture register for Port B
#define GPIOA    0x12  // General-purpose I/O register for Port A (read/write the state of the pins)
#define GPIOB    0x13  // General-purpose I/O register for Port B
#define OLATA    0x14  // Output latch register for Port A (read/write the output state)
#define OLATB    0x15  // Output latch register for Port B

//volatile bool intARecieved = false;

#define SENSOR_COUNT_A 4
#define SENSOR_COUNT_B 4

volatile bool interuptedA;
unsigned long risingA = 0;
unsigned long travelTimeA = 0;

const uint8_t sensorPinsA[SENSOR_COUNT_A] = {0b00000001, 0b00000010, 0b00000100, 0b00001000};

int lastTriggeredSensorA = 0;
bool sensorTriggeredA = false;
unsigned long lastSensorTriggerTimeA = 0;

volatile bool interuptedB;
unsigned long risingB = 0;
unsigned long travelTimeB = 0;

const uint8_t sensorPinsB[SENSOR_COUNT_B] = {0b00000001, 0b00000010, 0b00000100, 0b00001000};

int lastTriggeredSensorB = 0;
bool sensorTriggeredB = false;
unsigned long lastSensorTriggerTimeB = 0;

void setup() {
  Wire.begin();
  Wire.setClock(800000);  // Set I²C speed to 400kHz
  Serial.begin(9600);           // Serial monitor for debugging
  while(!Serial){}

  Serial.println("setup!");

  pinMode(2, INPUT_PULLUP);     // Arduino pin 2 for INTA (interrupt pin), with internal pull-up
  pinMode(3, INPUT_PULLUP);     // Arduino pin 2 for INTA (interrupt pin), with internal pull-up

  setupMCP23017();

  //Attach interrupt to pin 3 (INTA)
  attachInterrupt(digitalPinToInterrupt(2), handleInterruptA, CHANGE);  // Trigger on CHANGE (INTA goes LOW)
  attachInterrupt(digitalPinToInterrupt(3), handleInterruptB, CHANGE);  // Trigger on CHANGE (INTA goes LOW)

  interuptedA = false;
  interuptedB = false;

  //Ethernet connection
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB
  }

  // Start Ethernet connection
  Ethernet.begin(mac, ip, gateway, subnet);
  Serial.println("Ethernet initialized.");

  // Allow the Ethernet shield to initialize
  delay(10000);

  Serial.println("Start connection check!");
  if (client.connect(server, ServerPort)) {
    Serial.println("Connected to Raspberry Pi");
    Serial.println("Requesting TubeID!");
    sendValueToServer();
  } else {
    Serial.println("Connection failed");
  }
}

void loop() {

  checkForInteruptsA();
  checkForInteruptsB();

  //Ethernet
  if (client.available()) {
      char c = client.read();
      Serial.print(c);
      ServerResponse += c;  // Collect server response data
      startMillis = millis(); // Reset timeout for new data
    }

    // If the server disconnected or no response in a while, stop the client
  // if (!client.connected() || (millis() - startMillis > responseTimeout)) {
  //   Serial.println("Disconnected or Timeout");
  //   handleServerResponse(); // Process collected server response

  //   // Wait before reconnecting
  //   delay(NextResponseTime * 1000);   
  //   sendValueToServer();
  // }

  if (!client.connected()) {
        Serial.println("Client not connected. Attempting to reconnect...");
        client.stop();  // Ensure to close the connection
        TubeNumber = 404;
        delay(NextResponseTime * 1000);  // Wait before reconnecting
        if (client.connect(server, ServerPort)) {
            Serial.println("Reconnected to server");
            sendValueToServer();  // Resend data if needed
        } else {
            Serial.println("Reconnection failed.");
        }
    }
  
  //TODO handel the travel time
  if (travelTimeA != 0) {
    //Serial.println(travelTime);
    int distanceCm = travelTimeA *  0.034 / 2;
    //Serial.print("Distance: ");
    if (distanceCm < 250) {
      Serial.println("A");
      Serial.print(lastTriggeredSensorA);
      Serial.print(", ");
      Serial.print(200);
      Serial.print(", ");
      Serial.println(distanceCm);

      long mappedDistanceInt = map(distanceCm, 1, 200, 100, 999);
      float mappedDistance = mappedDistanceInt / 1000.0;

      Serial.print("Mapped distance (int): ");
      Serial.println(mappedDistanceInt);  // Print with 3 decimal places

      ActivationFloat = mappedDistanceInt;

      sendValueToServer();
    }
    
    //Serial.println(" cm");
  }


  //TODO handel the travel time
  if (travelTimeB != 0) {
    //Serial.println(travelTime);
    int distanceCm = travelTimeB *  0.034 / 2;
    //Serial.print("Distance: ");
    if (distanceCm < 250) {
      Serial.println("B");
      Serial.print(lastTriggeredSensorB);
      Serial.print(", ");
      Serial.print(200);
      Serial.print(", ");
      Serial.println(distanceCm);
      
      long mappedDistanceInt = map(distanceCm, 1, 200, 100, 999);
      float mappedDistance = mappedDistanceInt / 1000.0;

      Serial.print("Mapped distance (int): ");
      Serial.println(mappedDistanceInt);  // Print with 3 decimal places

      ActivationFloat = mappedDistanceInt;

      sendValueToServer();
    }
    
    //Serial.println(" cm");
  }
   
  
}


void checkForInteruptsA() {
  unsigned long now = micros();

  if (!sensorTriggeredA) {
    triggerNextUltrasonicA();
  }
  else if (interuptedA) {
    readRegisterA(INTCAPA);
    interuptedA = false;

    if (risingA == 0) {
      risingA = now;
    }
    else {
      travelTimeA = now - risingA;
      sensorTriggeredA = false;
    }
  }
  else if (now - lastSensorTriggerTimeA > 50000) {
    readRegisterA(INTCAPA);
    triggerNextUltrasonicA();
  }
}
void checkForInteruptsB() {
  unsigned long now = micros();

  if (!sensorTriggeredB) {
    triggerNextUltrasonicB();
  }
  else if (interuptedB) {
    readRegisterB(INTCAPA);
    interuptedB = false;

    if (risingB == 0) {
      risingB = now;
    }
    else {
      travelTimeB = now - risingB;
      sensorTriggeredB = false;
    }
  }
  else if (now - lastSensorTriggerTimeB > 50000) {
    readRegisterB(INTCAPA);
    triggerNextUltrasonicB();
  }
}


// Function to configure MCP23017 registers for GPA7 (echo) and GPB0 (trigger)
void setupMCP23017() {
  // Set GPB0 as output (trigger) and GPA7 as input (echo)
  writeRegisterA(IODIRA, 0b11111111);  // IODIRA: 1 = input for GPA7 (echo)
  writeRegisterA(IODIRB, 0b00000000);  // IODIRB: 0 = output for GPB0 (trigger)
  writeRegisterB(IODIRA, 0b11111111);  // IODIRA: 1 = input for GPA7 (echo)
  writeRegisterB(IODIRB, 0b00000000);  // IODIRB: 0 = output for GPB0 (trigger)

  // Enable interrupt on GPA7 (echo pin)
  writeRegisterA(GPINTENA, 0b11111111);  // GPINTENA: Enable interrupt on GPA7 (echo)
  writeRegisterB(GPINTENA, 0b11111111);  // GPINTENA: Enable interrupt on GPA7 (echo)
  
  // Set interrupt-on-change to trigger on any change (both rising and falling edge) for GPA7
  writeRegisterA(INTCONA, 0b00000000);  // INTCONA: 0 = compare to previous value (interrupt on any change)
  writeRegisterB(INTCONA, 0b00000000);  // INTCONA: 0 = compare to previous value (interrupt on any change)

  writeRegisterA(DEFVALA, 0b11111111); // DEFVALA: 1 = high
  writeRegisterA(DEFVALB, 0b00000000); // DEFVALB: 0 = low
  writeRegisterB(DEFVALA, 0b11111111); // DEFVALA: 1 = high
  writeRegisterB(DEFVALB, 0b00000000); // DEFVALB: 0 = low

  // Enable pull-up resistor on GPA7 (echo)
  writeRegisterA(GPPUA, 0b11111111);  // GPPUA: 1 = pull-up enabled on GPA7 (echo)
  writeRegisterA(GPPUB, 0b11111111);  // GPPUB: 1 = pull-up enabled on GPA7 (echo)
  writeRegisterB(GPPUA, 0b11111111);  // GPPUA: 1 = pull-up enabled on GPA7 (echo)
  writeRegisterB(GPPUB, 0b11111111);  // GPPUB: 1 = pull-up enabled on GPA7 (echo)
  
  // Clear any pending interrupts (optional)
  readRegisterA(0x12);  // GPIOA: Read to clear any existing interrupt flags for Port A
  readRegisterB(0x12);  // GPIOA: Read to clear any existing interrupt flags for Port A
}


// Function to trigger the ultrasonic sensor for chip A
void triggerNextUltrasonicA() {
  //Serial.print("triggering sensor: ");
  //Serial.println(index);
  // Record the time the trigger pulse was sent
  risingA = 0;
  travelTimeA = 0;
  interuptedA = false;
  sensorTriggeredA = true;
  lastSensorTriggerTimeA = micros();

  int ultrasonicToTrigger = (lastTriggeredSensorA + 1) % SENSOR_COUNT_A;
  lastTriggeredSensorA = ultrasonicToTrigger;

  
  Wire.beginTransmission(MCP23017_ADDRESS_A);
  Wire.write(GPIOB);
  Wire.write(sensorPinsA[ultrasonicToTrigger]);
  
  delayMicroseconds(10);  // Delay for 10 microseconds

  Wire.write(GPIOB);
  Wire.write(0b00000000);
  Wire.endTransmission();
}
// Function to trigger the ultrasonic sensor for chip B
void triggerNextUltrasonicB() {
  //Serial.print("triggering sensor: ");
  //Serial.println(index);
  // Record the time the trigger pulse was sent
  risingB = 0;
  travelTimeB = 0;
  interuptedB = false;
  sensorTriggeredB = true;
  lastSensorTriggerTimeB = micros();

  int ultrasonicToTrigger = (lastTriggeredSensorB + 1) % SENSOR_COUNT_B;
  lastTriggeredSensorB = ultrasonicToTrigger;

  
  Wire.beginTransmission(MCP23017_ADDRESS_B);
  Wire.write(GPIOB);
  Wire.write(sensorPinsB[ultrasonicToTrigger]);
  
  delayMicroseconds(10);  // Delay for 10 microseconds

  Wire.write(GPIOB);
  Wire.write(0b00000000);
  Wire.endTransmission();
}


// Function to write to MCP23017 register
void writeRegisterA(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MCP23017_ADDRESS_A);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
// Function to write to MCP23017 register
void writeRegisterB(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MCP23017_ADDRESS_B);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


// Function to read from MCP23017 register
uint8_t readRegisterA(uint8_t reg) {
  Wire.beginTransmission(MCP23017_ADDRESS_A);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(MCP23017_ADDRESS_A, 1);
  return Wire.read();
}
// Function to read from MCP23017 register
uint8_t readRegisterB(uint8_t reg) {
  Wire.beginTransmission(MCP23017_ADDRESS_B);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(MCP23017_ADDRESS_B, 1);
  return Wire.read();
}


// Function that will be called when an interrupt occurs
void handleInterruptA() {
  interuptedA = true;
}
// Function that will be called when an interrupt occurs
void handleInterruptB() {
  interuptedB = true;
}

// Function to send a value to the server using HTTP POST
void sendValueToServer() {
  // Update the valueToSend array before sending it to the server
  // valueToSend[0] = TubeNumber;
  // valueToSend[1] = ActivationFloat;

  // int ArrayLength = sizeof(valueToSend) / sizeof(valueToSend[0]);

  String postData = "tubeNumber=";
  postData += TubeNumber;
  postData +=  ", value=";
  postData += ActivationFloat;

  client.print(postData);

  Serial.print("Sent values: ");
  Serial.println(postData);
}

bool SensorCheck() {
  // Placeholder function for sensor checking logic
  return true; // Assume sensor check passes for this example
}

bool DMXCheck() {
  // Placeholder function for DMX checking logic
  return true; // Assume DMX check passes for this example
}

void handleServerResponse() {
  if (ServerResponse.length() > 0) {
    Serial.println("Server response:");
    Serial.println(ServerResponse);
    extractIDFromCustomResponse();
  } else {
    Serial.println("No response from server.");
  }
}

void extractIDFromCustomResponse() {
  int idIndex = ServerResponse.indexOf("ID=");
  if (idIndex != -1) {
    int idStart = idIndex + 3;  // Skip past "ID="
    int idEnd = ServerResponse.indexOf(",", idStart);  // Find the next comma (or end of the line)
    String idString = ServerResponse.substring(idStart, idEnd);
    float id = idString.toFloat();

    Serial.print("Extracted ID: ");
    Serial.println(id);
    TubeNumber = id;

    ServerResponse = "";  // Clear previous response data
  } else {
    Serial.println("ID not found.");
  }
}