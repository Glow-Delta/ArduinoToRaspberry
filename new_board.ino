#include <Wire.h>

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

#define SENSOR_COUNT_A 8
#define SENSOR_COUNT_B 8

volatile bool interupted;
unsigned long risingA = 0;
unsigned long travelTimeA = 0;

const uint8_t sensorPinsA[SENSOR_COUNT_A] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000};

int lastTriggeredSensorA = 0;
bool sensorTriggeredA = false;
unsigned long lastSensorTriggerTimeA = 0;

unsigned long risingB = 0;
unsigned long travelTimeB = 0;

const uint8_t sensorPinsB[SENSOR_COUNT_B] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000};

int lastTriggeredSensorB = 0;
bool sensorTriggeredB = false;
unsigned long lastSensorTriggerTimeB = 0;

int distances[SENSOR_COUNT_A + SENSOR_COUNT_A];

bool ASide = false;

void checkForInterupt(unsigned long now) {
  if (ASide = true) {
    if (!sensorTriggeredA) {
      triggerNextUltrasonicA();
    }
    else if (interupted) {
      readRegisterA(INTCAPA);
      interupted = false;

      if (risingA == 0) {
        risingA = now;
      }
      else {
        travelTimeA = now - risingA;
        sensorTriggeredA = false;
        ASide = false;
      }
    }
    else if (now - lastSensorTriggerTimeA > 50000) {
      ASide = false;
      readRegisterA(INTCAPA);
      triggerNextUltrasonicA();
    }
  }
  else {
    if (!sensorTriggeredB) {
      triggerNextUltrasonicB();
    }
    else if (interupted) {
      readRegisterB(INTCAPA);
      interupted = false;

      if (risingB == 0) {
        risingB = now;
      }
      else {
        travelTimeB = now - risingB;
        sensorTriggeredB = false;
        //ASide = true;
      }
    }
    else if (now - lastSensorTriggerTimeB > 50000) {
      //ASide = true;
      readRegisterB(INTCAPA);
      triggerNextUltrasonicB();
    }
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
  interupted = false;
  sensorTriggeredA = true;
  lastSensorTriggerTimeA = micros();

  int ultrasonicToTrigger = (lastTriggeredSensorA + 1) % SENSOR_COUNT_A;
  lastTriggeredSensorA = ultrasonicToTrigger;

  //Serial.print("triggering A: ");
  //Serial.println(ultrasonicToTrigger);

  
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
  interupted = false;
  sensorTriggeredB = true;
  lastSensorTriggerTimeB = micros();

  int ultrasonicToTrigger = (lastTriggeredSensorB + 1) % SENSOR_COUNT_B;
  lastTriggeredSensorB = ultrasonicToTrigger;

  //Serial.print("triggering B: ");
  //Serial.println(ultrasonicToTrigger);

  
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
void handleInterrupt() {
  //Serial.println("interupt");
  interupted = true;
}

void printDistances(int distances[]) {
  int size = SENSOR_COUNT_A + SENSOR_COUNT_B;
  Serial.print("[");
  for (int i = 0; i < size; i++) {
    Serial.print(distances[i]);
    if (i < size - 1) {
      Serial.print(", ");  // Add a comma and space between values
    }
  }
  Serial.println("]");  // Close the array with a bracket and move to the next line
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);  // Set I²C speed to 400kHz

  pinMode(7, INPUT_PULLUP);     // Arduino pin 2 for INTA (interrupt pin), with internal pull-up
  //pinMode(3, INPUT_PULLUP);     // Arduino pin 2 for INTA (interrupt pin), with internal pull-up

  setupMCP23017();

  //Attach interrupt to pin 3 (INTA)
  attachInterrupt(digitalPinToInterrupt(7), handleInterrupt, CHANGE);  // Trigger on CHANGE (INTA goes LOW)
  //attachInterrupt(digitalPinToInterrupt(3), handleInterruptB, CHANGE);  // Trigger on CHANGE (INTA goes LOW)

  interupted = false;
  //interuptedB = false;
}

void loop() {
  unsigned long now = micros();

  printDistances(distances);

  checkForInterupt(now);
  //checkForInteruptsB(now);
  
  //TODO handel the travel times
  if (travelTimeA != 0) {
    int distanceCm = travelTimeA *  0.034 / 2;
    if (distanceCm < 250) {
      distances[lastTriggeredSensorA] = distanceCm;
    }
  }

  if (travelTimeB != 0) {
    int distanceCm = travelTimeB *  0.034 / 2;
    if (distanceCm < 250) {
      distances[lastTriggeredSensorA + SENSOR_COUNT_A] = distanceCm;
    }
  }
}










