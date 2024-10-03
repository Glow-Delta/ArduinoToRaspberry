#include <Wire.h>

#define MCP23017_ADDRESS 0x20 // Default I2C address

volatile bool interruptTriggered = false;  // Flag for interrupt

void setup() {
  Wire.begin();                 // Start I²C bus
  pinMode(3, INPUT_PULLUP);     // Arduino pin 3 for INTA (interrupt pin), with internal pull-up
  
  Serial.begin(9600);           // Serial monitor for debugging
  
  // Configure the MCP23017 to handle the button on GPA7
  setupMCP23017();

  // Attach interrupt to pin 3 (INTA)
  attachInterrupt(digitalPinToInterrupt(3), handleInterrupt, FALLING);  // Trigger on FALLING (INTA goes LOW)
}

void loop() {
  // Check if interrupt flag is set (ISR sets this flag)
  if (interruptTriggered) {

    Serial.println("INTERUPT!!!!");
    interruptTriggered = false;  // Reset the flag
    
    // Read the interrupt flags to determine which pin triggered it
    uint8_t intFlag = readRegister(0x0E);  // INTFA: Interrupt flag register
    
    if (intFlag & 0b10000000) {  // Check if GPA7 caused the interrupt
      // Determine if the button was pressed or released by reading GPIOA
      uint8_t gpioState = readRegister(0x12);  // GPIOA: read current state of port A
      
      if (gpioState & 0b10000000) {
        Serial.println("Button on GPA7 released!");
      } else {
        Serial.println("Button on GPA7 pressed!");
      }
      
      // Clear the interrupt by reading GPIOA (necessary to reset INTA)
      readRegister(0x12);  // GPIOA: reading clears the interrupt
    }
  }
}

// Function that will be called when an interrupt occurs
void handleInterrupt() {
  interruptTriggered = true;  // Set the flag to handle interrupt in the loop
}

// Function to configure MCP23017 registers for GPA7
void setupMCP23017() {
  // Set GPA7 as input
  writeRegister(0x00, 0b10000000);  // IODIRA: 1 = input for GPA7
  
  // Enable pull-up resistor on GPA7 (ensure button reads high when unpressed)
  writeRegister(0x0C, 0b10000000);  // GPPUA: 1 = pull-up enabled on GPA7
  
  // Enable interrupt on GPA7
  writeRegister(0x04, 0b10000000);  // GPINTENA: 1 = enable interrupt on GPA7
  
  // Set interrupt-on-change to trigger on a state change
  writeRegister(0x08, 0b00000000);  // INTCONA: 0 = compare to previous value (interrupt on any state change)
  
  // Set default comparison value (not used in this case)
  writeRegister(0x06, 0b00000000);  // DEFVALA: default value is ignored
  
  // Clear any pending interrupts (optional)
  readRegister(0x0E);  // INTFA: Read to clear any existing interrupt flags
}

// Function to write to MCP23017 register
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Function to read from MCP23017 register
uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(MCP23017_ADDRESS, 1);
  return Wire.read();
}
