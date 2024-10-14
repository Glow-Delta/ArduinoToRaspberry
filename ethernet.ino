#include <UIPEthernet.h>  // Use UIPEthernet for ENC28J60

#define csPin 10  // Chip Select pin for ENC28J60

// Ethernet settings
byte mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDA, 0x02 };
EthernetClient client;

// Sendable tube data
int TubeNumber = 404;
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

bool connected = false;

void setup() {
  // Initialize Ethernet and Serial communication
  Serial.begin(9600);
  while (!Serial) {
     // Wait for serial port to connect. Needed for native USB
  }

  // Start Ethernet connection
  Ethernet.init(csPin);
  Ethernet.begin(mac, ip, gateway, subnet);
  Serial.println("Ethernet initialized.");

  // Allow the Ethernet shield to initialize
  delay(2000);

  Serial.println("Start connection check!");
  if (client.connect(server, ServerPort)) {
    Serial.println("Connected to Server");
    Serial.println("Requesting TubeID!");
    sendValueToServer();
    connected = true;
  } else {
    Serial.println("Connection failed");
    connected = false;
  }
}

void loop() {
  // Handle server response if available
  if (client.available()) {
    char c = client.read();
    Serial.print(c);
    ServerResponse += c;  // Collect server response data
    startMillis = millis(); // Reset timeout for new data
  }

  while (!connected) {
    Serial.println("Client not connected. Attempting to reconnect...");
    client.stop();  // Ensure to close the connection
    TubeNumber = 404;
    delay(NextResponseTime * 1000);  // Wait before reconnecting
    if (client.connect(server, ServerPort)) {
        Serial.println("Reconnected to server");
        sendValueToServer();  // Resend data if needed
        connected = true;
    } else {
        Serial.println("Reconnection failed.");
    }
  }

  // Handle server response if there's a response to process
  handleServerResponse();
}

// Function to send a value to the server using HTTP POST
void sendValueToServer() {
  // Update the valueToSend array before sending it to the server
  valueToSend[0] = TubeNumber;
  valueToSend[1] = ActivationFloat;

  int ArrayLength = sizeof(valueToSend) / sizeof(valueToSend[0]);

  String postData = "value=";
  for (int i = 0; i < ArrayLength; i++) {
    postData += String(valueToSend[i]);
    if (i < ArrayLength - 1) {
      postData += ","; 
    }
  }

  client.print(postData);

  Serial.print("Sent values: ");
  Serial.println(postData);
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
