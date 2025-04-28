
#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>
#include <SoftwareSerial.h>

// UART Pins
SoftwareSerial arduinoSerial(13, 255); // RX=13 (no TX needed)

// WiFi credentials
const char* ssid = "12345678";      // Change to SSID of network
const char* password = "11111111";  // Change to password of network
// Note: Ensure the ESP8266 is connected to the same network as the computer running the server

// Server connection settings
const char* serverIP = "192.168.133.30"; // CHANGE THIS to your NodeServer IP address
const int serverPort = 8080;

// WebSocket client
WebSocketsClient webSocket;

// Variables to track robot state
String robotStatus = "stopped"; // "running", "stopped", "error"
String robotName = "Connecting..."; // Default robot name until received from Arduino
float robotSpeed = 0.0;
unsigned long startTime = 0;
unsigned long lastDataSent = 0;
unsigned long lastReconnectAttempt = 0;
unsigned long lastHeartbeat = 0;
bool isConnected = false;
int connectionFailCount = 0;

// Persistent serial buffer
char csvBuffer[256];
size_t csvIndex = 0;

// Function declarations
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
void parseAndSendData(char* csvData);
void handleLogMessage(char* logContent);
void handleIdentityMessage(char* identityContent);
void logMessage(const String& message);

void setup() {
  Serial.begin(115200);
  arduinoSerial.begin(9600);

  delay(1000); // Give time for Serial to initialize
  
  Serial.println("\n\n"); 
  Serial.println("MazeRunner ESP8266 starting up...");
  Serial.println("-------------------------------");

  // Connect to WiFi with timeout
  Serial.print("Connecting to WiFi SSID: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0; //Timeout mechanism
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (attempts >= 30) {
    Serial.println("\nWiFi connection failed! Restarting...");
    delay(1000);
    ESP.restart();
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  startTime = millis();
  Serial.println("Setup complete");

  // Connect to WebSocket server
  Serial.print("Connecting to WebSocket server at ");
  Serial.print(serverIP);
  Serial.print(":");
  Serial.println(serverPort);
  
  webSocket.begin(serverIP, serverPort, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED: {
      Serial.println("WebSocket disconnected");
      isConnected = false;
      break;
    }
    case WStype_CONNECTED: {
      Serial.println("WebSocket connected");
      isConnected = true;
      // Send initial connection message
      String initialMessage = String(millis()) + ";0;0;0;0;;;;IDENTITY:" + robotName;
      webSocket.sendTXT(initialMessage);
      Serial.println("Sent initial message: " + initialMessage);
      break;
    }
    case WStype_TEXT: {                             //Legacy feature (kept for error catching)
      String receivedText = String((char*)payload);
      Serial.printf("Received: %s\n", payload);
      
      // Forward to Arduino if needed
      if (receivedText.indexOf("command") > 0) {
        arduinoSerial.println(receivedText);
      }
      break;
    }
    case WStype_ERROR: {
      Serial.println("WebSocket error");
      break;
    }
    default: {
      break;
    }
  }
}

void loop() {
  // Keep the WebSocket connection alive                                                               
  webSocket.loop();
  
  // Send heartbeat every 30 seconds
  if (isConnected && millis() - lastHeartbeat > 30000) {
    String heartbeatMsg = String(millis()) + ";0;0;0;0;;;;Heartbeat";
    webSocket.sendTXT(heartbeatMsg);
    lastHeartbeat = millis();
  }
  
  // Process data from Arduino
  while (arduinoSerial.available()) {
    char c = arduinoSerial.read();
    
    // Store character if buffer not full
    if (csvIndex < sizeof(csvBuffer)-1) {
      csvBuffer[csvIndex++] = c;
    }
    
    // Process when newline received          
    if (c == '\n') {
      csvBuffer[csvIndex] = '\0'; // Null-terminate
      
      // Check if the buffer starts with specific prefixes
      if (strncmp(csvBuffer, "LOG:", 4) == 0) {
        handleLogMessage(csvBuffer + 4);
      } else if (strncmp(csvBuffer, "IDENTITY:", 9) == 0) {
        handleIdentityMessage(csvBuffer + 9);
      } else {
        parseAndSendData(csvBuffer);
      }
      
      csvIndex = 0; // Reset buffer
    }
  }
  
  // Check WiFi connection and attempt reconnect if needed
  if (WiFi.status() != WL_CONNECTED && millis() - lastReconnectAttempt > 10000) {
    Serial.println("WiFi connection lost. Attempting to reconnect...");
    WiFi.reconnect();
    lastReconnectAttempt = millis();
  }
}

void parseAndSendData(char* csvData) {
  char* tokens[9];
  uint8_t tokenCount = 0;
  
  char* token = strtok(csvData, ",");
  while (token && tokenCount < 9) {
    tokens[tokenCount++] = token;
    token = strtok(NULL, ",");
  }
  
  if (tokenCount < 5) {
    Serial.printf("Invalid CSV: Expected at least 5 fields, got %d\n", tokenCount);
    return;
  }

  // Get direction character from fifth field and ensure uppercase
  char directionChar = 'F'; // Default to Forward
  if (tokenCount > 4) {
    directionChar = tokens[4][0];
    // Ensure direction is uppercase for consistency
    if (directionChar >= 'a' && directionChar <= 'z') {
      directionChar = directionChar - 32; // Convert to uppercase
    }
  }

  // Format data as semicolon-separated string: timestamp;front;left;right;direction;value1;value2;feature_type;log_content
  String dataString = String(millis()) + ";" + 
                      String(tokens[1]) + ";" +  // front
                      String(tokens[2]) + ";" +  // left
                      String(tokens[3]) + ";" +  // right
                      String(directionChar) + ";"; // direction (first char)
  
  // Add speed and battery values if available 
  if (tokenCount > 5) {
    dataString += String(tokens[5]);
  } else {
    dataString += "0"; // Default speed
  }
  
  if (tokenCount > 6) {
    dataString += ";" + String(tokens[6]);
  } else {
    dataString += ";100"; // Default battery
  }
  
  // Add feature type if available (8th field, index 7)
  if (tokenCount > 7) {
    dataString += ";" + String(tokens[7]);
  } else {
    dataString += ";PATH"; // Default feature type
  }
  
  // Only send if connected and not too frequently
  if (isConnected && millis() - lastDataSent > 100) {
    webSocket.sendTXT(dataString);
    lastDataSent = millis();
    
    // For debugging
    Serial.println("Sent: " + dataString);
  }
}

void handleLogMessage(char* logContent) {
  // Format: timestamp;f;l;r;dir;speed;bat;feature;log_content
  String logString = String(millis()) + ";0;0;0;0;0;0;;" + String(logContent);
  
  if (isConnected) {
    webSocket.sendTXT(logString);
    Serial.println("Log sent: " + logString);
  }
}

void handleIdentityMessage(char* identityContent) {
  // Update the robot name
  robotName = String(identityContent);
  robotName.trim();
  
  // Send identity message to server
  String identityString = String(millis()) + ";0;0;0;0;0;0;;IDENTITY:" + robotName;
  
  if (isConnected) {
    webSocket.sendTXT(identityString);
    Serial.println("Identity sent: " + robotName);
  }
}

void logMessage(const String& message) {
  Serial.println(message);
  handleLogMessage((char*)(message.c_str()));
}
