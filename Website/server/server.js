
const WebSocket = require('ws');
const http = require('http');
const express = require('express');
const cors = require('cors');
const path = require('path');
const { networkInterfaces } = require('os');

// Create express app and use CORS
const app = express();
app.use(cors());
app.use(express.static(path.join(__dirname, 'public')));

// Create HTTP server
const server = http.createServer(app);

// Create a logging function
function logWithTime(message) {
  const timestamp = new Date().toISOString();
  console.log(`[${timestamp}] ${message}`);
}

// Create a WebSocket server that listens on all interfaces
const wss = new WebSocket.Server({ server });

// Store connected clients
const clients = new Set();

// Store robot connection data
let robotClient = null;
let robotData = null;
let isRobotConnected = false;
let robotName = "Connecting...";
const lastSensorData = {
  front: 0,
  left: 0,
  right: 0,
  timestamp: Date.now(),
  direction: 'F'
};

// Add a ping endpoint for connection testing
app.get('/ping', (req, res) => {
  res.status(200).send('pong');
});

// Function to identify if a message is from the robot
function isRobotMessage(data) {
  try {
    // Try to parse as semicolon-separated values, which is the robot's format
    const parts = data.toString().trim().split(';');
    return parts.length >= 5 && !isNaN(parseFloat(parts[1])) && !isNaN(parseFloat(parts[2])) && !isNaN(parseFloat(parts[3]));
  } catch (e) {
    return false;
  }
}

// Function to parse direction character to more descriptive string
function parseDirection(dirChar) {
  // Original direction character(s) should be preserved for the maze map
  return dirChar;
}

// Function to parse incoming data from robot
function parseRobotData(dataString, robotWs) {
  try {
    const parts = dataString.trim().split(';');
    if (parts.length >= 5) {
      const timestamp = parseInt(parts[0], 10);
      const front = parseFloat(parts[1]);
      const left = parseFloat(parts[2]);
      const right = parseFloat(parts[3]);
      const direction = parts[4].toUpperCase(); // Ensure uppercase for consistency
      const speed = parts.length > 5 ? parseFloat(parts[5]) : 0;
      const battery = parts.length > 6 ? parseFloat(parts[6]) : 100;
      const featureType = parts.length > 7 ? parts[7] : "PATH";
      
      // Check for identity message
      if (parts.length > 8 && parts[8].startsWith("IDENTITY:")) {
        const newRobotName = parts[8].substring(9);
        if (newRobotName && newRobotName.length > 0) {
          robotName = newRobotName;
          logWithTime(`Robot identity set to: ${robotName}`);
          
          // Send identity update to all clients
          broadcastToClients(JSON.stringify({
            type: 'identity',
            name: robotName
          }));
          return;
        }
      } 
      // Check for log message - if we have at least 8 parts and it's not an identity message
      else if (parts.length > 8) {
        const logContent = parts[8];
        const logMessage = JSON.stringify({
          type: 'log',
          timestamp,
          content: logContent,
          msgType: 'info'
        });
        broadcastToClients(logMessage);
        logWithTime(`Log message forwarded: ${logContent}`);
        return;
      }

      // Update last sensor data if direction is valid and not a log message
      if (direction !== '0') {
        lastSensorData.front = front;
        lastSensorData.left = left;
        lastSensorData.right = right;
        lastSensorData.timestamp = timestamp;
        lastSensorData.direction = direction;
        lastSensorData.featureType = featureType;

        // Create a JSON message for clients
        const sensorMessage = JSON.stringify({
          type: 'sensor_data',
          timestamp,
          front,
          left,
          right,
          direction,
          speed,
          battery,
          featureType
        });

        robotData = sensorMessage;
        broadcastToClients(sensorMessage);

        // If there's a direction change, also send a move message
        if (direction && ['F', 'L', 'R', 'B'].includes(direction)) {
          const moveMessage = JSON.stringify({
            type: 'move',
            direction,
            timestamp,
            featureType
          });
          broadcastToClients(moveMessage);
        }
      }
      
      // If not already tracked as the robot client
      if (!robotClient && robotWs) {
        robotClient = robotWs;
        isRobotConnected = true;
        
        // Broadcast connection status and robot name
        broadcastToClients(JSON.stringify({
          type: 'connection_status',
          connected: true
        }));
        
        // Also send the robot name
        broadcastToClients(JSON.stringify({
          type: 'identity',
          name: robotName
        }));
        
        logWithTime('Robot identified and connected');
      }
    }
  } catch (e) {
    logWithTime(`Error parsing data: ${e.message}`);
  }
}

// Function to broadcast message to all web clients
function broadcastToClients(message) {
  let clientCount = 0;
  clients.forEach(client => {
    // Don't send back to robot client
    if (client !== robotClient && client.readyState === WebSocket.OPEN) {
      client.send(message);
      clientCount++;
    }
  });
  if (clientCount > 0) {
    logWithTime(`Broadcasted message to ${clientCount} clients`);
  }
}

// Handle WebSocket connections
wss.on('connection', (ws, req) => {
  const ip = req.socket.remoteAddress;
  logWithTime(`Client connected from ${ip}`);
  clients.add(ws);

  // Send current connection status to the client
  ws.send(JSON.stringify({
    type: 'connection_status',
    connected: isRobotConnected
  }));
  
  // Send current robot name
  ws.send(JSON.stringify({
    type: 'identity',
    name: robotName
  }));

  // If we have recent sensor data, send it
  if (robotData) {
    ws.send(robotData);
  }

  // Handle messages from clients
  ws.on('message', (message) => {
    try {
      // Try to parse as JSON first
      try {
        const data = JSON.parse(message.toString());
        logWithTime(`Client sent JSON: ${message.toString()}`);
        
        // Handle web client commands
        if (data.command && robotClient && robotClient.readyState === WebSocket.OPEN) {
          // Forward commands to robot
          robotClient.send(JSON.stringify(data));
          logWithTime(`Command forwarded to robot: ${JSON.stringify(data)}`);

          // Also send move notification for F, L, R, B commands for maze mapping
          if (['F', 'L', 'R', 'B'].includes(data.command.toUpperCase())) {
            const moveMessage = JSON.stringify({
              type: 'move',
              direction: data.command.toUpperCase(),
              timestamp: Date.now(),
              featureType: data.featureType || "PATH" // Include feature type if provided
            });
            broadcastToClients(moveMessage);
          }
        }
      } catch (jsonError) {
        // Not JSON, might be direct data from ESP8266
        const dataString = message.toString();
        logWithTime(`Received raw data: ${dataString}`);
        
        // Check if this is robot data format
        if (isRobotMessage(dataString)) {
          parseRobotData(dataString, ws);
        }
      }
    } catch (e) {
      logWithTime(`Error handling message: ${e.message}`);
    }
  });

  // Handle client disconnection
  ws.on('close', () => {
    logWithTime(`Client disconnected from ${ip}`);
    clients.delete(ws);

    // Check if the disconnected client is the robot
    if (ws === robotClient) {
      robotClient = null;
      isRobotConnected = false;
      robotName = "Connecting..."; // Reset name when robot disconnects
      
      broadcastToClients(JSON.stringify({
        type: 'connection_status',
        connected: false
      }));
      
      // Also broadcast the name reset
      broadcastToClients(JSON.stringify({
        type: 'identity',
        name: robotName
      }));
      
      logWithTime('Robot disconnected');
    }
  });

  // Handle errors
  ws.on('error', (error) => {
    logWithTime(`WebSocket error from ${ip}: ${error.message}`);
    clients.delete(ws);

    // Check if the disconnected client is the robot
    if (ws === robotClient) {
      robotClient = null;
      isRobotConnected = false;
      robotName = "Connecting..."; // Reset name when robot disconnects
      
      broadcastToClients(JSON.stringify({
        type: 'connection_status',
        connected: false
      }));
      
      // Also broadcast the name reset
      broadcastToClients(JSON.stringify({
        type: 'identity',
        name: robotName
      }));
    }
  });
});

// Start the server on all network interfaces
const PORT = 8080;
server.listen(PORT, '0.0.0.0', () => {
  logWithTime(`WebSocket server running on port ${PORT}`);
  
  // Log all server IP addresses to help with connections
  const nets = networkInterfaces();
  
  logWithTime(`Server IP addresses (Use these for the ESP8266):`);
  for (const name of Object.keys(nets)) {
    for (const net of nets[name]) {
      // Skip over non-IPv4 and internal (i.e. 127.0.0.1) addresses
      if (net.family === 'IPv4' && !net.internal) {
        logWithTime(`Interface: ${name}, IP: ${net.address}`);
      }
    }
  }
});

// Handle server shutdown
process.on('SIGINT', () => {
  logWithTime('Server shutting down...');
  
  server.close(() => {
    logWithTime('HTTP/WebSocket server closed');
    process.exit(0);
  });
});
