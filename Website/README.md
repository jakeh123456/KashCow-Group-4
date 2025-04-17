How to launch the website and server for the maze solver:

# Step 1: Navigate to the project directory.
In your terminal with dir as KashCow-Group-4 type:
```
cd Website
```
# Step 3: Install the necessary dependencies.
```
npm i
```
# Step 4: Run the Node server to receive data (socket connections)
```
cd server
node server.js
```
# Step 5: Open new terminal and Start the website server (vite instance). Make sure dir is Website NOT /server
```
npm run dev
```
# Step 6: Locate the server ip and port from the node server terminal and upload it to esp
  Esp code is under Esp8266 folder



For sensor data, the message format is comma-separated values (CSV) with the following fields:
```
<timestamp>,<front-distance>,<left-distance>,<right-distance>,<direction>,<value1>,<value2>,<feature>,<log>
```
Where:

    <timestamp>: Usually an incrementing value or milliseconds from Arduino startup
    <front-distance>: Front sensor distance in cm
    <left-distance>: Left sensor distance in cm
    <right-distance>: Right sensor distance in cm
    <direction>: Current robot direction (F, L, R, B) or numeric values (0, 1, 2, 3)
    <value1>, <value2>: Optional additional values (Currently set to : speed battery level)
    <feature>: Describes maze feature such as deadend or turn, etc.
    <log>: Optional log message to append
        If log message starts with "IDENTITY:" then this will be used for RobotName which will be seen on website

