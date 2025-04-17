
# Arduino Messaging Guide for MazeRunner

This guide explains the message formats that the Arduino can send to the ESP8266, which then forwards them to the server.

## Basic Format

All messages follow a comma-separated value (CSV) format, which the ESP8266 then converts to semicolon-separated format for the server:

```
<timestamp>,<front-distance>,<left-distance>,<right-distance>,<direction>,<speed>,<battery>,<feature_type>[,<log_message>]
```

### Field Descriptions

1. **timestamp**: Milliseconds since Arduino startup
2. **front-distance**: Front sensor distance reading in cm
3. **left-distance**: Left sensor distance reading in cm
4. **right-distance**: Right sensor distance reading in cm
5. **direction**: Current robot movement direction
   - `F` = Forward
   - `B` = Backward
   - `L` = Left turn
   - `R` = Right turn
   - `S` = Stopped
6. **speed**: Current robot speed in cm/s (optional, default: 0)
7. **battery**: Battery level percentage (optional, default: 100)
8. **feature_type**: Type of maze feature (optional)
   - Examples: `PATH`, `JUNCTION`, `DEADEND`, `TURN_LEFT`, `TURN_RIGHT`, etc.
9. **log_message**: Optional log message (rarely used in this format)

## Log Message Format

For sending log messages without sensor data (Prefix required):

```
LOG:<your log message here>
```

The ESP8266 will properly format this and send it to the server.

## Robot Identification

To set the robot name that will be displayed on the interface:

```
IDENTITY:<robot name>
```

Example: `IDENTITY:KashCow`

## Examples

### Standard Sensor Reading
```
1500,30.5,12.0,18.5,F,15.0,95.0,PATH
```

### Junction Detection
```
2500,25.0,40.0,42.0,S,0.0,94.0,JUNCTION
```

### Dead End Detection
```
3500,10.0,8.0,7.0,S,0.0,93.0,DEADEND
```

### Turn Execution
```
4500,35.0,12.0,40.0,R,10.0,92.0,TURN_RIGHT
```

### Log Message
```
LOG:Robot has ended the run
```

## Tips

1. Send sensor data every 100-200ms for smooth visualization
2. Send important log messages as needed using the LOG: prefix
3. Set the robot identity once at startup with IDENTITY:


