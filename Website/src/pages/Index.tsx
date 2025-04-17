
import { useState, useEffect } from 'react';
import { RobotStatus } from '@/components/RobotStatus';
import { SensorData } from '@/components/SensorData';
import { MazeMap } from '@/components/MazeMap';
import { SerialMonitor } from '@/components/SerialMonitor';
import { MovesList } from '@/components/MovesList';
import { toast } from '@/components/ui/use-toast';

type RobotStatusType = "running" | "stopped" | "error";
type MessageType = "info" | "error" | "warning";
type MoveDirection = "F" | "R" | "L" | "B" | "S";

interface AppData {
  robotStatus: {
    name: string;
    status: RobotStatusType;
    speed: number;
    runtime: number;
  };
  sensorData: {
    leftSensor: number[];
    rightSensor: number[];
    frontSensor: number[];
    timePoints: number[];
  };
  mazeRuns: Array<{
    moves: Array<{
      direction: string;
      type: string;
      timestamp?: number;
      featureType?: string;
    }>;
  }>;
  currentMazeRun: Array<{
    direction: string;
    type: string;
    timestamp: number;
    featureType?: string;
  }>;
  serialMonitor: {
    messages: Array<{
      timestamp: number;
      content: string;
      type: MessageType;
    }>;
  };
  moves: Array<{
    direction: MoveDirection;
    timestamp: number;
    featureType?: string;
  }>;
}

const initialData: AppData = {
  robotStatus: {
    name: "Connecting...",
    status: "stopped" as RobotStatusType,
    speed: 0,
    runtime: 0
  },
  sensorData: {
    leftSensor: Array(100).fill(0),
    rightSensor: Array(100).fill(0),
    frontSensor: Array(100).fill(0),
    timePoints: Array.from({ length: 100 }, (_, i) => i)
  },
  mazeRuns: [],
  currentMazeRun: [],
  serialMonitor: {
    messages: [
      { timestamp: Date.now(), content: 'System initialized', type: 'info' as MessageType }
    ]
  },
  moves: [] as Array<{ direction: MoveDirection; timestamp: number; featureType?: string }>
};

const Index = () => {
  const [data, setData] = useState<AppData>(initialData);
  const [selectedRun, setSelectedRun] = useState(0);
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [viewMode, setViewMode] = useState<'current' | 'saved'>('current');

  useEffect(() => {
    connectToWebSocketServer();

    return () => {
      if (ws) {
        ws.close();
      }
    };
  }, []);

  const connectToWebSocketServer = () => {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.hostname;
    const port = 8080;
    const websocket = new WebSocket(`${protocol}//${host}:${port}`);

    websocket.onopen = () => {
      setIsConnected(true);
      setWs(websocket);
      toast({
        title: "Connected to server",
        description: "Successfully connected to the WebSocket server",
      });
    };

    websocket.onclose = () => {
      setIsConnected(false);
      setWs(null);
      toast({
        title: "Disconnected from server",
        description: "Connection to WebSocket server lost",
        variant: "destructive",
      });
    };

    websocket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        
        if (message.type === 'sensor_data') {
          updateDataWithSensorInfo(message);
        } 
        else if (message.type === 'move') {
          addMove(message.direction, message.timestamp, message.featureType);
        }
        else if (message.type === 'log') {
          // Make sure we properly handle log messages
          console.log('Received log message:', message);
          addLogMessage(message.content, message.msgType || 'info', message.timestamp);
        }
        else if (message.type === 'status') {
          updateRobotStatus(message);
        }
        else if (message.type === 'identity') {
          // Handle robot identity/name update
          updateRobotName(message.name);
        }
        else if (message.type === 'connection_status') {
          setIsConnected(message.connected);
          
          if (message.connected) {
            toast({
              title: "Robot connected",
              description: "The robot is now connected to the server",
            });
          } else {
            // Reset name to "Connecting..." when disconnected
            updateRobotName("Connecting...");
            
            toast({
              title: "Robot disconnected",
              description: "The robot is not connected to the server",
              variant: "destructive",
            });
          }
        }
        else if (message.type === 'error') {
          toast({
            title: "Error",
            description: message.message,
            variant: "destructive",
          });
        }
      } catch (error) {
        console.error('Error parsing WebSocket message:', error);
      }
    };

    websocket.onerror = (error) => {
      console.error('WebSocket error:', error);
      toast({
        title: "Connection Error",
        description: "Failed to connect to the WebSocket server",
        variant: "destructive",
      });
    };

    return websocket;
  };

  // Add new function to update robot name
  const updateRobotName = (name: string) => {
    setData(prev => ({
      ...prev,
      robotStatus: {
        ...prev.robotStatus,
        name: name
      }
    }));
  };

  const updateDataWithSensorInfo = (sensorData: {
    left: number;
    right: number;
    front: number;
    timestamp: number;
    direction?: string;
    featureType?: string;
  }) => {
    setData(prev => ({
      ...prev,
      sensorData: {
        leftSensor: [...prev.sensorData.leftSensor.slice(1), sensorData.left],
        rightSensor: [...prev.sensorData.rightSensor.slice(1), sensorData.right],
        frontSensor: [...prev.sensorData.frontSensor.slice(1), sensorData.front],
        timePoints: prev.sensorData.timePoints
      }
    }));

    // Only process direction for maze mapping if it's not '0' (from a log message)
    if (sensorData.direction && 
        sensorData.direction !== '0') {
        
      // Normalize direction to uppercase
      const direction = sensorData.direction.toUpperCase();
      
      // Only process valid movement directions
      if (['F', 'L', 'R', 'B'].includes(direction)) {
        // Use the server-provided feature type if available
        let type = sensorData.featureType?.toLowerCase() || 'path';
        
        // If feature type is not provided by the server, determine it based on sensor readings
        // This is the legacy code, kept as a fallback but commented out
        /*
        if (!sensorData.featureType) {
          if (sensorData.front < 10 && sensorData.left < 15 && sensorData.right < 15) {
            type = 'deadend';
          } 
          else if (sensorData.front > 10 && sensorData.left < 15 && sensorData.right < 15) {
            type = 't-junction';
          }
          else if ((sensorData.left < 15 && sensorData.right > 15) || 
                  (sensorData.left > 15 && sensorData.right < 15)) {
            type = 'turn';
          }
          else if (sensorData.left > 15 && sensorData.right > 15) {
            type = 'junction';
          }
        }
        */
        
        addMazePoint(direction, type, sensorData.timestamp);
      }
    }
  };

  const addMove = (direction: MoveDirection, timestamp: number, featureType?: string) => {
    setData(prev => ({
      ...prev,
      moves: [...prev.moves, { direction, timestamp, featureType }].slice(-100)
    }));
  };

  const addMazePoint = (direction: string, type: string, timestamp: number) => {
    setData(prev => ({
      ...prev,
      currentMazeRun: [
        ...prev.currentMazeRun,
        { direction, type, timestamp, featureType: type }
      ]
    }));
  };

  const addLogMessage = (content: string, type: MessageType, timestamp: number) => {
    console.log(`Adding log message: ${content}, type: ${type}, time: ${timestamp}`);
    setData(prev => ({
      ...prev,
      serialMonitor: {
        messages: [
          ...prev.serialMonitor.messages,
          { timestamp, content, type }
        ].slice(-100)
      }
    }));
  };

  const updateRobotStatus = (status: {
    name?: string;
    status?: RobotStatusType;
    speed?: number;
    runtime?: number;
  }) => {
    setData(prev => ({
      ...prev,
      robotStatus: {
        ...prev.robotStatus,
        ...(status.name && { name: status.name }),
        ...(status.status && { status: status.status }),
        ...(status.speed !== undefined && { speed: status.speed }),
        ...(status.runtime !== undefined && { runtime: status.runtime })
      }
    }));
  };

  const sendCommand = (command: string) => {
    if (ws && isConnected) {
      ws.send(JSON.stringify({ command }));
    } else {
      toast({
        title: "Connection Error",
        description: "Not connected to the robot",
        variant: "destructive",
      });
    }
  };

  const saveCurrentMazeRun = () => {
    if (data.currentMazeRun.length === 0) {
      toast({
        title: "No maze data",
        description: "There is no maze run data to save",
        variant: "destructive",
      });
      return;
    }

    setData(prev => ({
      ...prev,
      mazeRuns: [...prev.mazeRuns, { moves: [...prev.currentMazeRun] }]
    }));

    toast({
      title: "Maze run saved",
      description: "The current maze run has been saved",
    });
  };

  const clearCurrentMazeRun = () => {
    setData(prev => ({
      ...prev,
      currentMazeRun: []
    }));

    toast({
      title: "Maze run cleared",
      description: "The current maze run has been cleared",
    });
  };

  return (
    <div className="min-h-screen p-4 md:p-8 space-y-6 animate-fade-in">
      <div className="flex flex-col md:flex-row justify-between items-start md:items-center">
        <RobotStatus {...data.robotStatus} connected={isConnected} />
        
        <div className="flex flex-col md:flex-row gap-2">
          <div className="flex gap-2">
            <button
              onClick={saveCurrentMazeRun}
              className="px-3 py-1 bg-green-700 text-green-100 rounded-md text-sm mt-2 md:mt-0"
            >
              Save Maze Run
            </button>
            <button
              onClick={clearCurrentMazeRun}
              className="px-3 py-1 bg-red-700 text-red-100 rounded-md text-sm mt-2 md:mt-0"
            >
              Clear Maze
            </button>
          </div>
        </div>
      </div>
      
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
        <SensorData {...data.sensorData} />
        <div>
          <div className="flex items-center gap-2 mb-2">
            <button
              onClick={() => setViewMode('current')}
              className={`px-4 py-2 rounded-md transition-colors ${
                viewMode === 'current'
                  ? 'bg-primary text-primary-foreground'
                  : 'bg-secondary hover:bg-secondary/80'
              }`}
            >
              Current Run
            </button>
            <button
              onClick={() => setViewMode('saved')}
              className={`px-4 py-2 rounded-md transition-colors ${
                viewMode === 'saved'
                  ? 'bg-primary text-primary-foreground'
                  : 'bg-secondary hover:bg-secondary/80'
              }`}
            >
              Saved Runs
            </button>
            
            {viewMode === 'saved' && data.mazeRuns.length > 0 && (
              <div className="flex ml-4">
                {data.mazeRuns.map((_, index) => (
                  <button
                    key={index}
                    onClick={() => setSelectedRun(index)}
                    className={`px-4 py-2 rounded-md transition-colors ${
                      selectedRun === index
                        ? 'bg-primary text-primary-foreground'
                        : 'bg-secondary hover:bg-secondary/80'
                    }`}
                  >
                    Run {index + 1}
                  </button>
                ))}
              </div>
            )}
          </div>
          
          {viewMode === 'current' ? (
            <MazeMap moves={data.currentMazeRun} liveMode={true} />
          ) : data.mazeRuns.length > 0 ? (
            <MazeMap moves={data.mazeRuns[selectedRun].moves} />
          ) : (
            <div className="flex items-center justify-center h-[300px] bg-muted rounded-lg">
              <p className="text-muted-foreground">No saved maze runs yet</p>
            </div>
          )}
        </div>
      </div>
      
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
        <SerialMonitor messages={data.serialMonitor.messages} />
        <MovesList moves={data.moves} />
      </div>
    </div>
  );
};

export default Index;
