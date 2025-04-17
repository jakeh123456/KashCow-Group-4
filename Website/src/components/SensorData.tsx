
import { Card } from "@/components/ui/card";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  ResponsiveContainer,
  Tooltip,
} from "recharts";

interface SensorDataProps {
  leftSensor: number[];
  rightSensor: number[];
  frontSensor: number[];
  timePoints: number[];
}

export function SensorData({ 
  leftSensor, 
  rightSensor, 
  frontSensor,
  timePoints 
}: SensorDataProps) {
  const data = timePoints.map((time, index) => ({
    time,
    left: leftSensor[index],
    right: rightSensor[index],
    front: frontSensor[index],
  }));

  return (
    <Card className="data-card">
      <h3 className="text-lg font-semibold mb-2">Sensor Readings</h3>
      <div className="h-[250px] w-full">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data}>
            <CartesianGrid strokeDasharray="3 3" stroke="hsl(var(--secondary))" />
            <XAxis 
              dataKey="time" 
              stroke="hsl(var(--muted-foreground))"
              label={{ value: 'Time (s)', position: 'insideBottom', offset: -5 }}
            />
            <YAxis 
              stroke="hsl(var(--muted-foreground))"
              label={{ value: 'Distance (cm)', angle: -90, position: 'insideLeft' }}
            />
            <Tooltip 
              contentStyle={{ 
                backgroundColor: "hsl(var(--card))",
                borderColor: "hsl(var(--border))",
              }}
            />
            <Line 
              type="monotone" 
              dataKey="left" 
              stroke="#8b5cf6" 
              strokeWidth={2}
              dot={false}
            />
            <Line 
              type="monotone" 
              dataKey="right" 
              stroke="#06b6d4" 
              strokeWidth={2}
              dot={false}
            />
            <Line 
              type="monotone" 
              dataKey="front" 
              stroke="#10b981" 
              strokeWidth={2}
              dot={false}
            />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </Card>
  );
}
