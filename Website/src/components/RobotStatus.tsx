
import { Card } from "@/components/ui/card";
import { ActivitySquare, Timer } from "lucide-react";

export interface RobotStatusProps {
  name: string;
  status: "running" | "stopped" | "error";
  speed: number;
  runtime: number;
  connected: boolean;
}

export function RobotStatus({ name, status, speed, runtime, connected }: RobotStatusProps) {
  return (
    <div className="w-full mb-4">
      <div className="flex items-center justify-between mb-2">
        <div className="flex items-center gap-2">
          <h1 className="text-2xl font-bold">{name}</h1>
          <div className="flex items-center gap-1.5">
            <div className={`w-2 h-2 rounded-full ${status === "running" ? "bg-emerald-500" : status === "stopped" ? "bg-amber-500" : "bg-red-500"}`}></div>
            <span className="text-sm text-muted-foreground capitalize">{status}</span>
          </div>
        </div>
        <div className="bg-slate-800 text-slate-100 px-3 py-1 rounded-md flex items-center gap-1.5">
          <div className={`w-2 h-2 rounded-full ${connected ? "bg-emerald-500" : "bg-red-500"}`}></div>
          <span className="text-sm font-medium">Online</span>
        </div>
      </div>
      
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        <Card className="bg-background/50 border-slate-800">
          <div className="p-6">
            <div className="flex items-center gap-3 mb-2 text-muted-foreground">
              <ActivitySquare size={18} />
              <span>Speed</span>
            </div>
            <p className="text-4xl font-bold">{speed} cm/s</p>
          </div>
        </Card>
        
        <Card className="bg-background/50 border-slate-800">
          <div className="p-6">
            <div className="flex items-center gap-3 mb-2 text-muted-foreground">
              <Timer size={18} />
              <span>Runtime</span>
            </div>
            <p className="text-4xl font-bold">{runtime}s</p>
          </div>
        </Card>
      </div>
    </div>
  );
}
