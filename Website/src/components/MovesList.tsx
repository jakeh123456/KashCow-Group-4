
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { ArrowUp, ArrowRight, ArrowLeft, ArrowDown, Square } from "lucide-react";

export interface MovesListProps {
  moves: Array<{
    direction: "F" | "R" | "L" | "B" | "S";
    timestamp: number;
    featureType?: string;
  }>;
}

export function MovesList({ moves }: MovesListProps) {
  const getDirectionIcon = (direction: "F" | "R" | "L" | "B" | "S") => {
    switch (direction) {
      case "F":
        return <ArrowUp className="h-4 w-4" />;
      case "R":
        return <ArrowRight className="h-4 w-4" />;
      case "L":
        return <ArrowLeft className="h-4 w-4" />;
      case "B":
        return <ArrowDown className="h-4 w-4" />;
      case "S":
        return <Square className="h-4 w-4" />;
    }
  };

  const getDirectionText = (direction: "F" | "R" | "L" | "B" | "S") => {
    switch (direction) {
      case "F":
        return "Forward";
      case "R":
        return "Right";
      case "L":
        return "Left";
      case "B":
        return "Back";
      case "S":
        return "Stop";
    }
  };

  const getDirectionColor = (direction: "F" | "R" | "L" | "B" | "S") => {
    switch (direction) {
      case "F":
        return "bg-emerald-900/80 text-emerald-400";
      case "R":
        return "bg-cyan-900/80 text-cyan-400";
      case "L":
        return "bg-purple-900/80 text-purple-400";
      case "B":
        return "bg-amber-900/80 text-amber-400";
      case "S":
        return "bg-red-900/80 text-red-400";
    }
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle className="text-xl font-bold">Movement History</CardTitle>
      </CardHeader>
      <CardContent>
        <div className="space-y-4">          
          <div className="space-y-2 max-h-[400px] overflow-y-auto pr-1">
            {moves.length === 0 ? (
              <div className="text-sm text-muted-foreground text-center py-4">
                No movement recorded yet
              </div>
            ) : (
              moves.map((move, index) => (
                <div 
                  key={index} 
                  className={`flex items-center justify-between p-2 rounded-md text-sm ${getDirectionColor(move.direction)}`}
                >
                  <div className="flex items-center gap-2">
                    <span className="p-1 rounded-full bg-black/20">
                      {getDirectionIcon(move.direction)}
                    </span>
                    <span className="font-medium">
                      {getDirectionText(move.direction)}
                      {move.featureType && <span className="ml-2 text-xs opacity-75">({move.featureType})</span>}
                    </span>
                  </div>
                  <span className="text-xs opacity-90">
                    {new Date(move.timestamp).toLocaleTimeString()}
                  </span>
                </div>
              ))
            )}
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
