import { useEffect, useRef, useState } from 'react';
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight } from 'lucide-react';

// Update the interface to match the actual data structure and robot's decision points
interface MazePoint {
  direction: 'F' | 'L' | 'R' | 'B' | string;
  type: 'path' | 'junction' | 't-junction' | 'deadend' | 'turn' | 'start' | 'end' | string;
  timestamp?: number;
  featureType?: string;
}

interface MazeMapProps {
  moves: MazePoint[];
  scale?: number;
  liveMode?: boolean;
}

export function MazeMap({ moves, scale = 1, liveMode = false }: MazeMapProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [viewBox, setViewBox] = useState({ minX: 0, minY: 0, maxX: 400, maxY: 400 });
  const [autoZoom, setAutoZoom] = useState(true);
  const [showDirectionArrows, setShowDirectionArrows] = useState(true);
  const [showLegend, setShowLegend] = useState(true);

  const calculatePoints = (moves: MazePoint[]) => {
    let x = 200; // Start at center
    let y = 350; // Start near bottom
    let angle = -Math.PI/2; // Start facing up
    const step = 50; // Distance between points
    
    const points = [{x, y, type: 'start', direction: 'F'}];
    let minX = x;
    let minY = y;
    let maxX = x;
    let maxY = y;
    
    moves.forEach((move, index) => {
      // Normalize direction to uppercase
      const dir = move.direction.toUpperCase();
      
      // Update angle based on direction
      if (dir === 'L') {
        angle -= Math.PI/2;
      } else if (dir === 'R') {
        angle += Math.PI/2;
      } else if (dir === 'B') {
        angle += Math.PI; // 180 degree turn
      }
      
      // Add a point (even turns need a point for visualization)
      x += Math.cos(angle) * step;
      y += Math.sin(angle) * step;
      
      // Track min/max for auto-sizing
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
      
      points.push({
        x, 
        y, 
        type: move.type || 'path',
        direction: dir
      });
    });
    
    // Mark the last point as end if it's the last one
    if (points.length > 1) {
      points[points.length - 1].type = 'end';
    }
    
    return { points, bounds: { minX, minY, maxX, maxY } };
  };

  const drawPoint = (
    ctx: CanvasRenderingContext2D,
    point: {x: number, y: number, type?: string, direction?: string},
    color: string,
    size: number
  ) => {
    if (!point) return; // Guard against undefined points
    
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(point.x * scale, point.y * scale, size, 0, Math.PI * 2);
    ctx.fill();
    
    // Add direction arrow if enabled and there's a direction
    if (showDirectionArrows && point.direction) {
      drawDirectionIndicator(ctx, point, point.direction);
    }
  };

  const drawDirectionIndicator = (
    ctx: CanvasRenderingContext2D, 
    point: {x: number, y: number},
    direction: string
  ) => {
    const arrowSize = 8;
    const arrowX = point.x * scale;
    const arrowY = point.y * scale;
    
    ctx.save();
    ctx.translate(arrowX, arrowY);
    
    // Rotate based on direction
    let angle = 0;
    if (direction === 'R') angle = Math.PI/2;
    else if (direction === 'L') angle = -Math.PI/2;
    else if (direction === 'B') angle = Math.PI;
    
    ctx.rotate(angle);
    
    // Draw arrow
    ctx.fillStyle = "#ffffff";
    ctx.beginPath();
    ctx.moveTo(0, -arrowSize);
    ctx.lineTo(arrowSize/2, 0);
    ctx.lineTo(-arrowSize/2, 0);
    ctx.closePath();
    ctx.fill();
    
    ctx.restore();
  };

  const drawPath = (ctx: CanvasRenderingContext2D, points: {x: number, y: number, type?: string, direction?: string}[]) => {
    if (points.length < 2) return;

    // Draw main path
    ctx.strokeStyle = '#E5DEFF'; // Light purple color
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(points[0].x * scale, points[0].y * scale);

    for (let i = 1; i < points.length; i++) {
      ctx.lineTo(points[i].x * scale, points[i].y * scale);
    }
    ctx.stroke();

    // Draw parallel walls
    ctx.strokeStyle = '#F1F0FB'; // Even lighter purple for walls
    ctx.lineWidth = 1;
    const wallOffset = 10; // Distance of walls from path

    for (let i = 0; i < points.length - 1; i++) {
      const start = points[i];
      const end = points[i + 1];
      
      // Calculate perpendicular vector
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      const length = Math.sqrt(dx * dx + dy * dy);
      const perpX = -dy / length * wallOffset;
      const perpY = dx / length * wallOffset;

      // Draw left wall
      ctx.beginPath();
      ctx.moveTo((start.x + perpX) * scale, (start.y + perpY) * scale);
      ctx.lineTo((end.x + perpX) * scale, (end.y + perpY) * scale);
      ctx.stroke();

      // Draw right wall
      ctx.beginPath();
      ctx.moveTo((start.x - perpX) * scale, (start.y - perpY) * scale);
      ctx.lineTo((end.x - perpX) * scale, (end.y - perpY) * scale);
      ctx.stroke();
    }
  };

  const drawLegend = (ctx: CanvasRenderingContext2D) => {
    const itemHeight = 25;
    const legendX = 20;
    let legendY = 20;
    const legendWidth = 150;
    
    // Draw legend background
    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fillRect(legendX, legendY, legendWidth, itemHeight * 6);
    
    // Draw legend items
    ctx.font = '12px sans-serif';
    ctx.textBaseline = 'middle';
    
    // Start point
    ctx.fillStyle = '#FFD700';
    ctx.beginPath();
    ctx.arc(legendX + 10, legendY + itemHeight/2, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = 'white';
    ctx.fillText('Start', legendX + 25, legendY + itemHeight/2);
    legendY += itemHeight;
    
    // Junction
    ctx.fillStyle = '#D6BCFA';
    ctx.beginPath();
    ctx.arc(legendX + 10, legendY + itemHeight/2, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = 'white';
    ctx.fillText('Junction', legendX + 25, legendY + itemHeight/2);
    legendY += itemHeight;
    
    // T-Junction
    ctx.fillStyle = '#9b87f5';
    ctx.beginPath();
    ctx.arc(legendX + 10, legendY + itemHeight/2, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = 'white';
    ctx.fillText('T-Junction', legendX + 25, legendY + itemHeight/2);
    legendY += itemHeight;
    
    // Deadend
    ctx.fillStyle = '#FFDEE2';
    ctx.beginPath();
    ctx.arc(legendX + 10, legendY + itemHeight/2, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = 'white';
    ctx.fillText('Dead End', legendX + 25, legendY + itemHeight/2);
    legendY += itemHeight;
    
    // Turn
    ctx.fillStyle = '#0EA5E9';
    ctx.beginPath();
    ctx.arc(legendX + 10, legendY + itemHeight/2, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = 'white';
    ctx.fillText('Turn', legendX + 25, legendY + itemHeight/2);
    legendY += itemHeight;
    
    // Current position
    ctx.fillStyle = '#F2FCE2';
    ctx.beginPath();
    ctx.arc(legendX + 10, legendY + itemHeight/2, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = 'white';
    ctx.fillText('Current Position', legendX + 25, legendY + itemHeight/2);
  };

  const centerAndScaleMap = (canvas: HTMLCanvasElement, bounds: {minX: number, minY: number, maxX: number, maxY: number}) => {
    if (!canvas) return 1;
    
    const ctx = canvas.getContext('2d');
    if (!ctx) return 1;
    
    // Add padding to bounds
    const padding = 50;
    bounds.minX -= padding;
    bounds.minY -= padding;
    bounds.maxX += padding;
    bounds.maxY += padding;
    
    // Calculate dimensions
    const width = bounds.maxX - bounds.minX;
    const height = bounds.maxY - bounds.minY;
    
    // Calculate scale to fit canvas while maintaining aspect ratio
    const canvasWidth = canvas.width;
    const canvasHeight = canvas.height;
    
    const scaleX = canvasWidth / width;
    const scaleY = canvasHeight / height;
    
    // Use the smaller scale to ensure everything fits
    const fitScale = Math.min(scaleX, scaleY);
    
    // Set the transform to center the map
    ctx.setTransform(
      fitScale, 0,
      0, fitScale,
      -bounds.minX * fitScale + (canvasWidth - width * fitScale) / 2,
      -bounds.minY * fitScale + (canvasHeight - height * fitScale) / 2
    );
    
    return fitScale;
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas with reset transform
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Calculate points from moves
    const { points, bounds } = calculatePoints(moves);

    // Center and scale map if auto-zoom is enabled
    if (autoZoom) {
      centerAndScaleMap(canvas, bounds);
      setViewBox(bounds);
    }

    // Draw path and walls
    drawPath(ctx, points);

    // Draw special points
    points.forEach((point, index) => {
      if (!point) return; // Skip if point is undefined
      
      if (point.type === 't-junction') {
        drawPoint(ctx, point, '#9b87f5', 6); // Vivid purple for T-junctions
      } else if (point.type === 'junction') {
        drawPoint(ctx, point, '#D6BCFA', 6); // Light purple for junctions
      } else if (point.type === 'deadend') {
        drawPoint(ctx, point, '#FFDEE2', 6); // Light red for deadends
      } else if (point.type === 'turn') {
        drawPoint(ctx, point, '#0EA5E9', 6); // Blue for turns
      } else if (point.type === 'start') {
        drawPoint(ctx, point, '#FFD700', 8); // Gold for start
      } else if (point.type === 'end') {
        drawPoint(ctx, point, '#F97316', 8); // Orange for end
      } else {
        // Regular path points
        drawPoint(ctx, point, '#E5DEFF', 4); // Light purple for regular path
      }
    });

    // Draw current position (last point)
    if (points.length > 0 && liveMode) {
      drawPoint(ctx, points[points.length - 1], '#F2FCE2', 8); // Light green for current position
    }

    // Draw legend if enabled
    if (showLegend) {
      drawLegend(ctx);
    }
  }, [moves, scale, autoZoom, showDirectionArrows, showLegend, liveMode]);

  return (
    <div className="data-card">
      <div className="flex justify-between items-center mb-4">
        <h3 className="text-lg font-semibold">Maze Map</h3>
        <div className="flex items-center space-x-2">
          <label className="text-sm flex items-center">
            <input 
              type="checkbox"
              checked={autoZoom}
              onChange={() => setAutoZoom(!autoZoom)}
              className="mr-2"
            />
            Auto-zoom
          </label>
          <label className="text-sm flex items-center">
            <input 
              type="checkbox"
              checked={showDirectionArrows}
              onChange={() => setShowDirectionArrows(!showDirectionArrows)}
              className="mr-2"
            />
            Arrows
          </label>
          <label className="text-sm flex items-center">
            <input 
              type="checkbox"
              checked={showLegend}
              onChange={() => setShowLegend(!showLegend)}
              className="mr-2"
            />
            Legend
          </label>
        </div>
      </div>
      <div className="relative aspect-square w-full border rounded-lg overflow-hidden">
        <canvas
          ref={canvasRef}
          className="absolute inset-0 w-full h-full"
          width={400}
          height={400}
        />
      </div>
    </div>
  );
}
