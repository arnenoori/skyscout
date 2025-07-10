'use client'

import React, { useEffect, useRef, useState } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { useROS } from '@/contexts/ROSContext';

interface Position {
  x: number;
  y: number;
}

export function DroneMap() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { subscribeToTopic, connected } = useROS();
  const [dronePosition, setDronePosition] = useState<Position>({ x: 0, y: 0 });
  const [waypoints, setWaypoints] = useState<Position[]>([]);

  useEffect(() => {
    if (!connected) return;

    const unsubPosition = subscribeToTopic('/navigation/current_position', 'geometry_msgs/Point', (message) => {
      setDronePosition({ x: message.x, y: message.y });
    });

    const unsubWaypoint = subscribeToTopic('/navigation/waypoint', 'geometry_msgs/Point', (message) => {
      setWaypoints(prev => [...prev.slice(-9), { x: message.x, y: message.y }]);
    });

    return () => {
      unsubPosition();
      unsubWaypoint();
    };
  }, [subscribeToTopic, connected]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Set up coordinate system (center at middle)
    const centerX = canvas.width / 2;
    const centerY = canvas.height / 2;
    const scale = 2; // 2 pixels per meter

    // Draw grid
    ctx.strokeStyle = '#e5e7eb';
    ctx.lineWidth = 1;
    for (let i = 0; i < canvas.width; i += 20) {
      ctx.beginPath();
      ctx.moveTo(i, 0);
      ctx.lineTo(i, canvas.height);
      ctx.stroke();
    }
    for (let i = 0; i < canvas.height; i += 20) {
      ctx.beginPath();
      ctx.moveTo(0, i);
      ctx.lineTo(canvas.width, i);
      ctx.stroke();
    }

    // Draw origin
    ctx.strokeStyle = '#9ca3af';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(centerX - 10, centerY);
    ctx.lineTo(centerX + 10, centerY);
    ctx.moveTo(centerX, centerY - 10);
    ctx.lineTo(centerX, centerY + 10);
    ctx.stroke();

    // Draw waypoints
    ctx.strokeStyle = '#3b82f6';
    ctx.fillStyle = '#dbeafe';
    waypoints.forEach((wp, index) => {
      const x = centerX + wp.x * scale;
      const y = centerY - wp.y * scale; // Invert Y axis

      ctx.beginPath();
      ctx.arc(x, y, 4, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();

      // Draw path
      if (index > 0) {
        const prevWp = waypoints[index - 1];
        const prevX = centerX + prevWp.x * scale;
        const prevY = centerY - prevWp.y * scale;

        ctx.strokeStyle = '#93c5fd';
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.moveTo(prevX, prevY);
        ctx.lineTo(x, y);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    });

    // Draw drone
    const droneX = centerX + dronePosition.x * scale;
    const droneY = centerY - dronePosition.y * scale;

    ctx.fillStyle = '#ef4444';
    ctx.strokeStyle = '#dc2626';
    ctx.lineWidth = 2;

    // Draw drone as triangle pointing up
    ctx.beginPath();
    ctx.moveTo(droneX, droneY - 8);
    ctx.lineTo(droneX - 6, droneY + 6);
    ctx.lineTo(droneX + 6, droneY + 6);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Draw position text
    ctx.fillStyle = '#374151';
    ctx.font = '12px monospace';
    ctx.fillText(`(${dronePosition.x.toFixed(1)}, ${dronePosition.y.toFixed(1)})`, droneX + 10, droneY - 10);

  }, [dronePosition, waypoints]);

  return (
    <Card>
      <CardHeader>
        <CardTitle className="text-lg">Drone Position</CardTitle>
      </CardHeader>
      <CardContent>
        <canvas
          ref={canvasRef}
          width={400}
          height={300}
          className="w-full border rounded-md bg-gray-50"
        />
        <div className="mt-2 flex items-center gap-4 text-xs text-muted-foreground">
          <div className="flex items-center gap-1">
            <div className="w-3 h-3 bg-red-500 rounded-sm"></div>
            <span>Drone</span>
          </div>
          <div className="flex items-center gap-1">
            <div className="w-3 h-3 bg-blue-500 rounded-full"></div>
            <span>Waypoints</span>
          </div>
          <div className="ml-auto">
            Scale: 1m = 2px
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
