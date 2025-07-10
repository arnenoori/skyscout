'use client'

import React, { useState, useEffect } from 'react';
import { Battery, Navigation, Radio, Zap } from 'lucide-react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { useROS } from '@/contexts/ROSContext';

interface TelemetryData {
  battery: number;
  altitude: number;
  position: { x: number; y: number; z: number };
  mode: string;
  armed: boolean;
}

export function TelemetryDisplay() {
  const { subscribeToTopic, connected } = useROS();
  const [telemetry, setTelemetry] = useState<TelemetryData>({
    battery: 100,
    altitude: 0,
    position: { x: 0, y: 0, z: 0 },
    mode: 'MANUAL',
    armed: false
  });

  useEffect(() => {
    if (!connected) return;

    const unsubBattery = subscribeToTopic('/navigation/battery_level', 'std_msgs/Float32', (message) => {
      setTelemetry(prev => ({ ...prev, battery: Math.round(message.data) }));
    });

    const unsubPosition = subscribeToTopic('/navigation/current_position', 'geometry_msgs/Point', (message) => {
      setTelemetry(prev => ({
        ...prev,
        position: { x: message.x, y: message.y, z: message.z },
        altitude: message.z
      }));
    });

    const unsubArmed = subscribeToTopic('/navigation/armed', 'std_msgs/Bool', (message) => {
      setTelemetry(prev => ({ ...prev, armed: message.data }));
    });

    const unsubStatus = subscribeToTopic('/navigation/status', 'std_msgs/String', (message) => {
      try {
        const status = JSON.parse(message.data);
        if (status.mode) {
          setTelemetry(prev => ({ ...prev, mode: status.mode }));
        }
      } catch (e) {
        console.error('Failed to parse status:', e);
      }
    });

    return () => {
      unsubBattery();
      unsubPosition();
      unsubArmed();
      unsubStatus();
    };
  }, [subscribeToTopic, connected]);

  const getBatteryColor = (level: number) => {
    if (level > 60) return 'text-green-500';
    if (level > 30) return 'text-yellow-500';
    return 'text-red-500';
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle className="text-lg">Telemetry</CardTitle>
      </CardHeader>
      <CardContent className="grid grid-cols-2 gap-4">
        <div className="flex items-center gap-2">
          <Battery className={`h-4 w-4 ${getBatteryColor(telemetry.battery)}`} />
          <div>
            <p className="text-sm text-muted-foreground">Battery</p>
            <p className="font-semibold">{telemetry.battery}%</p>
          </div>
        </div>

        <div className="flex items-center gap-2">
          <Navigation className="h-4 w-4 text-blue-500" />
          <div>
            <p className="text-sm text-muted-foreground">Altitude</p>
            <p className="font-semibold">{telemetry.altitude.toFixed(1)}m</p>
          </div>
        </div>

        <div className="flex items-center gap-2">
          <Radio className={`h-4 w-4 ${connected ? 'text-green-500' : 'text-red-500'}`} />
          <div>
            <p className="text-sm text-muted-foreground">Status</p>
            <p className="font-semibold">{connected ? 'Connected' : 'Disconnected'}</p>
          </div>
        </div>

        <div className="flex items-center gap-2">
          <Zap className={`h-4 w-4 ${telemetry.armed ? 'text-red-500' : 'text-gray-400'}`} />
          <div>
            <p className="text-sm text-muted-foreground">Mode</p>
            <p className="font-semibold">{telemetry.armed ? telemetry.mode : 'DISARMED'}</p>
          </div>
        </div>

        <div className="col-span-2 mt-2">
          <p className="text-sm text-muted-foreground">Position</p>
          <p className="font-mono text-sm">
            X: {telemetry.position.x.toFixed(1)}m,
            Y: {telemetry.position.y.toFixed(1)}m,
            Z: {telemetry.position.z.toFixed(1)}m
          </p>
        </div>
      </CardContent>
    </Card>
  );
}
