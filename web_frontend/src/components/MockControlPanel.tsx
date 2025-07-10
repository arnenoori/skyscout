'use client'

import React from 'react';
import { AlertTriangle, Battery, Navigation2, Wind } from 'lucide-react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { useROS } from '@/contexts/ROSContext';

export function MockControlPanel() {
  const { sendCommand, connectionMode } = useROS();

  if (connectionMode !== 'mock') {
    return null;
  }

  const simulateScenario = (scenario: string) => {
    // Send special commands that the mock drone can interpret
    switch (scenario) {
      case 'low-battery':
        sendCommand('SIMULATE:BATTERY:15');
        break;
      case 'gps-loss':
        sendCommand('SIMULATE:GPS:LOST');
        break;
      case 'strong-wind':
        sendCommand('SIMULATE:WIND:STRONG');
        break;
      case 'obstacle':
        sendCommand('SIMULATE:OBSTACLE:AHEAD');
        break;
    }
  };

  return (
    <Card className="border-yellow-500/50">
      <CardHeader>
        <CardTitle className="text-lg flex items-center gap-2">
          <AlertTriangle className="h-4 w-4 text-yellow-500" />
          Mock Simulation Controls
        </CardTitle>
      </CardHeader>
      <CardContent>
        <div className="grid grid-cols-2 gap-2">
          <Button
            variant="outline"
            size="sm"
            onClick={() => simulateScenario('low-battery')}
            className="justify-start"
          >
            <Battery className="h-4 w-4 mr-2" />
            Low Battery
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={() => simulateScenario('gps-loss')}
            className="justify-start"
          >
            <Navigation2 className="h-4 w-4 mr-2" />
            GPS Loss
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={() => simulateScenario('strong-wind')}
            className="justify-start"
          >
            <Wind className="h-4 w-4 mr-2" />
            Strong Wind
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={() => simulateScenario('obstacle')}
            className="justify-start"
          >
            <AlertTriangle className="h-4 w-4 mr-2" />
            Obstacle
          </Button>
        </div>
        <p className="text-xs text-muted-foreground mt-3">
          Simulate various scenarios to test system behavior
        </p>
      </CardContent>
    </Card>
  );
}
