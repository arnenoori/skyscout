'use client'

import { useState } from 'react';
import { CommandInput } from '@/components/CommandInput';
import { TelemetryDisplay } from '@/components/TelemetryDisplay';
import { MissionStatus } from '@/components/MissionStatus';
import { CommandHistory } from '@/components/CommandHistory';
import { DroneMap } from '@/components/DroneMap';
import { ConnectionModeSelector } from '@/components/ConnectionModeSelector';
import { ConnectionStatus } from '@/components/ConnectionStatus';
import { MockControlPanel } from '@/components/MockControlPanel';
import { ROSProvider } from '@/contexts/ROSContext';

export default function Home() {
  const [commands, setCommands] = useState<Array<{ id: string; text: string; timestamp: Date }>>([]);

  const handleCommandSent = (command: string) => {
    setCommands(prev => [...prev, {
      id: Date.now().toString(),
      text: command,
      timestamp: new Date()
    }]);
  };

  return (
    <ROSProvider>
      <div className="min-h-screen bg-background">
        <header className="border-b">
          <div className="container mx-auto px-4 py-4">
            <div className="flex items-center justify-between">
              <div>
                <h1 className="text-2xl font-bold">SkyScout</h1>
                <p className="text-sm text-muted-foreground">Natural Language Drone Control</p>
              </div>
              <ConnectionModeSelector />
            </div>
          </div>
        </header>

        <main className="container mx-auto px-4 py-6">
          <div className="grid gap-6 md:grid-cols-1 lg:grid-cols-3">
            {/* Left Column - Command and Status */}
            <div className="space-y-6 lg:col-span-2">
              <ConnectionStatus />
              <CommandInput onCommandSent={handleCommandSent} />
              <div className="grid gap-6 md:grid-cols-1 lg:grid-cols-1">
                <MissionStatus />
                <DroneMap />
              </div>
            </div>

            {/* Right Column - Telemetry and History */}
            <div className="space-y-6 lg:col-span-1">
              <TelemetryDisplay />
              <CommandHistory commands={commands} />
              <MockControlPanel />
            </div>
          </div>
        </main>
      </div>
    </ROSProvider>
  );
}
