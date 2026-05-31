'use client';

import React, { useState } from 'react';
import {
  Building2,
  Car,
  Search,
  Send,
  Shield,
  Siren,
  Sprout,
  Square,
} from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Card, CardContent } from '@/components/ui/card';
import { useROS } from '@/contexts/ROSContext';

interface CommandInputProps {
  onCommandSent?: (command: string) => void;
}

const EXAMPLE_COMMANDS = [
  {
    label: 'Search: "Find missing person"',
    command: 'Search for missing person in the park using spiral pattern',
    Icon: Search,
  },
  {
    label: 'Inspect: "Check building roof"',
    command: 'Inspect the building roof for damage',
    Icon: Building2,
  },
  {
    label: 'Count: "Count vehicles"',
    command: 'Count cars in the parking lot',
    Icon: Car,
  },
  {
    label: 'Patrol: "Security check"',
    command: 'Patrol the property perimeter for security',
    Icon: Shield,
  },
  {
    label: 'Emergency: "Rapid response"',
    command: 'Emergency response to GPS coordinates 37.7749, -122.4194',
    Icon: Siren,
  },
  {
    label: 'Survey: "Check crops"',
    command: 'Survey the agricultural field for crop health',
    Icon: Sprout,
  },
];

export function CommandInput({ onCommandSent }: CommandInputProps) {
  const [command, setCommand] = useState('');
  const [isProcessing, setIsProcessing] = useState(false);
  const { sendCommand, connected } = useROS();

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!command.trim() || !connected || isProcessing) return;

    setIsProcessing(true);
    sendCommand(command);

    if (onCommandSent) {
      onCommandSent(command);
    }

    setCommand('');
    // Simulate processing time
    setTimeout(() => setIsProcessing(false), 2000);
  };

  const handleEmergencyStop = () => {
    sendCommand('EMERGENCY STOP');
  };

  return (
    <Card>
      <CardContent className="p-4">
        <form onSubmit={handleSubmit} className="flex gap-2">
          <Input
            type="text"
            placeholder="Tell the drone what to do... (e.g., 'Find all red cars in the area')"
            value={command}
            onChange={(e) => setCommand(e.target.value)}
            disabled={!connected || isProcessing}
            className="flex-1"
          />
          <Button
            type="submit"
            disabled={!connected || isProcessing || !command.trim()}
            size="icon"
          >
            {isProcessing ? (
              <div className="h-4 w-4 animate-spin rounded-full border-2 border-current border-t-transparent" />
            ) : (
              <Send className="h-4 w-4" />
            )}
          </Button>
          <Button
            type="button"
            variant="destructive"
            size="icon"
            onClick={handleEmergencyStop}
            disabled={!connected}
            title="Emergency Stop"
          >
            <Square className="h-4 w-4" />
          </Button>
        </form>
        {!connected && (
          <p className="text-sm text-muted-foreground mt-2">
            Connect to ROS to send commands
          </p>
        )}

        {/* Example Commands */}
        <div className="mt-4 space-y-2">
          <p className="text-sm text-muted-foreground">Example commands:</p>
          <div className="grid grid-cols-2 gap-2">
            {EXAMPLE_COMMANDS.map(
              ({ label, command: exampleCommand, Icon }) => (
                <button
                  key={label}
                  type="button"
                  onClick={() => setCommand(exampleCommand)}
                  className="flex items-center gap-2 rounded p-2 text-left text-xs transition-colors hover:bg-muted"
                  disabled={!connected}
                >
                  <Icon className="h-3.5 w-3.5 shrink-0 text-muted-foreground" />
                  <span>{label}</span>
                </button>
              )
            )}
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
