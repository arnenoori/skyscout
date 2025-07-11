'use client'

import React, { useState } from 'react';
import { Send, Square } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Card, CardContent } from '@/components/ui/card';
import { useROS } from '@/contexts/ROSContext';

interface CommandInputProps {
  onCommandSent?: (command: string) => void;
}

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
            <button
              onClick={() => setCommand("Search for missing person in the park using spiral pattern")}
              className="text-xs text-left p-2 rounded hover:bg-muted transition-colors"
              disabled={!connected}
            >
              🔍 Search: &quot;Find missing person&quot;
            </button>
            <button
              onClick={() => setCommand("Inspect the building roof for damage")}
              className="text-xs text-left p-2 rounded hover:bg-muted transition-colors"
              disabled={!connected}
            >
              🏢 Inspect: &quot;Check building roof&quot;
            </button>
            <button
              onClick={() => setCommand("Count cars in the parking lot")}
              className="text-xs text-left p-2 rounded hover:bg-muted transition-colors"
              disabled={!connected}
            >
              🚗 Count: &quot;Count vehicles&quot;
            </button>
            <button
              onClick={() => setCommand("Patrol the property perimeter for security")}
              className="text-xs text-left p-2 rounded hover:bg-muted transition-colors"
              disabled={!connected}
            >
              🔒 Patrol: &quot;Security check&quot;
            </button>
            <button
              onClick={() => setCommand("Emergency response to GPS coordinates 37.7749, -122.4194")}
              className="text-xs text-left p-2 rounded hover:bg-muted transition-colors"
              disabled={!connected}
            >
              🚨 Emergency: &quot;Rapid response&quot;
            </button>
            <button
              onClick={() => setCommand("Survey the agricultural field for crop health")}
              className="text-xs text-left p-2 rounded hover:bg-muted transition-colors"
              disabled={!connected}
            >
              🌾 Survey: &quot;Check crops&quot;
            </button>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
