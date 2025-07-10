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
            Connecting to drone system...
          </p>
        )}
      </CardContent>
    </Card>
  );
}
