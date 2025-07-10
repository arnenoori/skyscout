'use client'

import React from 'react';
import { Clock } from 'lucide-react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';

interface Command {
  id: string;
  text: string;
  timestamp: Date;
}

interface CommandHistoryProps {
  commands: Command[];
}

export function CommandHistory({ commands }: CommandHistoryProps) {
  return (
    <Card>
      <CardHeader>
        <CardTitle className="text-lg flex items-center gap-2">
          <Clock className="h-4 w-4" />
          Command History
        </CardTitle>
      </CardHeader>
      <CardContent>
        <div className="space-y-2 max-h-64 overflow-y-auto">
          {commands.length === 0 ? (
            <p className="text-sm text-muted-foreground">No commands sent yet</p>
          ) : (
            commands.slice().reverse().map((cmd) => (
              <div key={cmd.id} className="p-2 rounded-md bg-muted/50">
                <p className="text-sm">{cmd.text}</p>
                <p className="text-xs text-muted-foreground">
                  {cmd.timestamp.toLocaleTimeString()}
                </p>
              </div>
            ))
          )}
        </div>
      </CardContent>
    </Card>
  );
}
