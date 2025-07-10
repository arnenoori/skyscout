'use client'

import React from 'react';
import { Plane, Cpu } from 'lucide-react';
import { Badge } from '@/components/ui/badge';
import { Switch } from '@/components/ui/switch';
import { useROS } from '@/contexts/ROSContext';

export function ConnectionModeSelector() {
  const { connectionMode, setConnectionMode, connected } = useROS();
  const isMock = connectionMode === 'mock';

  const handleToggle = (checked: boolean) => {
    setConnectionMode(checked ? 'real' : 'mock');
  };

  return (
    <div className="flex items-center gap-3">
      <div className="flex items-center gap-2">
        <Cpu className={`h-4 w-4 ${isMock ? 'text-primary' : 'text-muted-foreground'}`} />
        <span className={`text-sm ${isMock ? 'font-medium' : 'text-muted-foreground'}`}>
          Mock
        </span>
      </div>

      <Switch
        checked={!isMock}
        onCheckedChange={handleToggle}
        disabled={!connected}
      />

      <div className="flex items-center gap-2">
        <Plane className={`h-4 w-4 ${!isMock ? 'text-primary' : 'text-muted-foreground'}`} />
        <span className={`text-sm ${!isMock ? 'font-medium' : 'text-muted-foreground'}`}>
          Real
        </span>
      </div>

      {isMock && (
        <Badge variant="warning" className="ml-2">
          SIMULATION MODE
        </Badge>
      )}
    </div>
  );
}
