'use client'

import React from 'react';
import { Wifi, WifiOff, Loader2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Card, CardContent } from '@/components/ui/card';
import { useROS } from '@/contexts/ROSContext';

export function ConnectionStatus() {
  const { connected, connect, disconnect, connectionMode } = useROS();
  const [isConnecting, setIsConnecting] = React.useState(false);

  const handleConnect = async () => {
    setIsConnecting(true);
    connect();
    // Give it 2 seconds to connect
    setTimeout(() => setIsConnecting(false), 2000);
  };

  return (
    <Card>
      <CardContent className="p-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            {connected ? (
              <Wifi className="h-5 w-5 text-green-500" />
            ) : (
              <WifiOff className="h-5 w-5 text-red-500" />
            )}
            <div>
              <p className="font-medium">ROS Connection</p>
              <p className="text-sm text-muted-foreground">
                {connected ? `Connected (${connectionMode} mode)` : 'Not connected'}
              </p>
            </div>
          </div>

          {!connected ? (
            <Button
              onClick={handleConnect}
              disabled={isConnecting}
              size="sm"
            >
              {isConnecting ? (
                <>
                  <Loader2 className="h-4 w-4 mr-2 animate-spin" />
                  Connecting...
                </>
              ) : (
                'Connect'
              )}
            </Button>
          ) : (
            <Button
              onClick={disconnect}
              variant="outline"
              size="sm"
            >
              Disconnect
            </Button>
          )}
        </div>

        {!connected && !isConnecting && (
          <div className="mt-3 text-xs text-muted-foreground">
            <p>Make sure rosbridge is running:</p>
            <code className="block mt-1 p-1 bg-muted rounded">
              ros2 launch rosbridge_server rosbridge_websocket_launch.xml
            </code>
          </div>
        )}
      </CardContent>
    </Card>
  );
}
