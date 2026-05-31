'use client';

import type { ReactNode } from 'react';
import {
  createContext,
  use,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from 'react';
import ROSLIB from 'roslib';

interface ROSContextType {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  connectionMode: 'mock' | 'real';
  connect: () => void;
  disconnect: () => void;
  sendCommand: (command: string) => void;
  subscribeToTopic: (
    topicName: string,
    messageType: string,
    callback: (message: Record<string, unknown>) => void
  ) => () => void;
  setConnectionMode: (mode: 'mock' | 'real') => void;
}

const ROSContext = createContext<ROSContextType | undefined>(undefined);

function getStoredConnectionMode(): 'mock' | 'real' {
  if (typeof window === 'undefined') {
    return 'mock';
  }

  const savedMode = window.localStorage.getItem('skyscout-connection-mode');
  return savedMode === 'real' || savedMode === 'mock' ? savedMode : 'mock';
}

function updateConnectionModeService(ros: ROSLIB.Ros, mode: 'mock' | 'real') {
  const service = new ROSLIB.Service({
    ros,
    name: '/set_connection_mode',
    serviceType: 'std_srvs/SetBool',
  });

  const request = new ROSLIB.ServiceRequest({
    data: mode === 'mock',
  });

  service.callService(request, (result) => {
    console.log('Connection mode updated:', result);
  });
}

export function ROSProvider({ children }: { children: ReactNode }) {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [connected, setConnected] = useState(false);
  const [connectionMode, setConnectionModeState] = useState<'mock' | 'real'>(
    getStoredConnectionMode
  );
  const connectionModeRef = useRef(connectionMode);

  const connect = useCallback(() => {
    try {
      const rosInstance = new ROSLIB.Ros({
        url: 'ws://localhost:9090', // Default rosbridge websocket URL
      });

      rosInstance.on('connection', () => {
        console.log('Connected to rosbridge websocket');
        setConnected(true);
        updateConnectionModeService(rosInstance, connectionModeRef.current);
      });

      rosInstance.on('error', (error) => {
        console.warn(
          'ROS connection error (this is normal if rosbridge is not running):',
          error
        );
        setConnected(false);
      });

      rosInstance.on('close', () => {
        console.log('Connection to rosbridge closed');
        setConnected(false);
      });

      setRos(rosInstance);
    } catch (error) {
      console.warn('Failed to create ROS instance:', error);
      setConnected(false);
    }
  }, []);

  const disconnect = useCallback(() => {
    setRos((currentRos) => {
      currentRos?.close();
      return null;
    });
    setConnected(false);
  }, []);

  const sendCommand = useCallback(
    (command: string) => {
      if (!ros || !connected) {
        console.error('Not connected to ROS');
        return;
      }

      const commandTopic = new ROSLIB.Topic({
        ros,
        name: '/web_command',
        messageType: 'std_msgs/String',
      });

      const message = new ROSLIB.Message({
        data: command,
      });

      commandTopic.publish(message);
    },
    [ros, connected]
  );

  const subscribeToTopic = useCallback(
    (
      topicName: string,
      messageType: string,
      callback: (message: Record<string, unknown>) => void
    ) => {
      if (!ros) {
        console.error('ROS not initialized');
        return () => {};
      }

      const topic = new ROSLIB.Topic({
        ros,
        name: topicName,
        messageType: messageType,
      });

      topic.subscribe(callback);

      // Return unsubscribe function
      return () => {
        topic.unsubscribe(callback);
      };
    },
    [ros]
  );

  const setConnectionMode = useCallback(
    (mode: 'mock' | 'real') => {
      connectionModeRef.current = mode;
      setConnectionModeState(mode);
      window.localStorage.setItem('skyscout-connection-mode', mode);

      if (ros && connected) {
        updateConnectionModeService(ros, mode);
      }
    },
    [ros, connected]
  );

  useEffect(() => {
    connectionModeRef.current = connectionMode;
  }, [connectionMode]);

  useEffect(() => {
    const shouldAutoConnect =
      process.env.NODE_ENV === 'production' ||
      process.env.NEXT_PUBLIC_AUTO_CONNECT === 'true';

    if (!shouldAutoConnect) {
      return;
    }

    const timer = window.setTimeout(connect, 1000);

    return () => {
      window.clearTimeout(timer);
      disconnect();
    };
  }, [connect, disconnect]);

  const value = useMemo(
    () => ({
      ros,
      connected,
      connectionMode,
      connect,
      disconnect,
      sendCommand,
      subscribeToTopic,
      setConnectionMode,
    }),
    [
      ros,
      connected,
      connectionMode,
      connect,
      disconnect,
      sendCommand,
      subscribeToTopic,
      setConnectionMode,
    ]
  );

  return <ROSContext.Provider value={value}>{children}</ROSContext.Provider>;
}

export function useROS() {
  const context = use(ROSContext);
  if (context === undefined) {
    throw new Error('useROS must be used within a ROSProvider');
  }
  return context;
}
