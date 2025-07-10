'use client'

import React, { createContext, useContext, useEffect, useState, useCallback } from 'react';
import ROSLIB from 'roslib';

interface ROSContextType {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  connectionMode: 'mock' | 'real';
  connect: () => void;
  disconnect: () => void;
  sendCommand: (command: string) => void;
  subscribeToTopic: (topicName: string, messageType: string, callback: (message: any) => void) => () => void;
  setConnectionMode: (mode: 'mock' | 'real') => void;
}

const ROSContext = createContext<ROSContextType | undefined>(undefined);

export function ROSProvider({ children }: { children: React.ReactNode }) {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [connected, setConnected] = useState(false);
  const [connectionMode, setConnectionModeState] = useState<'mock' | 'real'>(() => {
    // Load from localStorage on init
    if (typeof window !== 'undefined') {
      return (localStorage.getItem('skyscout-connection-mode') as 'mock' | 'real') || 'mock';
    }
    return 'mock';
  });

  const connect = useCallback(() => {
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://localhost:9090' // Default rosbridge websocket URL
    });

    rosInstance.on('connection', () => {
      console.log('Connected to rosbridge websocket');
      setConnected(true);
    });

    rosInstance.on('error', (error) => {
      console.error('Error connecting to rosbridge:', error);
      setConnected(false);
    });

    rosInstance.on('close', () => {
      console.log('Connection to rosbridge closed');
      setConnected(false);
    });

    setRos(rosInstance);
  }, []);

  const disconnect = useCallback(() => {
    if (ros) {
      ros.close();
      setRos(null);
      setConnected(false);
    }
  }, [ros]);

  const sendCommand = useCallback((command: string) => {
    if (!ros || !connected) {
      console.error('Not connected to ROS');
      return;
    }

    const commandTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/web_command',
      messageType: 'std_msgs/String'
    });

    const message = new ROSLIB.Message({
      data: command
    });

    commandTopic.publish(message);
  }, [ros, connected]);

  const subscribeToTopic = useCallback((topicName: string, messageType: string, callback: (message: any) => void) => {
    if (!ros) {
      console.error('ROS not initialized');
      return () => {};
    }

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: topicName,
      messageType: messageType
    });

    topic.subscribe(callback);

    // Return unsubscribe function
    return () => {
      topic.unsubscribe(callback);
    };
  }, [ros]);

  const setConnectionMode = useCallback((mode: 'mock' | 'real') => {
    setConnectionModeState(mode);
    localStorage.setItem('skyscout-connection-mode', mode);

    // Call service to update navigation bridge
    if (ros && connected) {
      const service = new ROSLIB.Service({
        ros: ros,
        name: '/set_connection_mode',
        serviceType: 'std_srvs/SetBool'
      });

      const request = new ROSLIB.ServiceRequest({
        data: mode === 'mock' // true for mock, false for real
      });

      service.callService(request, (result) => {
        console.log('Connection mode updated:', result);
      });
    }
  }, [ros, connected]);

  // Auto-connect on mount
  useEffect(() => {
    connect();
    return () => {
      disconnect();
    };
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  // Update connection mode when connected
  useEffect(() => {
    if (connected) {
      setConnectionMode(connectionMode);
    }
  }, [connected]); // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <ROSContext.Provider value={{ ros, connected, connectionMode, connect, disconnect, sendCommand, subscribeToTopic, setConnectionMode }}>
      {children}
    </ROSContext.Provider>
  );
}

export function useROS() {
  const context = useContext(ROSContext);
  if (context === undefined) {
    throw new Error('useROS must be used within a ROSProvider');
  }
  return context;
}
