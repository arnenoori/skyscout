'use client'

import React, { createContext, useContext, useEffect, useState, useCallback } from 'react';
import ROSLIB from 'roslib';

interface ROSContextType {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  connect: () => void;
  disconnect: () => void;
  sendCommand: (command: string) => void;
  subscribeToTopic: (topicName: string, messageType: string, callback: (message: any) => void) => () => void;
}

const ROSContext = createContext<ROSContextType | undefined>(undefined);

export function ROSProvider({ children }: { children: React.ReactNode }) {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [connected, setConnected] = useState(false);

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

  // Auto-connect on mount
  useEffect(() => {
    connect();
    return () => {
      disconnect();
    };
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <ROSContext.Provider value={{ ros, connected, connect, disconnect, sendCommand, subscribeToTopic }}>
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
