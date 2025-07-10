'use client'

import React, { useState, useEffect } from 'react';
import { CheckCircle, Circle, AlertCircle, Loader2, Plane } from 'lucide-react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { useROS } from '@/contexts/ROSContext';

interface MissionState {
  state: string;
  message: string;
  missionType?: string;
  targetDescription?: string;
  progress?: number;
}

const StateIcon = ({ state }: { state: string }) => {
  switch (state) {
    case 'IDLE':
      return <Circle className="h-4 w-4 text-gray-400" />;
    case 'PLANNING':
      return <Loader2 className="h-4 w-4 text-blue-500 animate-spin" />;
    case 'EXECUTING':
      return <Plane className="h-4 w-4 text-yellow-500" />;
    case 'COMPLETED':
      return <CheckCircle className="h-4 w-4 text-green-500" />;
    case 'ABORTED':
      return <AlertCircle className="h-4 w-4 text-red-500" />;
    default:
      return <Circle className="h-4 w-4 text-gray-400" />;
  }
};

export function MissionStatus() {
  const { subscribeToTopic, connected } = useROS();
  const [mission, setMission] = useState<MissionState>({
    state: 'IDLE',
    message: 'Waiting for command...'
  });
  const [missionPlan, setMissionPlan] = useState<any>(null);

  useEffect(() => {
    if (!connected) return;

    const unsubStatus = subscribeToTopic('/mission_status', 'std_msgs/String', (message) => {
      try {
        const status = JSON.parse(message.data);
        setMission(status);
      } catch (e) {
        console.error('Failed to parse mission status:', e);
      }
    });

    const unsubPlan = subscribeToTopic('/mission_plan', 'std_msgs/String', (message) => {
      try {
        const plan = JSON.parse(message.data);
        setMissionPlan(plan);
      } catch (e) {
        console.error('Failed to parse mission plan:', e);
      }
    });

    return () => {
      unsubStatus();
      unsubPlan();
    };
  }, [subscribeToTopic, connected]);

  return (
    <Card>
      <CardHeader>
        <CardTitle className="text-lg flex items-center gap-2">
          <StateIcon state={mission.state} />
          Mission Status
        </CardTitle>
      </CardHeader>
      <CardContent>
        <div className="space-y-3">
          <div>
            <p className="text-sm text-muted-foreground">State</p>
            <p className="font-semibold">{mission.state}</p>
          </div>

          <div>
            <p className="text-sm text-muted-foreground">Message</p>
            <p className="text-sm">{mission.message}</p>
          </div>

          {missionPlan && (
            <>
              <div>
                <p className="text-sm text-muted-foreground">Mission Type</p>
                <p className="font-semibold capitalize">{missionPlan.mission_type}</p>
              </div>

              <div>
                <p className="text-sm text-muted-foreground">Target</p>
                <p className="text-sm">{missionPlan.target_description}</p>
              </div>

              <div>
                <p className="text-sm text-muted-foreground">Pattern</p>
                <p className="text-sm capitalize">{missionPlan.flight_pattern}</p>
              </div>

              {missionPlan.parameters && (
                <div>
                  <p className="text-sm text-muted-foreground">Parameters</p>
                  <p className="text-sm">
                    Alt: {missionPlan.parameters.altitude}m,
                    Speed: {missionPlan.parameters.speed}m/s
                  </p>
                </div>
              )}
            </>
          )}
        </div>
      </CardContent>
    </Card>
  );
}
