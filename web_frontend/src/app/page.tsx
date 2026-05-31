import type { Metadata } from 'next';
import { SkyScoutDashboard } from '@/components/SkyScoutDashboard';

export const metadata: Metadata = {
  title: 'SkyScout Dashboard',
  description:
    'Command and monitor ROS2 drone missions from a Next.js dashboard.',
};

export default function Home() {
  return <SkyScoutDashboard />;
}
