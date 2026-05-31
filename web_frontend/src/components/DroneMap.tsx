'use client';

import { useEffect, useRef, useState } from 'react';
import maplibregl from 'maplibre-gl';
import 'maplibre-gl/dist/maplibre-gl.css';
import { Maximize2, Minimize2, Navigation, Home } from 'lucide-react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { Button } from '@/components/ui/button';
import { useROS } from '@/contexts/ROSContext';
import { useTheme } from 'next-themes';

interface Position {
  x: number;
  y: number;
  z?: number;
}

const DEFAULT_LOCATION: [number, number] = [-122.4194, 37.7749];
const MAX_TRAIL_POINTS = 100;
const MAX_WAYPOINTS = 10;

const DARK_MAP_STYLE: maplibregl.StyleSpecification = {
  version: 8,
  sources: {
    'osm-tiles': {
      type: 'raster',
      tiles: [
        'https://tiles.stadiamaps.com/tiles/alidade_smooth_dark/{z}/{x}/{y}.png',
      ],
      tileSize: 256,
      attribution:
        '&copy; <a href="https://stadiamaps.com/">Stadia Maps</a> &copy; <a href="https://openmaptiles.org/">OpenMapTiles</a> &copy; <a href="http://openstreetmap.org">OpenStreetMap</a> contributors',
    },
  },
  layers: [
    {
      id: 'osm-tiles',
      type: 'raster',
      source: 'osm-tiles',
      minzoom: 0,
      maxzoom: 19,
    },
  ],
};

const LIGHT_MAP_STYLE: maplibregl.StyleSpecification = {
  version: 8,
  sources: {
    'osm-tiles': {
      type: 'raster',
      tiles: [
        'https://tiles.stadiamaps.com/tiles/alidade_smooth/{z}/{x}/{y}.png',
      ],
      tileSize: 256,
      attribution:
        '&copy; <a href="https://stadiamaps.com/">Stadia Maps</a> &copy; <a href="https://openmaptiles.org/">OpenMapTiles</a> contributors &copy; <a href="http://openstreetmap.org">OpenStreetMap</a>',
    },
  },
  layers: [
    {
      id: 'osm-tiles',
      type: 'raster',
      source: 'osm-tiles',
      minzoom: 0,
      maxzoom: 19,
    },
  ],
};

function getMapStyle(theme: string | undefined) {
  return theme === 'light' ? LIGHT_MAP_STYLE : DARK_MAP_STYLE;
}

function localToLatLng(
  x: number,
  y: number,
  userLocation: [number, number] | null
): [number, number] {
  const [refLng, refLat] = userLocation ?? DEFAULT_LOCATION;
  const lat = refLat + y / 111000;
  const lng = refLng + x / (111000 * Math.cos((refLat * Math.PI) / 180));

  return [lng, lat];
}

function createDroneMarkerElement() {
  const element = document.createElement('div');
  element.className = 'drone-marker';
  element.innerHTML = `
    <div class="drone-body">
      <svg width="40" height="40" viewBox="0 0 40 40" fill="none" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
        <g class="drone-icon">
          <circle cx="20" cy="20" r="18" fill="rgba(6, 182, 212, 0.2)" stroke="rgb(6, 182, 212)" stroke-width="2"/>
          <path d="M20 10 L30 25 L20 22 L10 25 Z" fill="rgb(6, 182, 212)"/>
          <circle cx="20" cy="20" r="3" fill="white"/>
        </g>
      </svg>
    </div>
    <div class="drone-pulse"></div>
  `;

  return element;
}

function appendMapMarkerStyles() {
  const style = document.createElement('style');
  style.textContent = `
    .drone-marker { position: relative; width: 40px; height: 40px; }
    .drone-body { position: relative; z-index: 2; }
    .drone-icon {
      filter: drop-shadow(0 0 10px rgba(6, 182, 212, 0.5));
      animation: drone-float 3s ease-in-out infinite;
    }
    .drone-pulse {
      position: absolute;
      top: 50%;
      left: 50%;
      width: 60px;
      height: 60px;
      margin: -30px 0 0 -30px;
      border-radius: 50%;
      background: rgba(6, 182, 212, 0.3);
      animation: pulse 2s ease-out infinite;
    }
    @keyframes drone-float {
      0%, 100% { transform: translateY(0); }
      50% { transform: translateY(-5px); }
    }
    @keyframes pulse {
      0% { transform: scale(0); opacity: 1; }
      100% { transform: scale(2); opacity: 0; }
    }
    @media (prefers-reduced-motion: reduce) {
      .drone-icon,
      .drone-pulse { animation: none; }
    }
    .waypoint-marker {
      width: 24px;
      height: 24px;
      background: rgba(59, 130, 246, 0.8);
      border: 2px solid white;
      border-radius: 50%;
      box-shadow: 0 2px 4px rgba(0, 0, 0, 0.3);
      display: flex;
      align-items: center;
      justify-content: center;
      color: white;
      font-size: 12px;
      font-weight: bold;
    }
  `;
  document.head.appendChild(style);

  return style;
}

function setTrailData(map: maplibregl.Map, coordinates: [number, number][]) {
  const source = map.getSource('trail') as maplibregl.GeoJSONSource | undefined;

  source?.setData({
    type: 'Feature',
    properties: {},
    geometry: { type: 'LineString', coordinates },
  });
}

function addTrailLayer(map: maplibregl.Map, coordinates: [number, number][]) {
  if (!map.getSource('trail')) {
    map.addSource('trail', {
      type: 'geojson',
      data: {
        type: 'Feature',
        properties: {},
        geometry: { type: 'LineString', coordinates },
      },
    });
  }

  if (!map.getLayer('trail')) {
    map.addLayer({
      id: 'trail',
      type: 'line',
      source: 'trail',
      layout: { 'line-join': 'round', 'line-cap': 'round' },
      paint: {
        'line-color': 'rgba(6, 182, 212, 0.8)',
        'line-width': 3,
        'line-blur': 1,
      },
    });
  }
}

function clearWaypointMarkers(markers: maplibregl.Marker[]) {
  markers.forEach((marker) => marker.remove());
  markers.length = 0;
}

function renderWaypointMarkers(
  map: maplibregl.Map,
  markers: maplibregl.Marker[],
  waypoints: Position[],
  userLocation: [number, number] | null
) {
  clearWaypointMarkers(markers);

  waypoints.forEach((waypoint, index) => {
    const element = document.createElement('div');
    element.className = 'waypoint-marker';
    element.textContent = (index + 1).toString();

    markers.push(
      new maplibregl.Marker({ element })
        .setLngLat(localToLatLng(waypoint.x, waypoint.y, userLocation))
        .addTo(map)
    );
  });
}

function DroneMapControls({
  isFullscreen,
  onCenter,
  onReset,
  onToggleFullscreen,
}: {
  isFullscreen: boolean;
  onCenter: () => void;
  onReset: () => void;
  onToggleFullscreen: () => void;
}) {
  return (
    <div className="flex gap-1">
      <Button
        variant="ghost"
        size="icon"
        className="h-8 w-8"
        onClick={onCenter}
        title="Center on drone"
      >
        <Navigation className="h-4 w-4" />
      </Button>
      <Button
        variant="ghost"
        size="icon"
        className="h-8 w-8"
        onClick={onReset}
        title="Reset view"
      >
        <Home className="h-4 w-4" />
      </Button>
      <Button
        variant="ghost"
        size="icon"
        className="h-8 w-8"
        onClick={onToggleFullscreen}
        title={isFullscreen ? 'Exit fullscreen' : 'Enter fullscreen'}
      >
        {isFullscreen ? (
          <Minimize2 className="h-4 w-4" />
        ) : (
          <Maximize2 className="h-4 w-4" />
        )}
      </Button>
    </div>
  );
}

function DroneTelemetryOverlay({ position }: { position: Position }) {
  return (
    <div className="absolute left-4 top-4 rounded-lg p-3 text-sm font-mono glassmorphism">
      <div className="space-y-1">
        <div className="flex items-center gap-2">
          <span className="text-muted-foreground">Position:</span>
          <span className="text-primary">
            {position.x.toFixed(1)}m, {position.y.toFixed(1)}m
          </span>
        </div>
        {position.z !== undefined && (
          <div className="flex items-center gap-2">
            <span className="text-muted-foreground">Altitude:</span>
            <span className="text-primary">{position.z.toFixed(1)}m</span>
          </div>
        )}
      </div>
    </div>
  );
}

export function DroneMap() {
  const mapContainer = useRef<HTMLDivElement>(null);
  const map = useRef<maplibregl.Map | null>(null);
  const droneMarker = useRef<maplibregl.Marker | null>(null);
  const trailCoordinates = useRef<[number, number][]>([]);
  const waypointMarkers = useRef<maplibregl.Marker[]>([]);
  const waypoints = useRef<Position[]>([]);
  const userLocation = useRef<[number, number] | null>(null);
  const mapReady = useRef(false);
  const themeRef = useRef<string | undefined>(undefined);
  const { subscribeToTopic, connected, connectionMode } = useROS();
  const { theme } = useTheme();
  const [dronePosition, setDronePosition] = useState<Position>({
    x: 0,
    y: 0,
    z: 0,
  });
  const [isFullscreen, setIsFullscreen] = useState(false);

  const updateDroneMarker = (position: Position) => {
    const currentMap = map.current;
    const marker = droneMarker.current;

    if (!currentMap || !marker) return;

    const lngLat = localToLatLng(position.x, position.y, userLocation.current);
    marker.setLngLat(lngLat);
    trailCoordinates.current.push(lngLat);

    if (trailCoordinates.current.length > MAX_TRAIL_POINTS) {
      trailCoordinates.current.shift();
    }

    if (mapReady.current) {
      setTrailData(currentMap, trailCoordinates.current);
    }
  };

  useEffect(() => {
    themeRef.current = theme;
  }, [theme]);

  useEffect(() => {
    navigator.geolocation?.getCurrentPosition(
      (position) => {
        userLocation.current = [
          position.coords.longitude,
          position.coords.latitude,
        ];
        map.current?.setCenter(localToLatLng(0, 0, userLocation.current));

        if (map.current && mapReady.current) {
          renderWaypointMarkers(
            map.current,
            waypointMarkers.current,
            waypoints.current,
            userLocation.current
          );
        }
      },
      (error) => console.log('Geolocation error:', error)
    );
  }, []);

  useEffect(() => {
    const container = mapContainer.current;
    if (!container || map.current) return;

    const activeWaypointMarkers = waypointMarkers.current;
    let activeMap: maplibregl.Map | null = null;
    let activeDroneMarker: maplibregl.Marker | null = null;
    let activeStyleElement: HTMLStyleElement | null = null;
    let resizeTimer: ReturnType<typeof setTimeout> | null = null;

    const handleLoad = () => {
      if (!activeMap) return;

      mapReady.current = true;
      addTrailLayer(activeMap, trailCoordinates.current);
      resizeTimer = setTimeout(() => activeMap?.resize(), 100);
    };

    const initializeTimer = setTimeout(() => {
      activeMap = new maplibregl.Map({
        container,
        style: getMapStyle(themeRef.current),
        center: localToLatLng(0, 0, userLocation.current),
        zoom: 18,
        pitch: 0,
        bearing: 0,
      });
      map.current = activeMap;

      activeMap.addControl(new maplibregl.NavigationControl(), 'top-right');
      activeMap.addControl(new maplibregl.ScaleControl(), 'bottom-right');

      activeStyleElement = appendMapMarkerStyles();
      activeDroneMarker = new maplibregl.Marker({
        element: createDroneMarkerElement(),
      })
        .setLngLat(localToLatLng(0, 0, userLocation.current))
        .addTo(activeMap);
      droneMarker.current = activeDroneMarker;

      activeMap.on('load', handleLoad);
    }, 100);

    return () => {
      clearTimeout(initializeTimer);
      if (resizeTimer) clearTimeout(resizeTimer);
      activeMap?.off('load', handleLoad);
      clearWaypointMarkers(activeWaypointMarkers);
      activeDroneMarker?.remove();
      activeMap?.remove();
      activeStyleElement?.remove();
    };
  }, []);

  useEffect(() => {
    const currentMap = map.current;
    if (!currentMap || !mapReady.current) return;

    const restoreTrail = () =>
      addTrailLayer(currentMap, trailCoordinates.current);
    currentMap.once('styledata', restoreTrail);
    currentMap.setStyle(getMapStyle(theme));

    return () => {
      currentMap.off('styledata', restoreTrail);
    };
  }, [theme]);

  useEffect(() => {
    if (!connected) return;

    const unsubPosition = subscribeToTopic(
      '/navigation/current_position',
      'geometry_msgs/Point',
      (message) => {
        const point = message as { x: number; y: number; z: number };
        const nextPosition = { x: point.x, y: point.y, z: point.z };
        setDronePosition(nextPosition);
        updateDroneMarker(nextPosition);
      }
    );

    const unsubWaypoint = subscribeToTopic(
      '/navigation/waypoint',
      'geometry_msgs/Point',
      (message) => {
        const point = message as { x: number; y: number; z: number };
        waypoints.current = [
          ...waypoints.current.slice(1 - MAX_WAYPOINTS),
          { x: point.x, y: point.y, z: point.z },
        ];

        if (map.current && mapReady.current) {
          renderWaypointMarkers(
            map.current,
            waypointMarkers.current,
            waypoints.current,
            userLocation.current
          );
        }
      }
    );

    return () => {
      unsubPosition();
      unsubWaypoint();
    };
  }, [connected, subscribeToTopic]);

  useEffect(() => {
    const handleResize = () => map.current?.resize();

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  const flyTo = (position: Position) => {
    map.current?.flyTo({
      center: localToLatLng(position.x, position.y, userLocation.current),
      zoom: 18,
      pitch: 0,
      bearing: 0,
      duration: 1000,
    });
  };

  const toggleFullscreen = () => {
    setIsFullscreen((current) => !current);
    setTimeout(() => map.current?.resize(), 100);
  };

  return (
    <Card
      className={`relative ${connectionMode === 'mock' ? 'border-yellow-500/50' : ''} ${isFullscreen ? 'fixed inset-4 z-50' : ''} transition-all duration-300`}
    >
      <CardHeader>
        <CardTitle className="flex items-center justify-between text-lg">
          <span>Drone Position</span>
          <div className="flex items-center gap-2">
            {connectionMode === 'mock' && (
              <Badge variant="warning" className="text-xs">
                SIMULATED
              </Badge>
            )}
            <DroneMapControls
              isFullscreen={isFullscreen}
              onCenter={() => flyTo(dronePosition)}
              onReset={() => flyTo({ x: 0, y: 0 })}
              onToggleFullscreen={toggleFullscreen}
            />
          </div>
        </CardTitle>
      </CardHeader>
      <CardContent className="p-0">
        <div
          className={`relative ${isFullscreen ? 'h-full' : 'h-[400px]'} overflow-hidden`}
        >
          <div ref={mapContainer} className="h-full w-full rounded-b-xl" />
          <DroneTelemetryOverlay position={dronePosition} />
        </div>
      </CardContent>
    </Card>
  );
}
