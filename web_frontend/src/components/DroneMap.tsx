'use client'

import React, { useEffect, useRef, useState } from 'react';
import maplibregl from 'maplibre-gl';
import 'maplibre-gl/dist/maplibre-gl.css';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { useROS } from '@/contexts/ROSContext';
import { useTheme } from 'next-themes';
import { Maximize2, Minimize2, Navigation, Home } from 'lucide-react';
import { Button } from '@/components/ui/button';

interface Position {
  x: number;
  y: number;
  z?: number;
}

const DARK_MAP_STYLE: maplibregl.StyleSpecification = {
  version: 8,
  sources: {
    'osm-tiles': {
      type: 'raster',
      tiles: [
        'https://tiles.stadiamaps.com/tiles/alidade_smooth_dark/{z}/{x}/{y}.png'
      ],
      tileSize: 256,
      attribution: '&copy; <a href="https://stadiamaps.com/">Stadia Maps</a> &copy; <a href="https://openmaptiles.org/">OpenMapTiles</a> &copy; <a href="http://openstreetmap.org">OpenStreetMap</a> contributors'
    }
  },
  layers: [
    {
      id: 'osm-tiles',
      type: 'raster',
      source: 'osm-tiles',
      minzoom: 0,
      maxzoom: 19
    }
  ]
};

const LIGHT_MAP_STYLE: maplibregl.StyleSpecification = {
  version: 8,
  sources: {
    'osm-tiles': {
      type: 'raster',
      tiles: [
        'https://tiles.stadiamaps.com/tiles/alidade_smooth/{z}/{x}/{y}.png'
      ],
      tileSize: 256,
      attribution: '&copy; <a href="https://stadiamaps.com/">Stadia Maps</a> &copy; <a href="https://openmaptiles.org/">OpenMapTiles</a> &copy; <a href="http://openstreetmap.org">OpenStreetMap</a> contributors'
    }
  },
  layers: [
    {
      id: 'osm-tiles',
      type: 'raster',
      source: 'osm-tiles',
      minzoom: 0,
      maxzoom: 19
    }
  ]
};

export function DroneMap() {
  const mapContainer = useRef<HTMLDivElement>(null);
  const map = useRef<maplibregl.Map | null>(null);
  const droneMarker = useRef<maplibregl.Marker | null>(null);
  const trailCoordinates = useRef<[number, number][]>([]);
  const { subscribeToTopic, connected, connectionMode } = useROS();
  const { theme } = useTheme();
  const [dronePosition, setDronePosition] = useState<Position>({ x: 0, y: 0, z: 0 });
  const [waypoints, setWaypoints] = useState<Position[]>([]);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [isMapReady, setIsMapReady] = useState(false);
  const [userLocation, setUserLocation] = useState<[number, number] | null>(null);
  const styleElementRef = useRef<HTMLStyleElement | null>(null);

  // Convert local coordinates to lat/lng (assuming meters from a reference point)
  const localToLatLng = (x: number, y: number): [number, number] => {
    // Use user location as reference if available, otherwise default to San Francisco
    const refLat = userLocation ? userLocation[1] : 37.7749;
    const refLng = userLocation ? userLocation[0] : -122.4194;

    // Approximate conversion (1 degree latitude ≈ 111km)
    const lat = refLat + (y / 111000);
    const lng = refLng + (x / (111000 * Math.cos(refLat * Math.PI / 180)));

    return [lng, lat];
  };

  // Get user location on mount
  useEffect(() => {
    if (navigator.geolocation) {
      navigator.geolocation.getCurrentPosition(
        (position) => {
          setUserLocation([position.coords.longitude, position.coords.latitude]);
        },
        (error) => {
          console.log('Geolocation error:', error);
          // Keep default location if geolocation fails
        }
      );
    }
  }, []);

  // Initialize map
  useEffect(() => {
    if (!mapContainer.current || map.current) return;

    // Wait for container to be ready
    const initializeMap = () => {
      if (!mapContainer.current) return;

      // Create map instance
      map.current = new maplibregl.Map({
        container: mapContainer.current,
        style: theme === 'dark' ? DARK_MAP_STYLE : LIGHT_MAP_STYLE,
        center: localToLatLng(0, 0),
        zoom: 18,
        pitch: 0,  // Birds eye view
        bearing: 0
      });

    // Add navigation controls
    map.current.addControl(new maplibregl.NavigationControl(), 'top-right');
    map.current.addControl(new maplibregl.ScaleControl(), 'bottom-right');

    // Create custom drone marker
    const el = document.createElement('div');
    el.className = 'drone-marker';
    el.innerHTML = `
      <div class="drone-body">
        <svg width="40" height="40" viewBox="0 0 40 40" fill="none" xmlns="http://www.w3.org/2000/svg">
          <g class="drone-icon">
            <circle cx="20" cy="20" r="18" fill="rgba(6, 182, 212, 0.2)" stroke="rgb(6, 182, 212)" stroke-width="2"/>
            <path d="M20 10 L30 25 L20 22 L10 25 Z" fill="rgb(6, 182, 212)"/>
            <circle cx="20" cy="20" r="3" fill="white"/>
          </g>
        </svg>
      </div>
      <div class="drone-pulse"></div>
    `;

    // Add CSS for drone marker
    const style = document.createElement('style');
    styleElementRef.current = style;
    style.textContent = `
      .drone-marker {
        position: relative;
        width: 40px;
        height: 40px;
      }
      .drone-body {
        position: relative;
        z-index: 2;
      }
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
        0% {
          transform: scale(0);
          opacity: 1;
        }
        100% {
          transform: scale(2);
          opacity: 0;
        }
      }
      .waypoint-marker {
        width: 24px;
        height: 24px;
        background: rgba(59, 130, 246, 0.8);
        border: 2px solid white;
        border-radius: 50%;
        box-shadow: 0 2px 4px rgba(0,0,0,0.3);
        display: flex;
        align-items: center;
        justify-content: center;
        color: white;
        font-size: 12px;
        font-weight: bold;
      }
    `;
    document.head.appendChild(style);

      droneMarker.current = new maplibregl.Marker({ element: el })
        .setLngLat(localToLatLng(0, 0))
        .addTo(map.current!);

      map.current!.on('load', () => {
      setIsMapReady(true);

      // Add trail source and layer
      map.current!.addSource('trail', {
        type: 'geojson',
        data: {
          type: 'Feature',
          properties: {},
          geometry: {
            type: 'LineString',
            coordinates: []
          }
        }
      });

        map.current!.addLayer({
          id: 'trail',
          type: 'line',
          source: 'trail',
          layout: {
            'line-join': 'round',
            'line-cap': 'round'
          },
          paint: {
            'line-color': 'rgba(6, 182, 212, 0.8)',
            'line-width': 3,
            'line-blur': 1
          }
        });

        // Force resize after load
        setTimeout(() => {
          map.current?.resize();
        }, 100);
      });
    };

    // Initialize map after a short delay to ensure container is ready
    setTimeout(initializeMap, 100);

    return () => {
      if (map.current) {
        map.current.remove();
        map.current = null;
      }
      if (styleElementRef.current) {
        styleElementRef.current.remove();
        styleElementRef.current = null;
      }
    };
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  // Update map style when theme changes
  useEffect(() => {
    if (map.current && isMapReady) {
      map.current.setStyle(theme === 'dark' ? DARK_MAP_STYLE : LIGHT_MAP_STYLE);

      // Re-add trail source and layer after style change
      map.current.once('styledata', () => {
        if (!map.current!.getSource('trail')) {
          map.current!.addSource('trail', {
            type: 'geojson',
            data: {
              type: 'Feature',
              properties: {},
              geometry: {
                type: 'LineString',
                coordinates: trailCoordinates.current
              }
            }
          });

          map.current!.addLayer({
            id: 'trail',
            type: 'line',
            source: 'trail',
            layout: {
              'line-join': 'round',
              'line-cap': 'round'
            },
            paint: {
              'line-color': 'rgba(6, 182, 212, 0.8)',
              'line-width': 3,
              'line-blur': 1
            }
          });
        }
      });
    }
  }, [theme, isMapReady]);

  // Subscribe to ROS topics
  useEffect(() => {
    if (!connected) return;

    const unsubPosition = subscribeToTopic('/navigation/current_position', 'geometry_msgs/Point', (message) => {
      const point = message as { x: number; y: number; z: number };
      setDronePosition({ x: point.x, y: point.y, z: point.z });
    });

    const unsubWaypoint = subscribeToTopic('/navigation/waypoint', 'geometry_msgs/Point', (message) => {
      const point = message as { x: number; y: number; z: number };
      setWaypoints(prev => [...prev.slice(-9), { x: point.x, y: point.y, z: point.z }]);
    });

    return () => {
      unsubPosition();
      unsubWaypoint();
    };
  }, [subscribeToTopic, connected]);

  // Update drone position on map
  useEffect(() => {
    if (!droneMarker.current || !map.current) return;

    const lngLat = localToLatLng(dronePosition.x, dronePosition.y);
    droneMarker.current.setLngLat(lngLat);

    // Update trail
    trailCoordinates.current.push(lngLat);
    if (trailCoordinates.current.length > 100) {
      trailCoordinates.current.shift();
    }

    if (isMapReady && map.current.getSource('trail')) {
      (map.current.getSource('trail') as maplibregl.GeoJSONSource).setData({
        type: 'Feature',
        properties: {},
        geometry: {
          type: 'LineString',
          coordinates: trailCoordinates.current
        }
      });
    }
  }, [dronePosition, isMapReady, localToLatLng]);

  // Update waypoints on map
  useEffect(() => {
    if (!map.current || !isMapReady) return;

    // Remove existing waypoint markers
    const existingMarkers = document.querySelectorAll('.waypoint-marker');
    existingMarkers.forEach(marker => marker.remove());

    // Add new waypoint markers
    waypoints.forEach((wp, index) => {
      const el = document.createElement('div');
      el.className = 'waypoint-marker';
      el.textContent = (index + 1).toString();

      new maplibregl.Marker({ element: el })
        .setLngLat(localToLatLng(wp.x, wp.y))
        .addTo(map.current!);
    });
  }, [waypoints, isMapReady, localToLatLng]);

  const centerOnDrone = () => {
    if (map.current) {
      map.current.flyTo({
        center: localToLatLng(dronePosition.x, dronePosition.y),
        zoom: 18,
        pitch: 0,
        bearing: 0,
        duration: 1000
      });
    }
  };

  const resetView = () => {
    if (map.current) {
      map.current.flyTo({
        center: localToLatLng(0, 0),
        zoom: 18,
        pitch: 0,
        bearing: 0,
        duration: 1000
      });
    }
  };

  const toggleFullscreen = () => {
    setIsFullscreen(prev => {
      const newFullscreen = !prev;
      // Force map resize after state change
      setTimeout(() => {
        map.current?.resize();
      }, 100);
      return newFullscreen;
    });
  };

  // Handle window resize
  useEffect(() => {
    const handleResize = () => {
      map.current?.resize();
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  // Force resize when fullscreen changes
  useEffect(() => {
    if (map.current && isMapReady) {
      setTimeout(() => {
        map.current?.resize();
      }, 300);
    }
  }, [isFullscreen, isMapReady]);

  // Update map when user location changes
  useEffect(() => {
    if (map.current && userLocation && isMapReady) {
      map.current.setCenter(localToLatLng(0, 0));
    }
  }, [userLocation, isMapReady]); // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <Card className={`relative ${connectionMode === 'mock' ? 'border-yellow-500/50' : ''} ${isFullscreen ? 'fixed inset-4 z-50' : ''} transition-all duration-300`}>
      <CardHeader>
        <CardTitle className="text-lg flex items-center justify-between">
          <span>Drone Position</span>
          <div className="flex items-center gap-2">
            {connectionMode === 'mock' && (
              <Badge variant="warning" className="text-xs">SIMULATED</Badge>
            )}
            <div className="flex gap-1">
              <Button
                variant="ghost"
                size="icon"
                className="h-8 w-8"
                onClick={centerOnDrone}
                title="Center on drone"
              >
                <Navigation className="h-4 w-4" />
              </Button>
              <Button
                variant="ghost"
                size="icon"
                className="h-8 w-8"
                onClick={resetView}
                title="Reset view"
              >
                <Home className="h-4 w-4" />
              </Button>
              <Button
                variant="ghost"
                size="icon"
                className="h-8 w-8"
                onClick={toggleFullscreen}
                title={isFullscreen ? "Exit fullscreen" : "Enter fullscreen"}
              >
                {isFullscreen ? <Minimize2 className="h-4 w-4" /> : <Maximize2 className="h-4 w-4" />}
              </Button>
            </div>
          </div>
        </CardTitle>
      </CardHeader>
      <CardContent className="p-0">
        <div className={`relative ${isFullscreen ? 'h-full' : 'h-[400px]'} overflow-hidden`}>
          <div ref={mapContainer} className="w-full h-full rounded-b-xl" />

          {/* Telemetry overlay */}
          <div className="absolute top-4 left-4 glassmorphism rounded-lg p-3 text-sm font-mono">
            <div className="space-y-1">
              <div className="flex items-center gap-2">
                <span className="text-muted-foreground">Position:</span>
                <span className="text-primary">
                  {dronePosition.x.toFixed(1)}m, {dronePosition.y.toFixed(1)}m
                </span>
              </div>
              {dronePosition.z !== undefined && (
                <div className="flex items-center gap-2">
                  <span className="text-muted-foreground">Altitude:</span>
                  <span className="text-primary">{dronePosition.z.toFixed(1)}m</span>
                </div>
              )}
            </div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
