#!/usr/bin/env python3
"""
Mock drone implementation for testing without hardware.
Simulates MAVLink responses and drone behavior.
"""

import math
from typing import Dict, Any
from enum import Enum
import time


class DroneMode(Enum):
    """Drone flight modes."""

    MANUAL = "MANUAL"
    GUIDED = "GUIDED"
    AUTO = "AUTO"
    LAND = "LAND"
    RTL = "RTL"
    LOITER = "LOITER"


class MockDrone:
    """Simulates a drone for testing purposes."""

    def __init__(self):
        # State
        self.is_armed = False
        self.mode = DroneMode.MANUAL
        self.battery_percent = 100.0
        self.battery_drain_rate = 0.1  # % per second when armed

        # Position (lat, lon, alt in meters relative to home)
        self.home_position = {"lat": 37.4419, "lon": -122.1430, "alt": 0}
        self.position = self.home_position.copy()
        self.heading = 0.0  # degrees

        # Target position
        self.target_position = None
        self.target_reached = True

        # Movement parameters
        self.climb_rate = 3.0  # m/s
        self.horizontal_speed = 5.0  # m/s
        self.turn_rate = 90.0  # deg/s

        # Safety
        self.geofence = {"max_altitude": 120, "max_distance": 500}
        self.rtl_battery_threshold = 20

        # Timing
        self.last_update_time = time.time()

    def arm(self) -> bool:
        """Arm the drone."""
        if self.battery_percent < 25:
            return False

        self.is_armed = True
        self.mode = DroneMode.GUIDED
        return True

    def disarm(self) -> bool:
        """Disarm the drone."""
        if self.position["alt"] > 0.5:  # Can't disarm in air
            return False

        self.is_armed = False
        self.mode = DroneMode.MANUAL
        return True

    def takeoff(self, altitude: float) -> bool:
        """Initiate takeoff to target altitude."""
        if not self.is_armed:
            return False

        if altitude > self.geofence["max_altitude"]:
            altitude = self.geofence["max_altitude"]

        self.target_position = {
            "lat": self.position["lat"],
            "lon": self.position["lon"],
            "alt": altitude,
        }
        self.target_reached = False
        self.mode = DroneMode.GUIDED
        return True

    def land(self) -> bool:
        """Initiate landing at current position."""
        self.target_position = {
            "lat": self.position["lat"],
            "lon": self.position["lon"],
            "alt": 0,
        }
        self.target_reached = False
        self.mode = DroneMode.LAND
        return True

    def goto(self, lat: float, lon: float, alt: float) -> bool:
        """Fly to a specific position."""
        if not self.is_armed:
            return False

        # Check geofence
        distance = self._calculate_distance(
            self.home_position["lat"], self.home_position["lon"], lat, lon
        )

        if distance > self.geofence["max_distance"]:
            return False

        if alt > self.geofence["max_altitude"]:
            alt = self.geofence["max_altitude"]

        self.target_position = {"lat": lat, "lon": lon, "alt": alt}
        self.target_reached = False
        self.mode = DroneMode.GUIDED
        return True

    def return_to_launch(self) -> bool:
        """Return to home position."""
        self.target_position = {
            "lat": self.home_position["lat"],
            "lon": self.home_position["lon"],
            "alt": 0,
        }
        self.target_reached = False
        self.mode = DroneMode.RTL
        return True

    def update(self):
        """Update drone state (call regularly)."""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Update battery
        if self.is_armed:
            self.battery_percent -= self.battery_drain_rate * dt
            self.battery_percent = max(0, self.battery_percent)

            # Check for low battery RTL
            if (
                self.battery_percent < self.rtl_battery_threshold
                and self.mode != DroneMode.RTL
            ):
                self.return_to_launch()

        # Update position if we have a target
        if self.target_position and not self.target_reached:
            self._move_towards_target(dt)

        # Auto-disarm if landed
        if self.position["alt"] < 0.1 and self.mode == DroneMode.LAND:
            self.disarm()

    def _move_towards_target(self, dt: float):
        """Move drone towards target position."""
        if not self.target_position:
            return

        # Calculate altitude difference
        alt_diff = self.target_position["alt"] - self.position["alt"]

        # Update altitude
        if abs(alt_diff) > 0.1:
            alt_change = self.climb_rate * dt
            if alt_diff < 0:
                alt_change = -alt_change

            if abs(alt_change) > abs(alt_diff):
                self.position["alt"] = self.target_position["alt"]
            else:
                self.position["alt"] += alt_change

        # Calculate horizontal distance
        distance = self._calculate_distance(
            self.position["lat"],
            self.position["lon"],
            self.target_position["lat"],
            self.target_position["lon"],
        )

        # Update horizontal position
        if distance > 0.5:  # 0.5m tolerance
            # Calculate bearing to target
            bearing = self._calculate_bearing(
                self.position["lat"],
                self.position["lon"],
                self.target_position["lat"],
                self.target_position["lon"],
            )

            # Move towards target
            move_distance = min(self.horizontal_speed * dt, distance)

            # Convert to lat/lon changes (simplified for small distances)
            lat_change = move_distance * math.cos(math.radians(bearing)) / 111111.0
            lon_change = (
                move_distance
                * math.sin(math.radians(bearing))
                / (111111.0 * math.cos(math.radians(self.position["lat"])))
            )

            self.position["lat"] += lat_change
            self.position["lon"] += lon_change
            self.heading = bearing
        else:
            # Reached horizontal target
            if abs(alt_diff) < 0.1:
                self.target_reached = True
                if self.mode == DroneMode.RTL and self.position["alt"] < 0.5:
                    self.land()

    def _calculate_distance(
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> float:
        """Calculate distance between two points in meters (simplified)."""
        lat_diff = (lat2 - lat1) * 111111.0
        lon_diff = (lon2 - lon1) * 111111.0 * math.cos(math.radians(lat1))
        return math.sqrt(lat_diff**2 + lon_diff**2)

    def _calculate_bearing(
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> float:
        """Calculate bearing from point 1 to point 2 in degrees."""
        lat_diff = lat2 - lat1
        lon_diff = lon2 - lon1

        bearing = math.degrees(math.atan2(lon_diff, lat_diff))
        return (bearing + 360) % 360

    def get_telemetry(self) -> Dict[str, Any]:
        """Get current telemetry data."""
        return {
            "armed": self.is_armed,
            "mode": self.mode.value,
            "battery_percent": self.battery_percent,
            "position": self.position.copy(),
            "heading": self.heading,
            "altitude": self.position["alt"],
            "groundspeed": self.horizontal_speed if not self.target_reached else 0,
            "airspeed": self.horizontal_speed if not self.target_reached else 0,
            "target_reached": self.target_reached,
        }

    def simulate_scenario(self, scenario: str, value: str = None):
        """Handle simulation commands for testing."""
        if scenario == "BATTERY":
            if value:
                try:
                    self.battery_percent = float(value)
                    self.battery_percent = max(0, min(100, self.battery_percent))
                except ValueError:
                    pass
        elif scenario == "GPS":
            if value == "LOST":
                # Simulate GPS loss by setting mode
                self.mode = DroneMode.LAND
        elif scenario == "WIND":
            if value == "STRONG":
                # Could add position drift here
                pass
        elif scenario == "OBSTACLE":
            if value == "AHEAD":
                # Stop movement
                self.target_reached = True
