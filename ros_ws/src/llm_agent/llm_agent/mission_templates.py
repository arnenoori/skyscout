#!/usr/bin/env python3
"""
Predefined mission templates for common drone operations.
"""

from typing import Dict, Any


class MissionTemplates:
    """Collection of pre-configured mission templates."""

    @staticmethod
    def get_template(template_name: str) -> Dict[str, Any]:
        """Get a mission template by name."""
        templates = {
            "quick_search": MissionTemplates.quick_search(),
            "building_inspection": MissionTemplates.building_inspection(),
            "perimeter_patrol": MissionTemplates.perimeter_patrol(),
            "delivery_direct": MissionTemplates.delivery_direct(),
            "emergency_response": MissionTemplates.emergency_response(),
            "agricultural_survey": MissionTemplates.agricultural_survey(),
            "parking_count": MissionTemplates.parking_count(),
            "3d_mapping": MissionTemplates.mapping_3d(),
        }
        return templates.get(template_name, MissionTemplates.quick_search())

    @staticmethod
    def quick_search() -> Dict[str, Any]:
        """Quick area search for a missing person or object."""
        return {
            "mission_type": "search",
            "target_description": "missing person or object",
            "flight_pattern": "spiral",
            "parameters": {
                "altitude": 50,
                "speed": 8,
                "coverage_area": {"center": {"lat": 0.0, "lon": 0.0}, "radius": 200},
            },
            "safety": {
                "geofence": {"max_altitude": 100, "max_distance": 300},
                "rtl_battery_threshold": 30,
                "weather_check": True,
                "max_wind_speed": 12.0,
            },
        }

    @staticmethod
    def building_inspection() -> Dict[str, Any]:
        """360-degree building or structure inspection."""
        return {
            "mission_type": "inspect",
            "target_description": "building structure and roof",
            "flight_pattern": "circle",
            "parameters": {
                "altitude": 30,
                "speed": 3,
                "coverage_area": {"center": {"lat": 0.0, "lon": 0.0}, "radius": 50},
            },
            "safety": {
                "geofence": {"max_altitude": 60, "max_distance": 100},
                "rtl_battery_threshold": 25,
                "weather_check": True,
                "max_wind_speed": 8.0,
            },
        }

    @staticmethod
    def perimeter_patrol() -> Dict[str, Any]:
        """Security patrol around property perimeter."""
        return {
            "mission_type": "patrol",
            "target_description": "security threats or intrusions",
            "flight_pattern": "perimeter",
            "parameters": {
                "altitude": 40,
                "speed": 5,
                "coverage_area": {"center": {"lat": 0.0, "lon": 0.0}, "radius": 150},
            },
            "safety": {
                "geofence": {"max_altitude": 80, "max_distance": 200},
                "rtl_battery_threshold": 25,
                "weather_check": True,
                "max_wind_speed": 10.0,
            },
        }

    @staticmethod
    def delivery_direct() -> Dict[str, Any]:
        """Direct point-to-point delivery mission."""
        return {
            "mission_type": "delivery",
            "target_description": "delivery location",
            "flight_pattern": "waypoints",
            "parameters": {
                "altitude": 60,
                "speed": 10,
                "coverage_area": {"center": {"lat": 0.0, "lon": 0.0}, "radius": 10},
            },
            "safety": {
                "geofence": {"max_altitude": 100, "max_distance": 1000},
                "rtl_battery_threshold": 35,
                "weather_check": True,
                "max_wind_speed": 8.0,
            },
        }

    @staticmethod
    def emergency_response() -> Dict[str, Any]:
        """Rapid emergency response to location."""
        return {
            "mission_type": "emergency",
            "target_description": "emergency scene assessment",
            "flight_pattern": "waypoints",
            "parameters": {
                "altitude": 80,
                "speed": 10,  # Maximum safe speed
                "coverage_area": {"center": {"lat": 0.0, "lon": 0.0}, "radius": 50},
            },
            "safety": {
                "geofence": {"max_altitude": 120, "max_distance": 2000},
                "rtl_battery_threshold": 20,  # Lower threshold for emergency
                "weather_check": False,  # Override weather for emergency
                "max_wind_speed": 15.0,  # Higher tolerance for emergency
            },
        }

    @staticmethod
    def agricultural_survey() -> Dict[str, Any]:
        """Systematic agricultural field survey."""
        return {
            "mission_type": "survey",
            "target_description": "crop health and field conditions",
            "flight_pattern": "zigzag",
            "parameters": {
                "altitude": 70,
                "speed": 6,
                "coverage_area": {"center": {"lat": 0.0, "lon": 0.0}, "radius": 300},
            },
            "safety": {
                "geofence": {"max_altitude": 100, "max_distance": 500},
                "rtl_battery_threshold": 25,
                "weather_check": True,
                "max_wind_speed": 10.0,
            },
        }

    @staticmethod
    def parking_count() -> Dict[str, Any]:
        """Count vehicles in a parking lot."""
        return {
            "mission_type": "count",
            "target_description": "vehicles in parking area",
            "flight_pattern": "grid",
            "parameters": {
                "altitude": 40,
                "speed": 4,
                "coverage_area": {"center": {"lat": 0.0, "lon": 0.0}, "radius": 100},
            },
            "safety": {
                "geofence": {"max_altitude": 60, "max_distance": 150},
                "rtl_battery_threshold": 25,
                "weather_check": True,
                "max_wind_speed": 10.0,
            },
        }

    @staticmethod
    def mapping_3d() -> Dict[str, Any]:
        """3D mapping mission with multiple altitude passes."""
        return {
            "mission_type": "map",
            "target_description": "3D terrain and structure mapping",
            "flight_pattern": "grid",
            "parameters": {
                "altitude": 60,  # Will vary during mission
                "speed": 5,
                "coverage_area": {"center": {"lat": 0.0, "lon": 0.0}, "radius": 150},
            },
            "safety": {
                "geofence": {"max_altitude": 100, "max_distance": 200},
                "rtl_battery_threshold": 30,
                "weather_check": True,
                "max_wind_speed": 8.0,
            },
        }

    @staticmethod
    def list_templates() -> list:
        """List all available template names."""
        return [
            "quick_search",
            "building_inspection",
            "perimeter_patrol",
            "delivery_direct",
            "emergency_response",
            "agricultural_survey",
            "parking_count",
            "3d_mapping",
        ]
