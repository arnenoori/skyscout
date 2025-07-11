#!/usr/bin/env python3
"""
Weather client for checking flight conditions.
"""

import os
import requests
from typing import Dict, Optional
import logging


class WeatherClient:
    """Client for checking weather conditions before flight."""

    def __init__(self, api_key: Optional[str] = None):
        self.api_key = api_key or os.getenv("OPENWEATHER_API_KEY")
        self.logger = logging.getLogger("WeatherClient")
        self.base_url = "https://api.openweathermap.org/data/2.5/weather"

    def get_weather_conditions(self, lat: float, lon: float) -> Dict[str, any]:
        """Get current weather conditions at given coordinates."""
        if not self.api_key:
            # Return default safe conditions if no API key
            return self._default_weather()

        try:
            params = {"lat": lat, "lon": lon, "appid": self.api_key, "units": "metric"}

            response = requests.get(self.base_url, params=params, timeout=5)
            response.raise_for_status()

            data = response.json()

            return {
                "safe_to_fly": self._check_flight_safety(data),
                "wind_speed": data["wind"]["speed"],  # m/s
                "wind_gust": data["wind"].get("gust", data["wind"]["speed"]),
                "visibility": data.get("visibility", 10000) / 1000,  # km
                "conditions": data["weather"][0]["main"],
                "temperature": data["main"]["temp"],  # Celsius
                "humidity": data["main"]["humidity"],  # %
                "rain": data.get("rain", {}).get("1h", 0),  # mm
            }

        except Exception as e:
            self.logger.error(f"Error fetching weather: {e}")
            return self._default_weather()

    def _check_flight_safety(self, weather_data: dict) -> bool:
        """Check if weather conditions are safe for flight."""
        # Safety thresholds
        MAX_WIND_SPEED = 10  # m/s (36 km/h)
        MAX_WIND_GUST = 15  # m/s (54 km/h)
        MIN_VISIBILITY = 3  # km
        MAX_RAIN = 2.5  # mm/hour

        wind_speed = weather_data["wind"]["speed"]
        wind_gust = weather_data["wind"].get("gust", wind_speed)
        visibility = weather_data.get("visibility", 10000) / 1000
        rain = weather_data.get("rain", {}).get("1h", 0)
        conditions = weather_data["weather"][0]["main"].lower()

        # Check each safety parameter
        if wind_speed > MAX_WIND_SPEED:
            self.logger.warning(f"Wind speed too high: {wind_speed} m/s")
            return False

        if wind_gust > MAX_WIND_GUST:
            self.logger.warning(f"Wind gusts too high: {wind_gust} m/s")
            return False

        if visibility < MIN_VISIBILITY:
            self.logger.warning(f"Visibility too low: {visibility} km")
            return False

        if rain > MAX_RAIN:
            self.logger.warning(f"Rain too heavy: {rain} mm/h")
            return False

        # Check for dangerous conditions
        dangerous_conditions = ["thunderstorm", "tornado", "squall", "hurricane"]
        if any(cond in conditions for cond in dangerous_conditions):
            self.logger.warning(f"Dangerous weather conditions: {conditions}")
            return False

        return True

    def _default_weather(self) -> Dict[str, any]:
        """Return default weather conditions when API is unavailable."""
        return {
            "safe_to_fly": True,
            "wind_speed": 5.0,
            "wind_gust": 5.0,
            "visibility": 10.0,
            "conditions": "Clear",
            "temperature": 20.0,
            "humidity": 50,
            "rain": 0,
        }
