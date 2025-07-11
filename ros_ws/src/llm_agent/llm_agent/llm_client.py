#!/usr/bin/env python3
"""
LLM client implementations for OpenAI and Google Gemini.
"""

import os
import json
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
import logging

import openai
import google.generativeai as genai
from pydantic import BaseModel, Field, validator
from typing import Literal
from .weather_client import WeatherClient
from .mission_templates import MissionTemplates


class MissionParameters(BaseModel):
    """Parameters for the mission execution."""

    altitude: float = Field(ge=10, le=120, description="Flight altitude in meters")
    speed: float = Field(ge=1, le=10, description="Flight speed in m/s")
    coverage_area: Dict[str, Any] = Field(description="Area to cover")


class SafetyParameters(BaseModel):
    """Safety constraints for the mission."""

    geofence: Dict[str, float] = Field(
        default={"max_altitude": 120, "max_distance": 500}
    )
    rtl_battery_threshold: int = Field(
        default=20, ge=10, le=50, description="Battery % to trigger RTL"
    )
    weather_check: bool = Field(default=True, description="Check weather before flight")
    max_wind_speed: float = Field(default=10.0, description="Max wind speed in m/s")


class MissionPlan(BaseModel):
    """Structured mission plan output."""

    mission_type: Literal[
        "search",
        "inspect",
        "count",
        "map",
        "delivery",
        "patrol",
        "survey",
        "emergency",
        "follow",
    ]
    target_description: str
    flight_pattern: Literal[
        "grid", "spiral", "perimeter", "waypoints", "zigzag", "circle", "polygon"
    ]
    parameters: MissionParameters
    safety: SafetyParameters

    @validator("parameters")
    def validate_coverage_area(cls, v):
        """Ensure coverage area has required fields."""
        if "center" not in v.coverage_area:
            v.coverage_area["center"] = {"lat": 0.0, "lon": 0.0}
        if "radius" not in v.coverage_area:
            v.coverage_area["radius"] = 100
        return v


class LLMClient(ABC):
    """Abstract base class for LLM clients."""

    @abstractmethod
    def generate_mission_plan(
        self, command: str, context: Dict[str, Any]
    ) -> MissionPlan:
        """Generate a mission plan from natural language command."""
        pass


class OpenAIClient(LLMClient):
    """OpenAI GPT client implementation."""

    def __init__(
        self, api_key: Optional[str] = None, model: str = "gpt-4-turbo-preview"
    ):
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key not provided")

        self.client = openai.OpenAI(api_key=self.api_key)
        self.model = model
        self.logger = logging.getLogger("OpenAIClient")
        self.weather_client = WeatherClient()

    def generate_mission_plan(
        self, command: str, context: Dict[str, Any]
    ) -> MissionPlan:
        """Generate mission plan using OpenAI GPT."""
        prompt = self._create_prompt(command, context)

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": "You are a drone mission planner. Convert natural language commands into structured JSON mission plans.",
                    },
                    {"role": "user", "content": prompt},
                ],
                response_format={"type": "json_object"},
                temperature=0.3,
                max_tokens=1000,
            )

            json_str = response.choices[0].message.content
            mission_dict = json.loads(json_str)

            # Validate and parse using Pydantic
            return MissionPlan(**mission_dict)

        except Exception as e:
            self.logger.error(f"Error generating mission plan: {e}")
            # Return a safe default mission
            return self._default_mission(command)

    def _create_prompt(self, command: str, context: Dict[str, Any]) -> str:
        """Create the prompt for the LLM."""
        return f"""
Convert this natural language command into a structured drone mission plan.

Command: {command}

Current Context:
- Battery Level: {context.get("battery_percent", 100)}%
- Current Position: {context.get("position", "Home")}
- Weather: {context.get("weather", {}).get("conditions", "Clear")}
- Wind Speed: {context.get("weather", {}).get("wind_speed", 5.0)} m/s
- Visibility: {context.get("weather", {}).get("visibility", 10.0)} km
- Time of Day: {context.get("time", "Day")}

Generate a JSON mission plan with these exact fields:
{{
    "mission_type": "search|inspect|count|map|delivery|patrol|survey|emergency|follow",
    "target_description": "what to look for or do",
    "flight_pattern": "grid|spiral|perimeter|waypoints|zigzag|circle|polygon",
    "parameters": {{
        "altitude": 10-120,
        "speed": 1-10,
        "coverage_area": {{
            "center": {{"lat": number, "lon": number}},
            "radius": 50-500,
            "vertices": [{{"x": number, "y": number}}] // optional for polygon
        }}
    }},
    "safety": {{
        "geofence": {{
            "max_altitude": 120,
            "max_distance": 500
        }},
        "rtl_battery_threshold": 20,
        "weather_check": true,
        "max_wind_speed": 10.0
    }}
}}

Mission Type Guidelines:
- search: Find specific objects/people in an area
- inspect: Detailed examination of a specific target
- count: Count objects in an area (vehicles, animals, etc.)
- map: Create aerial map/survey of an area
- delivery: Navigate to location and hover for package drop
- patrol: Follow predefined route repeatedly
- survey: Agricultural/land survey with systematic coverage
- emergency: Rapid response to GPS coordinates
- follow: Track and follow a moving target

Flight Pattern Guidelines:
- grid: Systematic back-and-forth coverage (best for search/survey)
- spiral: Expanding outward search from center
- perimeter: Boundary inspection of an area
- zigzag: Efficient coverage with tighter spacing
- circle: Orbit around a target for 360° inspection
- polygon: Custom shape for specific area coverage
- waypoints: Custom path through specific points

Important:
- Choose appropriate altitude based on the mission (higher for large area, lower for detail)
- Adjust speed based on mission requirements (slower for inspection, faster for emergency)
- Set conservative safety parameters based on weather and battery
- For delivery/emergency, prioritize direct path and speed

Available Templates (use if command matches):
{self._get_template_descriptions()}

If the command closely matches a template scenario, use that template's parameters as a starting point.
"""

    def _default_mission(self, command: str) -> MissionPlan:
        """Create a safe default mission if generation fails."""
        return MissionPlan(
            mission_type="search",
            target_description=command[:100],  # Truncate long commands
            flight_pattern="grid",
            parameters=MissionParameters(
                altitude=30,
                speed=5,
                coverage_area={"center": {"lat": 0.0, "lon": 0.0}, "radius": 100},
            ),
            safety=SafetyParameters(),
        )

    def _get_template_descriptions(self) -> str:
        """Get descriptions of available templates for the prompt."""
        descriptions = [
            "- 'quick search' / 'find someone': Use quick_search template",
            "- 'inspect building' / 'check roof': Use building_inspection template",
            "- 'patrol' / 'security check': Use perimeter_patrol template",
            "- 'deliver' / 'drop package': Use delivery_direct template",
            "- 'emergency' / 'urgent': Use emergency_response template",
            "- 'survey field' / 'check crops': Use agricultural_survey template",
            "- 'count cars' / 'parking lot': Use parking_count template",
            "- 'map area' / '3D scan': Use mapping_3d template",
        ]
        return "\n".join(descriptions)


class GeminiClient(LLMClient):
    """Google Gemini client implementation."""

    def __init__(self, api_key: Optional[str] = None, model: str = "gemini-pro"):
        self.api_key = api_key or os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("Gemini API key not provided")

        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel(model)
        self.logger = logging.getLogger("GeminiClient")
        self.weather_client = WeatherClient()
        self.templates = MissionTemplates()

    def generate_mission_plan(
        self, command: str, context: Dict[str, Any]
    ) -> MissionPlan:
        """Generate mission plan using Google Gemini."""
        prompt = self._create_prompt(command, context)

        try:
            response = self.model.generate_content(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.3,
                    max_output_tokens=1000,
                ),
            )

            # Extract JSON from response
            json_str = response.text
            if "```json" in json_str:
                json_str = json_str.split("```json")[1].split("```")[0]
            elif "```" in json_str:
                json_str = json_str.split("```")[1].split("```")[0]

            mission_dict = json.loads(json_str.strip())
            return MissionPlan(**mission_dict)

        except Exception as e:
            self.logger.error(f"Error generating mission plan: {e}")
            return self._default_mission(command)

    def _create_prompt(self, command: str, context: Dict[str, Any]) -> str:
        """Create the prompt for Gemini."""
        # Use the same enhanced prompt as OpenAI but ensure JSON-only response
        prompt = OpenAIClient._create_prompt(self, command, context)
        return prompt + "\n\nReturn ONLY the JSON, no other text."

    def _default_mission(self, command: str) -> MissionPlan:
        """Create a safe default mission if generation fails."""
        return MissionPlan(
            mission_type="search",
            target_description=command[:100],
            flight_pattern="grid",
            parameters=MissionParameters(
                altitude=30,
                speed=5,
                coverage_area={"center": {"lat": 0.0, "lon": 0.0}, "radius": 100},
            ),
            safety=SafetyParameters(),
        )


def create_llm_client(
    provider: str = "openai", api_key: Optional[str] = None
) -> LLMClient:
    """Factory function to create the appropriate LLM client."""
    if provider.lower() == "openai":
        return OpenAIClient(api_key)
    elif provider.lower() == "gemini":
        return GeminiClient(api_key)
    else:
        raise ValueError(f"Unknown LLM provider: {provider}")
