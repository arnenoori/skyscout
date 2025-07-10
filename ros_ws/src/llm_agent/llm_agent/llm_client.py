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


class MissionPlan(BaseModel):
    """Structured mission plan output."""

    mission_type: Literal["search", "inspect", "count", "map"]
    target_description: str
    flight_pattern: Literal["grid", "spiral", "perimeter", "waypoints"]
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
- Weather: {context.get("weather", "Clear")}
- Time of Day: {context.get("time", "Day")}

Generate a JSON mission plan with these exact fields:
{{
    "mission_type": "search|inspect|count|map",
    "target_description": "what to look for",
    "flight_pattern": "grid|spiral|perimeter|waypoints",
    "parameters": {{
        "altitude": 10-120,
        "speed": 1-10,
        "coverage_area": {{
            "center": {{"lat": number, "lon": number}},
            "radius": 50-500
        }}
    }},
    "safety": {{
        "geofence": {{
            "max_altitude": 120,
            "max_distance": 500
        }},
        "rtl_battery_threshold": 20
    }}
}}

Important:
- Choose appropriate altitude based on the target (higher for large area search, lower for detailed inspection)
- Use grid pattern for systematic search, spiral for expanding search, perimeter for boundary inspection
- Set conservative safety parameters
- Ensure the mission can be completed with current battery level
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


class GeminiClient(LLMClient):
    """Google Gemini client implementation."""

    def __init__(self, api_key: Optional[str] = None, model: str = "gemini-pro"):
        self.api_key = api_key or os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("Gemini API key not provided")

        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel(model)
        self.logger = logging.getLogger("GeminiClient")

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
        """Create the prompt for Gemini (similar to OpenAI)."""
        # Same prompt as OpenAI
        return f"""
Convert this natural language command into a structured drone mission plan.

Command: {command}

Current Context:
- Battery Level: {context.get("battery_percent", 100)}%
- Current Position: {context.get("position", "Home")}
- Weather: {context.get("weather", "Clear")}
- Time of Day: {context.get("time", "Day")}

Generate a JSON mission plan with these exact fields:
{{
    "mission_type": "search|inspect|count|map",
    "target_description": "what to look for",
    "flight_pattern": "grid|spiral|perimeter|waypoints",
    "parameters": {{
        "altitude": 10-120,
        "speed": 1-10,
        "coverage_area": {{
            "center": {{"lat": number, "lon": number}},
            "radius": 50-500
        }}
    }},
    "safety": {{
        "geofence": {{
            "max_altitude": 120,
            "max_distance": 500
        }},
        "rtl_battery_threshold": 20
    }}
}}

Important:
- Choose appropriate altitude based on the target (higher for large area search, lower for detailed inspection)
- Use grid pattern for systematic search, spiral for expanding search, perimeter for boundary inspection
- Set conservative safety parameters
- Ensure the mission can be completed with current battery level

Return ONLY the JSON, no other text.
"""

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
