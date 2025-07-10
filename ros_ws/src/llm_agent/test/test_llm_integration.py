#!/usr/bin/env python3
"""
Test script for LLM integration.
Run this to verify your API keys are working.
"""

import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from llm_agent.llm_client import create_llm_client
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


def test_llm_client(provider: str = "openai"):
    """Test the LLM client with various commands."""

    print(f"\n=== Testing {provider.upper()} Client ===\n")

    try:
        # Create client
        client = create_llm_client(provider)
        print(f"✓ Successfully initialized {provider} client")

        # Test commands
        test_commands = [
            "Find all red vehicles in the parking lot",
            "Inspect the solar panels on building 3 for damage",
            "Count how many people are wearing safety helmets",
            "Map the perimeter of the construction site",
            "Search for a blue delivery truck near the loading dock",
        ]

        # Context for testing
        context = {
            "battery_percent": 85,
            "position": {"lat": 37.4419, "lon": -122.1430, "alt": 0},
            "weather": "Clear",
            "time": "Day",
        }

        for i, command in enumerate(test_commands, 1):
            print(f"\n--- Test {i} ---")
            print(f"Command: {command}")

            try:
                # Generate mission plan
                mission = client.generate_mission_plan(command, context)

                # Print results
                print("✓ Mission generated successfully")
                print(f"  Type: {mission.mission_type}")
                print(f"  Target: {mission.target_description}")
                print(f"  Pattern: {mission.flight_pattern}")
                print(f"  Altitude: {mission.parameters.altitude}m")
                print(f"  Speed: {mission.parameters.speed}m/s")
                print(
                    f"  Coverage: {mission.parameters.coverage_area['radius']}m radius"
                )

                # Validate mission
                assert 10 <= mission.parameters.altitude <= 120
                assert 1 <= mission.parameters.speed <= 10
                print("✓ Mission parameters validated")

            except Exception as e:
                print(f"✗ Error: {e}")

        print(f"\n=== {provider.upper()} Client Test Complete ===\n")

    except Exception as e:
        print(f"✗ Failed to initialize {provider} client: {e}")
        print(f"  Make sure {provider.upper()}_API_KEY is set in environment")


def main():
    """Run tests for available LLM providers."""

    print("SkyScout LLM Integration Test\n")

    # Check which API keys are available
    has_openai = bool(os.getenv("OPENAI_API_KEY"))
    has_gemini = bool(os.getenv("GEMINI_API_KEY"))

    if not has_openai and not has_gemini:
        print("✗ No API keys found!")
        print("\nPlease set one of the following environment variables:")
        print("  export OPENAI_API_KEY='your-key-here'")
        print("  export GEMINI_API_KEY='your-key-here'")
        return

    # Test available providers
    if has_openai:
        test_llm_client("openai")

    if has_gemini:
        test_llm_client("gemini")

    print("\nTo use in ROS2:")
    print("  ros2 run llm_agent llm_agent_node")
    print(
        "  ros2 topic pub /natural_language_command std_msgs/String \"data: 'Find all red cars'\""
    )


if __name__ == "__main__":
    main()
