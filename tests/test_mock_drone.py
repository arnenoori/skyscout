import importlib
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros_ws" / "src" / "navigation_bridge"))

mock_drone = importlib.import_module("navigation_bridge.mock_drone")
DroneMode = mock_drone.DroneMode
MockDrone = mock_drone.MockDrone


class MockDroneTest(unittest.TestCase):
    def test_arm_requires_safe_battery_level(self):
        drone = MockDrone()
        drone.battery_percent = 24

        self.assertFalse(drone.arm())
        self.assertFalse(drone.is_armed)
        self.assertEqual(drone.mode, DroneMode.MANUAL)

    def test_takeoff_clamps_altitude_to_geofence(self):
        drone = MockDrone()
        self.assertTrue(drone.arm())

        self.assertTrue(drone.takeoff(500))

        self.assertEqual(drone.target_position["alt"], drone.geofence["max_altitude"])
        self.assertFalse(drone.target_reached)

    def test_goto_rejects_targets_outside_geofence(self):
        drone = MockDrone()
        drone.arm()

        accepted = drone.goto(
            drone.home_position["lat"] + 1,
            drone.home_position["lon"] + 1,
            20,
        )

        self.assertFalse(accepted)
        self.assertIsNone(drone.target_position)

    def test_low_battery_triggers_return_to_launch(self):
        drone = MockDrone()
        drone.arm()
        drone.position["alt"] = 10
        drone.battery_percent = drone.rtl_battery_threshold - 1

        drone.update()

        self.assertEqual(drone.mode, DroneMode.RTL)
        self.assertEqual(drone.target_position["lat"], drone.home_position["lat"])
        self.assertEqual(drone.target_position["lon"], drone.home_position["lon"])

    def test_simulation_commands_are_bounded(self):
        drone = MockDrone()

        drone.simulate_scenario("BATTERY", "150")
        self.assertEqual(drone.battery_percent, 100)

        drone.simulate_scenario("BATTERY", "-10")
        self.assertEqual(drone.battery_percent, 0)


if __name__ == "__main__":
    unittest.main()
