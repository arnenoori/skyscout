import importlib
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros_ws" / "src" / "llm_agent"))

MissionTemplates = importlib.import_module(
    "llm_agent.mission_templates"
).MissionTemplates


class MissionTemplatesTest(unittest.TestCase):
    def test_every_listed_template_has_required_mission_shape(self):
        for name in MissionTemplates.list_templates():
            with self.subTest(template=name):
                template = MissionTemplates.get_template(name)

                self.assertIn(
                    template["mission_type"],
                    {
                        "search",
                        "inspect",
                        "patrol",
                        "delivery",
                        "emergency",
                        "survey",
                        "count",
                        "map",
                    },
                )
                self.assertIn(
                    template["flight_pattern"],
                    {
                        "grid",
                        "spiral",
                        "perimeter",
                        "waypoints",
                        "zigzag",
                        "circle",
                    },
                )
                self.assertGreaterEqual(template["parameters"]["altitude"], 10)
                self.assertLessEqual(template["parameters"]["altitude"], 120)
                self.assertGreaterEqual(template["parameters"]["speed"], 1)
                self.assertLessEqual(template["parameters"]["speed"], 10)
                self.assertIn("center", template["parameters"]["coverage_area"])
                self.assertIn("radius", template["parameters"]["coverage_area"])
                self.assertGreaterEqual(template["safety"]["rtl_battery_threshold"], 10)
                self.assertLessEqual(template["safety"]["rtl_battery_threshold"], 50)

    def test_unknown_template_falls_back_to_quick_search(self):
        self.assertEqual(
            MissionTemplates.get_template("unknown-template"),
            MissionTemplates.quick_search(),
        )


if __name__ == "__main__":
    unittest.main()
