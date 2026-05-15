import unittest
from pathlib import Path

from sim.run_sim import (
    pack_arm_target_frame,
    run_headless,
    write_html_report,
    write_interactive_html,
)


class HeadlessSimulationTests(unittest.TestCase):
    def test_arm_target_frame_uses_existing_protocol(self):
        frame = pack_arm_target_frame(0.04, 0.0, 0.28)
        self.assertEqual(frame[0:4], bytes([0x55, 0xAA, 0x12, 12]))
        self.assertEqual(len(frame), 17)
        self.assertEqual(frame[-1], sum(frame[:-1]) & 0xFF)

    def test_headless_simulation_reaches_place_and_reports_coordinates(self):
        result = run_headless(max_ms=26000, step_ms=5, build_if_needed=True)
        self.assertTrue(result.target_consumed)
        self.assertTrue(result.observed_place)
        self.assertTrue(result.observed_retreat)
        self.assertGreater(len(result.trace), 100)
        self.assertIsNotNone(result.detected_box)
        self.assertIsNotNone(result.final_position)

        z_values = [snap.end[2] for snap in result.trace]
        x_values = [snap.end[0] for snap in result.trace]
        self.assertGreater(max(z_values) - min(z_values), 0.55)
        self.assertGreater(max(x_values) - min(x_values), 0.10)

    def test_html_report_is_clear_animated_showcase_page(self):
        result = run_headless(max_ms=26000, step_ms=5, build_if_needed=True)
        output = write_html_report(result, Path("sim/output/test_showcase.html"))
        page = output.read_text(encoding="utf-8")

        self.assertIn("机械狗抓取与单侧绕背仿真", page)
        self.assertIn('canvas id="world"', page)
        self.assertIn("视觉发现物资箱（开始仿真）", page)
        self.assertIn("requestAnimationFrame", page)
        self.assertIn('"trace"', page)

    def test_interactive_page_has_manual_box_coordinate_inputs(self):
        page = write_interactive_html()

        self.assertIn('id="box-x"', page)
        self.assertIn('id="box-y"', page)
        self.assertIn('id="box-z"', page)
        self.assertIn("应用坐标并重生成仿真", page)
        self.assertIn("/simulate?", page)


if __name__ == "__main__":
    unittest.main()
