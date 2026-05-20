import ctypes
from pathlib import Path
import struct
import unittest

from sim.run_sim import (
    SimLibrary,
    pack_arm_target_frame,
    run_headless,
    write_html_report,
    write_interactive_html,
)


class CChassisMove(ctypes.Structure):
    _fields_ = [
        ("vx", ctypes.c_float),
        ("vy", ctypes.c_float),
        ("wz", ctypes.c_float),
        ("state", ctypes.c_uint8),
    ]


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

        self.assertIn("鏈烘鐙楁姄鍙栦笌鍗曚晶缁曡儗浠跨湡", page)
        self.assertIn('canvas id="world"', page)
        self.assertIn("瑙嗚鍙戠幇鐗╄祫绠憋紙寮€濮嬩豢鐪燂級", page)
        self.assertIn("requestAnimationFrame", page)
        self.assertIn('"trace"', page)

    def test_interactive_page_has_manual_box_coordinate_inputs(self):
        page = write_interactive_html()

        self.assertIn('id="box-x"', page)
        self.assertIn('id="box-y"', page)
        self.assertIn('id="box-z"', page)
        self.assertIn("搴旂敤鍧愭爣骞堕噸鐢熸垚浠跨湡", page)
        self.assertIn("/simulate?", page)

    def test_protocol_handler_parses_full_chassis_frame_and_timeout(self):
        lib = SimLibrary.load(build_if_needed=True)
        lib.init()

        frame = bytearray([0x55, 0xAA, 0x10, 13])
        frame.extend(struct.pack("<fffB", 0.35, -0.12, 1.5, 4))
        frame.append(sum(frame) & 0xFF)
        lib.process_buffer(bytes(frame))

        raw = lib._lib
        raw.sim_get_chassis_move.argtypes = [ctypes.POINTER(CChassisMove)]
        raw.sim_get_chassis_move.restype = None
        raw.sim_is_new_chassis_data.argtypes = []
        raw.sim_is_new_chassis_data.restype = ctypes.c_uint8
        raw.sim_clear_chassis_data_flag.argtypes = []
        raw.sim_clear_chassis_data_flag.restype = None
        raw.sim_is_chassis_timeout.argtypes = []
        raw.sim_is_chassis_timeout.restype = ctypes.c_uint8

        move = CChassisMove()
        raw.sim_get_chassis_move(ctypes.byref(move))

        self.assertAlmostEqual(move.vx, 0.35, places=4)
        self.assertAlmostEqual(move.vy, -0.12, places=4)
        self.assertAlmostEqual(move.wz, 1.5, places=4)
        self.assertEqual(move.state, 4)
        self.assertEqual(raw.sim_is_new_chassis_data(), 1)

        raw.sim_clear_chassis_data_flag()
        self.assertEqual(raw.sim_is_new_chassis_data(), 0)
        self.assertEqual(raw.sim_is_chassis_timeout(), 0)

        lib.step(120)
        self.assertEqual(raw.sim_is_chassis_timeout(), 1)

    def test_protocol_handler_tracks_new_gait_switch_data(self):
        lib = SimLibrary.load(build_if_needed=True)
        lib.init()

        frame = bytearray([0x55, 0xAA, 0x11, 1, 3])
        frame.append(sum(frame) & 0xFF)
        lib.process_buffer(bytes(frame))

        raw = lib._lib
        raw.sim_get_gait_switch.argtypes = []
        raw.sim_get_gait_switch.restype = ctypes.c_uint8
        raw.sim_is_new_gait_data.argtypes = []
        raw.sim_is_new_gait_data.restype = ctypes.c_uint8
        raw.sim_clear_gait_data_flag.argtypes = []
        raw.sim_clear_gait_data_flag.restype = None

        self.assertEqual(raw.sim_get_gait_switch(), 3)
        self.assertEqual(raw.sim_is_new_gait_data(), 1)
        raw.sim_clear_gait_data_flag()
        self.assertEqual(raw.sim_is_new_gait_data(), 0)


if __name__ == "__main__":
    unittest.main()
