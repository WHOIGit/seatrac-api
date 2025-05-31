import datetime
import unittest

from seatrac.protocol import (
    BoardID,
    COM_SWITCHES_FunctionID,
    MessageType,
    PMS_SWITCHES_FunctionID,
    PowerLevelMessage,
    Relay,
    SeaTracMessage,
    SinkID,
    SwitchSetCommand,
    calculate_checksum,
    datetime___bytes__,
    datetime_frombytes,
    verify_checksum,
)



class TestChecksum(unittest.TestCase):
    def test_verify_checksum(self):
        body = b'\42' * 8
        c1, c2 = calculate_checksum(body)
        self.assertTrue(verify_checksum(body + bytes([c1, c2])))
        self.assertFalse(verify_checksum(body + bytes([c1 ^ 1, c2])))


class TestDatetime(unittest.TestCase):
    def test_round_trip(self):
        samples = [
            datetime.datetime(1970, 1, 1, 0, 0, 0, 0),
            datetime.datetime(2022, 12, 31, 23, 59, 59, 120000),
            datetime.datetime(2025, 3, 7, 15, 17, 14, 250000),
        ]
        for dt in samples:
            serialized = datetime___bytes__(dt)
            deserialized = datetime_frombytes(serialized)
            self.assertEqual(dt, deserialized)

    def test_expected_encoding(self):
        expected = bytes.fromhex('e907030715171400')  # captured in packet trace
        dt = datetime.datetime(2025, 3, 7, 21, 23, 20, 0)
        serialized = datetime___bytes__(dt)
        self.assertEqual(serialized, expected)


class TestPowerLevelMessage(unittest.TestCase):
    def test_round_trip(self):
        original = PowerLevelMessage(
            pack_current=3.1415926535,
            load_current=2.7182818284,
            pack_voltage=1.6180339887,
            soc_percentage=1.4142135623,
        )
        recovered = PowerLevelMessage.from_bytes(bytes(original))

        assert_close = lambda a, b: self.assertAlmostEqual(a, b, places=2)
        assert_close(original.pack_current, recovered.pack_current)
        assert_close(original.load_current, recovered.load_current)
        assert_close(original.pack_voltage, recovered.pack_voltage)
        assert_close(original.soc_percentage, recovered.soc_percentage)
        self.assertEqual(bytes(original), bytes(recovered))


class TestSwitchSetCommand(unittest.TestCase):
    def test_round_trip_com(self):
        original = SwitchSetCommand(switch=5, state=True)
        recovered = SwitchSetCommand.from_bytes(bytes(original))
        self.assertEqual(recovered.switch, original.switch)
        self.assertTrue(recovered.state, original.state)

    def test_expected_encoding(self):
        expected = bytes.fromhex('0501')  # TODO: use packet capture
        cmd = SwitchSetCommand(switch=5, state=True)
        self.assertEqual(bytes(cmd), expected)


if __name__ == '__main__':
    unittest.main()
