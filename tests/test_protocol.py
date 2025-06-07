import datetime
import struct
import unittest

from seatrac.protocol import (
    BoardID,
    COM_SWITCHES_FunctionID,
    MessageType,
    PMS_SWITCHES_FunctionID,
    PMSSwitchStatusMessage,
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
    def test_round_trip(self):
        body = b'\42' * 8
        c1, c2 = calculate_checksum(body)
        self.assertTrue(verify_checksum(body + bytes([c1, c2])))
        self.assertFalse(verify_checksum(body + bytes([c1 ^ 1, c2])))

    def test_checksum_0xFF_digit(self):
        # Regression test for a bug where check digits of 0xFF were not handled
        # correctly due to an extraneous % 255 in the checksum calculation.
        self.assertTrue(verify_checksum(bytes.fromhex(
            '00ff1500ff0815e9070605101d210000c079c40000c079c4000500ff80'
        )))



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

    def test_null(self):
        serialized = datetime___bytes__(None)
        self.assertTrue(not any(serialized))  # all zero
        self.assertIsNone(datetime_frombytes(serialized))


class TestSeaTracMessage(unittest.TestCase):
    def test_command_header_parsed(self):
        original = SeaTracMessage(
            relay=Relay(42),
            msg_type=MessageType.COMMAND,
            board_id=BoardID.COM,
            sink_id=SinkID.COM_SWITCHES,
            function_id=COM_SWITCHES_FunctionID.SET,
            payload=SwitchSetCommand(switch=7, state=False),
        )
        recovered = SeaTracMessage.from_bytes(bytes(original))
        self.assertTrue(recovered.is_checksum_valid)
        self.assertEqual(recovered.relay, original.relay)
        self.assertEqual(recovered.msg_type, original.msg_type)
        self.assertEqual(recovered.board_id, original.board_id)
        self.assertEqual(recovered.sink_id, original.sink_id)
        self.assertEqual(recovered.function_id, original.function_id)

    def test_status_reply_header_parsed(self):
        original = SeaTracMessage(
            relay=Relay(42),
            msg_type=MessageType.STATUS_REPLY,
            sink_id=SinkID.MOTOR_GPS,
            timestamp=datetime.datetime(2025, 5, 31, 13, 12, 54, 0)
        )
        recovered = SeaTracMessage.from_bytes(bytes(original))
        self.assertTrue(recovered.is_checksum_valid)
        self.assertEqual(recovered.relay, original.relay)
        self.assertEqual(recovered.msg_type, original.msg_type)
        self.assertEqual(recovered.sink_id, original.sink_id)
        self.assertEqual(recovered.timestamp, original.timestamp)

    def test_too_short_raises(self):
        # Incomplete header
        with self.assertRaises(ValueError):
            SeaTracMessage.from_bytes(b'\x00\xff')

        # Full header, but incomplete payload
        with self.assertRaises(ValueError):
            SeaTracMessage.from_bytes(struct.pack(SeaTracMessage.HEADER_PATTERN,
                0x00, 0xFF, 0x64, Relay(1), MessageType.COMMAND) + b'\x00' * 16)

    def test_invalid_checksum_returns_raw_payload(self):
        msg = SeaTracMessage(
            relay=Relay(42),
            msg_type=MessageType.COMMAND,
            board_id=BoardID.PMS,
            sink_id=SinkID.PMS_SWITCHES,
            function_id=PMS_SWITCHES_FunctionID.SET,
            payload=SwitchSetCommand(switch=7, state=False),
        )
        packet = bytes(msg)
        packet = packet[:-1] + bytes([packet[-1] ^ 1])  # corrupt checksum
        recovered = SeaTracMessage.from_bytes(packet)
        self.assertFalse(recovered.is_checksum_valid)
        self.assertIsInstance(recovered.payload, bytes)

    def test_designated_parser_returns_object(self):
        original = SeaTracMessage(
            relay=Relay(42),
            msg_type=MessageType.COMMAND,
            board_id=BoardID.PMS,
            sink_id=SinkID.PMS_SWITCHES,
            function_id=PMS_SWITCHES_FunctionID.SET,
            payload=SwitchSetCommand(switch=7, state=False),
        )
        recovered = SeaTracMessage.from_bytes(bytes(original))
        self.assertTrue(recovered.is_checksum_valid)
        self.assertIsInstance(recovered.payload, SwitchSetCommand)

    def test_no_designated_parser_payload_bytes(self):
        # Use a function ID for which no parser is registered
        original = SeaTracMessage(
            relay=Relay(42),
            msg_type=MessageType.COMMAND,
            board_id=BoardID.COM,
            sink_id=SinkID.COM_SWITCHES,
            function_id=COM_SWITCHES_FunctionID.GET_BOARD_INFO,
            payload=b'\xAA\xBB\xCC',
        )
        recovered = SeaTracMessage.from_bytes(bytes(original))
        self.assertTrue(recovered.is_checksum_valid)
        self.assertIsInstance(recovered.payload, bytes)

    def test_peek_too_short(self):
        self.assertIsNone(SeaTracMessage.peek_length(b'\x00'))  # half sync
        self.assertIsNone(SeaTracMessage.peek_length(b'\x00\xff'))  # full sync
        self.assertIsNone(SeaTracMessage.peek_length(
            struct.pack(SeaTracMessage.HEADER_PATTERN, 0x00, 0xFF, 0x64,
                Relay(1), MessageType.COMMAND) + b'\x00' * 16))

    def test_peek_invalid_sync(self):
        with self.assertRaises(ValueError):
            SeaTracMessage.peek_length(b'\x42')  # short, incorrect
        with self.assertRaises(ValueError):
            SeaTracMessage.peek_length(b'\x42\x42')  # full, incorrect
        with self.assertRaises(ValueError):
            SeaTracMessage.peek_length(b'\x00\x42')  # half correct

    def test_peek_complete_packet(self):
        msg = SeaTracMessage(
            relay=Relay(42),
            msg_type=MessageType.COMMAND,
            board_id=BoardID.COM,
            sink_id=SinkID.COM_SWITCHES,
            function_id=COM_SWITCHES_FunctionID.SET,
            payload=SwitchSetCommand(switch=2, state=True),
        )
        packet = bytes(msg)
        self.assertEqual(len(packet), SeaTracMessage.peek_length(packet))


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

    def test_expected_encoding(self):
        # From packet capture, but we need to erase the reserved fields
        expected = bytearray.fromhex(
            '010800020d0a0d070d259ca30958099b0903c80030014e02559c2ea746010000')
        expected[0:20] = b'\x00' * 20
        expected[-4:] = b'\x00' * 4

        msg = PowerLevelMessage(
            pack_current=0.608,
            load_current=1.18,
            pack_voltage=40.021,
            soc_percentage=85.596,
        )
        self.assertEqual(bytes(msg), expected)


class TestSwitchSetCommand(unittest.TestCase):
    def test_round_trip_com(self):
        original = SwitchSetCommand(switch=5, state=True)
        recovered = SwitchSetCommand.from_bytes(bytes(original))
        self.assertEqual(recovered.switch, original.switch)
        self.assertTrue(recovered.state, original.state)

    def test_expected_encoding(self):
        expected = bytes.fromhex('0f01')  # from packet capture
        cmd = SwitchSetCommand(switch=15, state=True)
        self.assertEqual(bytes(cmd), expected)


class TestPMSSwitchStatusMessage(unittest.TestCase):
    def test_round_trip_all_states(self):
        nswitches = PMSSwitchStatusMessage.NUM_SWITCHES
        for i in range(1 << nswitches):
            original = PMSSwitchStatusMessage(
                states=[bool((i >> j) & 1) for j in range(nswitches)]
            )
            recovered = PMSSwitchStatusMessage.from_bytes(bytes(original))
            self.assertEqual(original.states, recovered.states)

    def test_invalid_states_length(self):
        nswitches = PMSSwitchStatusMessage.NUM_SWITCHES
        with self.assertRaises(ValueError):
            bytes(PMSSwitchStatusMessage(states=[False] * (nswitches - 1)))
        with self.assertRaises(ValueError):
            bytes(PMSSwitchStatusMessage(states=[False] * (nswitches + 1)))

    def test_expected_encoding(self):
        # From packet capture, but we need to erase the reserved fields
        expected = bytearray.fromhex(
            '1208824b3c9b62046a04260900002609fc0c0e6700')
        expected[0:18] = b'\x00' * 18  # reserved fields

        msg = PMSSwitchStatusMessage(states=[
            # Nav light off; COM, MOTOR, CAN on; 12V bus, 12V #1-#3 off
            False, True, True, True, False, False, False, False,
            # 24V bus, #1 (Router), #2 (Cameras) on; 24V #3 (Certus) off;
            # VBat #1 (Orange PC) off; VBat #2 (IFCB), VBat #3 (Starlink) on;
            # 24V #4 off
            True, True, True, False, False, True, True, False,
            # Anchor light, VBat #4 off
            False, False,
        ])
        self.assertEqual(bytes(msg), expected)


if __name__ == '__main__':
    unittest.main()
