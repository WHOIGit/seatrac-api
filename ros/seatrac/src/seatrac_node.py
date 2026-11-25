#!/usr/bin/env python3
import datetime
import functools
import struct
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from wr2_msgs.msg import RawData

import seatrac.protocol as proto
from seatrac.msg import OutletStatus, PowerLevel


class SeaTracNode(Node):
    def __init__(self, **kwargs):
        super().__init__('seatrac', automatically_declare_parameters_from_overrides=True,
                         **kwargs)
        outlet_params = self.get_parameters_by_prefix('outlets')
        self.outlets = {name: param.value for name, param in outlet_params.items()}
        num_switches = proto.PMSSwitchStatusMessage.NUM_SWITCHES
        for name, number in self.outlets.items():
            if type(number) is not int or not 1 <= number <= num_switches:
                raise ValueError(f'Invalid outlet number for {name}: {number!r}')
        if len(self.outlets) != len(set(self.outlets.values())):
            raise ValueError('Duplicate outlet numbers found in configuration')

        self.buffer = bytearray()
        self.power_pub = self.create_publisher(PowerLevel, '~/power', 10)
        self.out_pub = self.create_publisher(RawData, '~/out', 10)
        self.create_subscription(RawData, '~/in', self.handle_raw_packet, 10)
        self.status_pubs = {}
        for name, number in self.outlets.items():
            self.status_pubs[name] = self.create_publisher(
                OutletStatus, f'~/outlet/{name}/status', 10)
            self.create_subscription(Bool, f'~/outlet/{name}/control',
                                     functools.partial(self.handle_outlet_control, number), 10)

    def populate_timestamps(self, msg, timestamp: Optional[datetime.datetime]) -> None:
        msg.ds_header.io_time = self.get_clock().now().to_msg()
        if timestamp is None:
            msg.header.stamp = msg.ds_header.io_time
        else:
            msg.header.stamp.sec = int(timestamp.replace(microsecond=0).timestamp())
            msg.header.stamp.nanosec = timestamp.microsecond * 1000

    def handle_seatrac_message(self, msg: proto.SeaTracMessage) -> None:
        payload = msg.payload
        if isinstance(payload, proto.PowerLevelMessage):
            power = PowerLevel(
                pack_current=payload.pack_current,
                load_current=payload.load_current,
                pack_voltage=payload.pack_voltage,
                soc_percentage=payload.soc_percentage,
            )
            self.populate_timestamps(power, msg.timestamp)
            self.power_pub.publish(power)
        elif isinstance(payload, proto.PMSSwitchStatusMessage):
            for name, number in self.outlets.items():
                status = OutletStatus(is_active=payload.states[number - 1])
                self.populate_timestamps(status, msg.timestamp)
                self.status_pubs[name].publish(status)

    def handle_raw_packet(self, msg: RawData) -> None:
        self.buffer.extend(msg.data)
        while self.buffer:
            try:
                ready = proto.SeaTracMessage.peek_length(self.buffer)
            except ValueError:
                self.buffer.clear()
                return
            if ready is None:
                return

            packet = self.buffer[:ready]
            del self.buffer[:ready]
            try:
                parsed = proto.SeaTracMessage.from_bytes(packet)
            except (ValueError, IndexError, struct.error) as exc:
                self.get_logger().warning(f'Dropped malformed packet: {exc}')
                continue
            if not parsed.is_checksum_valid:
                self.get_logger().warning('Dropped packet with invalid checksum')
                continue
            self.handle_seatrac_message(parsed)

    def handle_outlet_control(self, outlet: int, msg: Bool) -> None:
        packet = proto.SeaTracMessage(
            outbound_relay=proto.Relay.CAN_BUS,
            return_relay=proto.Relay.DEFAULT,
            msg_type=proto.MessageType.COMMAND,
            board_id=proto.BoardID.PMS,
            sink_id=proto.SinkID.PMS_SWITCHES,
            function_id=proto.PMS_SWITCHES_FunctionID.SET,
            payload=proto.SwitchSetCommand(outlet, msg.data),
        )
        self.out_pub.publish(RawData(io_stamp=self.get_clock().now().to_msg(),
                                     data_direction=RawData.DIRECTION_DATA_OUT,
                                     data=list(bytes(packet))))


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = SeaTracNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
