#!/usr/bin/env python3
import datetime
import functools
import struct

import rclpy

from typing import Optional

import seatrac.protocol as proto

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from seatrac.msg import OutletStatus, PowerLevel
from std_msgs.msg import Bool
from wr2_msgs.msg import RawData


def populate_timestamps(node: Node, msg,
                        timestamp: Optional[datetime.datetime]) -> None:
    # See HEADERS.md in ds_core_msgs for details on the timestamps.
    #     header.stamp: timestamp reported by the boat
    #     ds_header.io_time: timestamp when we received it
    msg.ds_header.io_time = node.get_clock().now().to_msg()
    if timestamp is None:
        msg.header.stamp = msg.ds_header.io_time
    else:
        msg.header.stamp.sec = int(timestamp.replace(microsecond=0).timestamp())
        msg.header.stamp.nanosec = timestamp.microsecond * 1000


def configure_node(node: Node) -> None:
    outlets = {
        name: param.value
        for name, param in node.get_parameters_by_prefix('outlets').items()
    }

    # Sanity check the outlet numbers
    num_switches = proto.PMSSwitchStatusMessage.NUM_SWITCHES
    for name, number in outlets.items():
        if type(number) is not int or not 1 <= number <= num_switches:
            raise ValueError(f'Invalid outlet number for {name}: {number!r}')

    if len(outlets) != len(set(outlets.values())):
        raise ValueError('Duplicate outlet numbers found in configuration')

    power_pub = node.create_publisher(PowerLevel, '~/power', 10)
    out_pub = node.create_publisher(RawData, '~/out', 10)
    status_pubs = {
        name: node.create_publisher(OutletStatus, f'~/outlet/{name}/status', 10)
        for name in outlets
    }

    buffer = bytearray()

    def handle_power_level(msg: proto.PowerLevelMessage,
                           timestamp: Optional[datetime.datetime]) -> None:
        ros_msg = PowerLevel()
        populate_timestamps(node, ros_msg, timestamp)
        ros_msg.pack_current = msg.pack_current
        ros_msg.load_current = msg.load_current
        ros_msg.pack_voltage = msg.pack_voltage
        ros_msg.soc_percentage = msg.soc_percentage
        power_pub.publish(ros_msg)

    def handle_switch_status(msg: proto.PMSSwitchStatusMessage,
                             timestamp: Optional[datetime.datetime]) -> None:
        for name, number in outlets.items():
            ros_msg = OutletStatus()
            populate_timestamps(node, ros_msg, timestamp)
            ros_msg.is_active = msg.states[number - 1]
            status_pubs[name].publish(ros_msg)

    def handle_seatrac_message(msg: proto.SeaTracMessage) -> None:
        if isinstance(msg.payload, proto.PowerLevelMessage):
            handle_power_level(msg.payload, msg.timestamp)
        elif isinstance(msg.payload, proto.PMSSwitchStatusMessage):
            handle_switch_status(msg.payload, msg.timestamp)

    def handle_raw_packet(in_msg: RawData) -> None:
        buffer.extend(in_msg.data)
        while True:
            try:
                if not (ready := proto.SeaTracMessage.peek_length(buffer)):
                    break
            except ValueError:
                buffer.clear()
                break

            packet = buffer[:ready]
            del buffer[:ready]

            try:
                parsed = proto.SeaTracMessage.from_bytes(packet)
            except (ValueError, IndexError, struct.error) as exc:
                node.get_logger().warning(f'Dropped malformed packet: {exc}')
                continue
            if not parsed.is_checksum_valid:
                node.get_logger().warning('Dropped packet with invalid checksum')
                continue  # discard

            handle_seatrac_message(parsed)

    node.create_subscription(RawData, '~/in', handle_raw_packet, 10)


    def handle_outlet_control(outlet: int, msg: Bool) -> None:
        out = RawData()
        out.data_direction = RawData.DIRECTION_DATA_OUT
        out.data = list(bytes(proto.SeaTracMessage(
            outbound_relay=proto.Relay.CAN_BUS,
            return_relay=proto.Relay.DEFAULT,
            msg_type=proto.MessageType.COMMAND,
            board_id=proto.BoardID.PMS,
            sink_id=proto.SinkID.PMS_SWITCHES,
            function_id=proto.PMS_SWITCHES_FunctionID.SET,
            payload=proto.SwitchSetCommand(outlet, msg.data),
        )))
        out.io_stamp = node.get_clock().now().to_msg()
        out_pub.publish(out)

    for name, number in outlets.items():
        node.create_subscription(Bool, f'~/outlet/{name}/control',
                                 functools.partial(handle_outlet_control, number), 10)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node(
        'seatrac', automatically_declare_parameters_from_overrides=True)
    try:
        configure_node(node)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
