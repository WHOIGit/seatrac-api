#!/usr/bin/env python3
import rospy

from typing import Callable

import seatrac.protocol as proto

from ds_core_msgs.msg import RawData
from seatrac.msg import PowerLevel, OutletStatus


def main() -> None:
    rospy.init_node('seatrac')

    outlets = {
        o['name']: o['outlet']
        for o in rospy.get_param('~outlets', [])
        if hasattr(o, 'name') and hasattr(o, 'outlet')
    }

    power_pub = rospy.Publisher('~power', PowerLevel, queue_size=10)
    out_pub = rospy.Publisher('~out', RawData, queue_size=10)

    buffer = bytearray()

    def handle_power_level(msg: proto.PowerLevelMessage) -> None:
        ros_msg = PowerLevel()
        ros_msg.header.stamp = ros_msg.ds_header.io_time = rospy.Time.now()
        ros_msg.pack_current = msg.pack_current
        ros_msg.load_current = msg.load_current
        ros_msg.pack_voltage = msg.pack_voltage
        ros_msg.soc_percentage = msg.soc_percentage
        power_pub.publish(ros_msg)

    def handle_seatrac_message(msg: proto.SeaTracMessage) -> None:
        if isinstance(msg.payload, proto.PowerLevelMessage):
            handle_power_level(msg.payload)

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

            parsed = proto.SeaTracMessage.from_bytes(packet)
            if not parsed.is_checksum_valid:
                rospy.logwarn('Dropped packet with invalid checksum')
                continue  # discard

            handle_seatrac_message(parsed)

    def make_switch_control_callback(
        board_id: proto.BoardID,
        sink_id: proto.SinkID,
        function_id: proto.FunctionID
    ) -> Callable[[OutletStatus], None]:
        def cb(msg: OutletStatus) -> None:
            outlet = outlets.get(msg.name)
            if outlet is None:
                rospy.logwarn(f'Unknown outlet {msg.name}')
                return

            out = RawData()
            out.data_direction = RawData.DATA_OUT
            out.data = bytes(proto.SeaTracMessage(
                relay=0,  # FIXME? Should we use a specific relay?
                msg_type=proto.MessageType.COMMAND,
                board_id=board_id,
                sink_id=sink_id,
                function_id=function_id,
                payload=proto.SwitchSetCommand(outlet, msg.is_active),
            ))
            out.header.stamp = out.ds_header.io_time = rospy.Time.now()
            out_pub.publish(out)
        return cb

    rospy.Subscriber('~in', RawData, handle_raw_packet)
    rospy.Subscriber(
        '~com_switches/control', OutletStatus,
        make_switch_control_callback(
            proto.BoardID.COM,
            proto.SinkID.COM_SWITCHES,
            proto.PMS_SWITCHES_FunctionID.SET
        )
    )
    rospy.Subscriber(
        '~pms_switches/control', OutletStatus,
        make_switch_control_callback(
            proto.BoardID.PMS,
            proto.SinkID.PMS_SWITCHES,
            proto.PMS_SWITCHES_FunctionID.SET
        )
    )

    rospy.spin()


if __name__ == '__main__':
    main()
