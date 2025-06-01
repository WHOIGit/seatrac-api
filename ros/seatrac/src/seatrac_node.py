#!/usr/bin/env python3
import rospy

from ds_core_msgs.msg import RawData
from seatrac.msg import PowerLevel as PowerLevelMsg
from seatrac.protocol import SeaTracMessage, PowerLevelMessage


def handle_power_level(msg: PowerLevelMessage) -> None:
    global pub
    ros_msg = PowerLevelMsg()
    ros_msg.pack_current = msg.payload.pack_current
    ros_msg.load_current = msg.payload.load_current
    ros_msg.pack_voltage = msg.payload.pack_voltage
    ros_msg.soc_percentage = msg.payload.soc_percentage
    pub.publish(ros_msg)


def handle_seatrac_message(msg: SeaTracMessage) -> None:
    if isinstance(msg.payload, PowerLevelMessage):
        handle_power_level(msg.payload)


def handle_raw_packet(in_msg: RawData) -> None:
    global buffer
    buffer.extend(in_msg.data)
    while True:
        try:
            if not (ready := SeaTracMessage.peek_length(buffer)):
                break
        except ValueError:
            break

        packet = buffer[:ready]
        del buffer[:ready]

        parsed = SeaTracMessage.from_bytes(packet)
        if not parsed.is_checksum_valid:
            rospy.logwarn('Dropped packet with invalid checksum')
            continue  # discard

        handle_seatrac_message(parsed)


def main():
    global buffer, pub
    rospy.init_node('seatrac_node')
    buffer = bytearray()
    pub = rospy.Publisher('power', PowerLevelMsg, queue_size=10)
    rospy.Subscriber('in', RawData, handle_raw_packet)
    rospy.spin()


if __name__ == '__main__':
    main()
