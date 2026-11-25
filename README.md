# SeaTrac API for Python

This repository contains the `seatrac` Python package for interfacing with a [SeaTrac Systems SP-48 autonomous surface vessel](https://www.seatrac.com), along with a [ROS 2](https://ros.org) node for integration with ROS-based systems.

Documentation of the protocol is out of scope for this repository. Please contact SeaTrac Systems with questions.


## Usage

The `seatrac.protocol` module can parse the headers and validate checksums of all SeaTrac messages. The message payload is usually returned as a `bytes` object, though a small subset of messages can be fully or partially parsed, including:

  - `PowerLevelMessage`: status update related to current draw and state of charge
  - `PMSSwitchStatusMessage`: status update related to power switches
  - `SwitchSetCommand`: command for controlling a power switch

A message object can also be serialized calling `bytes()`.

```python3
from seatrac.protocol import SeaTracMessage
packet = bytes.fromhex('00ff0500080b073f000f01aae6')
msg = SeaTracMessage.from_bytes(packet)

# SeaTracMessage(
#     outbound_relay=<Relay.CAN_BUS: 8>,
#     return_relay=<Relay.DEFAULT: 0>,
#     msg_type=<MessageType.COMMAND: 11>,
#     board_id=<BoardID.PMS: 7>,
#     sink_id=<SinkID.PMS_SWITCHES: 63>,
#     function_id=<PMS_SWITCHES_FunctionID.SET: 0>,
#     timestamp=None,
#     is_checksum_valid=True,
#     payload=SwitchSetCommand(switch=15, state=True)
# )

assert bytes(msg) == packet
```


## Installation

```
python3 -m pip install git+https://github.com/WHOIGit/seatrac-api.git
```


## Command Line

A minimal command line interface is provided for testing purposes.

The command line client can listen to local UDP traffic:

```
python3 -m seatrac --listen --port 62001
```

Or connect to the SeaTrac server using a client certificate. Review [docs/ports.md](docs/ports.md) for the various port assignments.

```
python3 -m seatrac --connect --port 45107 --auth cert.pem key.pem
```

In `--connect` mode, it may be useful to use `--relay host port` which forwards the decrypted traffic as UDP packets to the specified destination.


## ROS Node

The `seatrac` ROS node interfaces between the ROS graph and the SeaTrac vessel. It supports ROS 2 Foxy Fitzroy, Humble Hawksbill, and Rolling Ridley. ROS 1 Noetic is available on the [noetic branch](https://github.com/WHOIGit/seatrac-api/tree/noetic).


### Network communication

The node does not communicate directly with the boat; instead, the `~/in` and `~/out` topics carry [wr2_msgs/msg/RawData](https://git.whoi.edu/whoi-ros/wr2_msgs/-/blob/5712cca06b294aa370efb3ceb53a2cdd2e971b9f/wr2_msgs/msg/RawData.msg) messages.

On Humble, an easy way to connect UDP traffic with these topics is to use the adapter_node from [wr2_asio](https://git.whoi.edu/whoi-ros/wr2_asio). An example is given below.

The UDP communications within the boat network can be configured using the SeaTrac Dashboard app. This is outside the scope of this documentation.


### Installation

Clone this repository and [wr2_msgs](https://git.whoi.edu/whoi-ros/wr2_msgs) into the `src/` directory of a ROS 2 workspace. For the UDP example, also clone [wr2_base](https://git.whoi.edu/whoi-ros/wr2_base) and [wr2_asio](https://git.whoi.edu/whoi-ros/wr2_asio). Install their dependencies with rosdep, build with `colcon build --packages-up-to seatrac` (add `wr2_asio` for the example), and source `install/setup.bash`.


### Launch configuration

To run the node alone, use `ros2 run seatrac seatrac_node --ros-args --params-file seatrac.yaml`.

```xml
<launch>
    <node name="seatrac" pkg="seatrac" exec="seatrac_node">
        <param from="$(dirname)/seatrac.yaml" />
        <!-- remap i/o to the adapter_node topics -->
        <remap from="~/in" to="~/comms/in" />
        <remap from="~/out" to="~/comms/out" />
    </node>

    <node namespace="seatrac" name="comms" pkg="wr2_asio" exec="adapter_node">
        <param from="$(dirname)/seatrac.yaml" />
        <remap from="ros_connection/transmit" to="~/in" />
        <remap from="ros_connection/receive" to="~/out" />
    </node>
</launch>
```

``` yaml
/seatrac:
  ros__parameters:
    outlets:
      starlink: 15

# Configuration for the adapter_node
/seatrac/comms:
  ros__parameters:
    connection:
      connection_type: udp
      listen_port: 62001
      destination_port: 62001
      destination_address: "10.1.20.88"
```


### Parameters

- `outlets.<name>`: Switch index for the named outlet (1-based)

The `/seatrac/comms` parameters in the above example belong to adapter_node.


### Subscribed Topics

- `~/in` (wr2_msgs/msg/RawData): Incoming SeaTrac packets.
- `~/outlet/<name>/control` (std_msgs/msg/Bool): Control the named outlet.


### Published Topics

- `~/out` (wr2_msgs/msg/RawData): Outgoing SeaTrac packets.
- `~/outlet/<name>/status` (seatrac/msg/OutletStatus): On/off status of the named outlet.
- `~/power` (seatrac/msg/PowerLevel): Battery pack state of charge and current draw.
