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

The `seatrac` node publishes vessel telemetry and accepts commands for configured outlets. The ROS 1 implementation is available on the [noetic branch](https://github.com/WHOIGit/seatrac-api/tree/noetic).

### Build

In a shell with ROS 2 Foxy or Rolling sourced, build the node and its [WHOI message dependencies](https://git.whoi.edu/whoi-ros/wr2_msgs) in a colcon workspace.

```sh
mkdir -p ~/seatrac_ws/src
cd ~/seatrac_ws
git clone https://github.com/WHOIGit/seatrac-api.git src/seatrac-api
git clone https://git.whoi.edu/whoi-ros/wr2_msgs.git src/wr2_msgs
colcon build --packages-up-to seatrac
. install/setup.sh
```

Foxy reached end of life in June 2023. Rolling receives breaking changes, and WHOI targets Humble with additional testing on Jazzy. The Foxy and Rolling builds were tested with WHOI revision `5712cca`. [ROS 2 distributions](https://docs.ros.org/en/rolling/Releases.html) · [WHOI-ROS](https://whoi-ros.pages.whoi.edu/)

### Run

The node exchanges [`wr2_msgs/msg/RawData`](https://git.whoi.edu/whoi-ros/wr2_msgs/-/blob/5712cca06b294aa370efb3ceb53a2cdd2e971b9f/wr2_msgs/msg/RawData.msg) with a separate transport bridge. For a bridge publishing incoming bytes on `/seatrac/comms/in` and subscribing to commands on `/seatrac/comms/out`, start the node with these remaps. The vessel’s UDP settings are configured in the SeaTrac Dashboard app.

```sh
ros2 run seatrac seatrac_node --ros-args --params-file seatrac.yaml \
  -r '~/in:=/seatrac/comms/in' -r '~/out:=/seatrac/comms/out'
```

Each entry under `outlets` in `seatrac.yaml` maps a name to a unique, one-based switch number from 1 through 18. Omit the parameters file to receive battery telemetry alone.

```yaml
/seatrac:
  ros__parameters:
    outlets:
      starlink: 15
```

| Topic | Direction | Message | Contents |
| --- | --- | --- | --- |
| `~/in` | Subscribes | `wr2_msgs/msg/RawData` | Incoming SeaTrac bytes |
| `~/out` | Publishes | `wr2_msgs/msg/RawData` | Outgoing SeaTrac commands |
| `~/power` | Publishes | `seatrac/msg/PowerLevel` | Battery current, voltage and state of charge |
| `~/outlet/<name>/status` | Publishes | `seatrac/msg/OutletStatus` | Switch state |
| `~/outlet/<name>/control` | Subscribes | `std_msgs/msg/Bool` | Requested switch state |

For telemetry, `header.stamp` is the timestamp reported by the boat and `ds_header.io_time` is the node’s receipt time. If the boat omits the timestamp, both fields carry the receipt time.
