# DISAL MoCap

Standalone package allowing to extract pose information as ROS topic from the **Motion Analysis Cortex** (MACortex) system.

Tested with Cortex software version `10`.

## Overview 

This repository contains the following ROS package: 
- `macortex_bridge`

Additional legacy code is contained in the `LEGACY` directory and can be safely ignored. 

### Published topics
The `macortex_bridge` node receives the data for the defined props in Cortex and automatically publishes one topic per prop: 
- `macortex_bridge/<prop_name>/pose`, of standard type `geometry_msgs::PoseStamped`

Optionally, linear speeds can be published as well by setting `enable_speed = true` in the config file: 
- `macortex_bridge/<prop_name>/speed`, of standard type `geometry_msgs::Vector3Stamped`

Additionnally, all props are bundled into a single topic: 
- `macortex_bridge/poses`, of custom type `macortex_bridge/msg/PoseStampedArray`

The node also broadcasts TF transforms on `/tf` (type `tf2_msgs/TFMessage`):
- Parent frame: `frame_id` (default: `world`)
- Child frame (one per body): `<node_namespace>/<body_name>`

Examples:
- If `ns:=` (empty) and the body is `starling2`, the child frame is `starling2`.
- If `ns:=uav1` and the body is `starling2`, the child frame is `uav1/starling2`.

Frame IDs are sanitized for TF compatibility (no leading `/`; unsupported characters are replaced with `_`).

Quick check:
```bash
ros2 topic echo /tf
ros2 run tf2_ros tf2_echo world starling2
```

No message is published as long as no subscriber is registered. This allows to save bandwidth:
- Per-prop pose/speed topics are only published when they have subscribers.
- The aggregated `macortex_bridge/poses` topic is only published when it has subscribers and `prop_name` is empty.

## Installation

The packages can be built with:
```bash
colcon build --packages-select macortex_bridge
source install/setup.bash
```

### Cortex library
The `macortex_bridge` package requires to be linked against the cortex library (compiled in `./lib`). The library is built automatically when building the `macortex_bridge` package. However, it can still be built independently if required with:
```bash
cd lib
./build.sh 
```

## Running macortex_bridge

The package can be run with default settings (as defined in `./config/default.yaml`) with:
```bash
ros2 launch macortex_bridge macortex_bridge.launch.py config:=default.yaml ns:=
```
This will let the node publish all poses combined into a single topic (`macortex_bridge/poses`), as well as publishing them in individual topics based on the prop name (`macortex_bridge/<prop_name>/pose`). 
This behavior can be altered based on the provided arguments to the launch file:
- `config`: YAML configuration file in the package's `config/` directory (default: `default.yaml`)
- `ns`: namespace to use (default: empty = root namespace)

Following parameters are available in the config file (see [`config/default.yaml`](./config/default.yaml)) to adjust the node behavior: 
- Base parameters:
    - `cortex.my_ip`: local NIC IP on the Cortex/MCS network. Leave empty to auto-detect.
    - `cortex.host_ip`: MCS/Cortex hostname/IP. Leave empty for SDK auto-discovery.
    - `rate`: publish rate in Hz (default: `30.0`)
    - `frame_id`: frame to express the data in (default: `world`)
    - `prop_name`: track a single prop; empty tracks all (default: empty)
    - `enable_speed`: publish linear speeds (default: `false`)
    - `speed_to_body`: express speed in `base_link` (default: `false`)
    - `cutoff`: speed low-pass cutoff frequency in Hz; disabled if `<= 0.0` (default: `0.0`)

> Note: `macortex_bridge/poses` uses a custom message type. Make sure you have sourced the workspace overlay
> (`source install/setup.bash`) in the same terminal where you run `ros2 topic echo`.

The node name and topics are set relative to the namespace. This allows running several `macortex_bridge` nodes on the same ROS network as long as they belong to **different namespaces**, and they run on **different hosts**.
> Note: Only one node can run on a device at a time (Cortex limitation)!
