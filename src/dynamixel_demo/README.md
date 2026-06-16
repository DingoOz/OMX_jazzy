# dynamixel_demo

Self-contained demos for controlling a **single Dynamixel X-series servo** on the
bench — first with the raw DynamixelSDK (no ROS), then with a minimal ROS 2 node.
Pairs with the LaTeX report in [`docs/`](docs/dynamixel_demo.tex).

Targets a Protocol 2.0 X-series servo (XL430-W250 / XM430-W350) wired to a U2D2.
Defaults assume a factory-fresh servo: **ID 1 at 57600 baud**. A servo taken from
an assembled Open Manipulator X runs at **1000000 baud** instead (`--baud 1000000`).

## Build

```bash
cd ~/Programming/OMX_jazzy
colcon build --packages-select dynamixel_demo --symlink-install --allow-overriding dynamixel_sdk
source install/setup.bash
```

The `dynamixel_sdk` Python module is already vendored in this workspace.

## Demo 1 — direct control (no ROS)

```bash
ros2 run dynamixel_demo single_servo_demo --port /dev/ttyUSB0 --id 1 --baud 57600
# or run the script directly:
python3 src/dynamixel_demo/dynamixel_demo/single_servo_demo.py --help
```

Pings the servo, enables torque, sweeps between two positions while printing the
present position, then disables torque. Flags: `--cycles`, `--min`, `--max`.

## Demo 2 — basic control with ROS 2

```bash
ros2 launch dynamixel_demo servo_demo.launch.py port:=/dev/ttyUSB0 baud:=57600 dxl_id:=1
```

| Topic | Type | Meaning |
|-------|------|---------|
| `/dxl/goal_position` | `std_msgs/Float64` | Target angle [rad] (you publish) |
| `/dxl/present_position` | `std_msgs/Float64` | Measured angle [rad] (node publishes) |

```bash
# command an angle (0 rad = encoder centre, range ~ +/- pi)
ros2 topic pub --once /dxl/goal_position std_msgs/msg/Float64 "{data: 0.5}"
# watch the measured angle
ros2 topic echo /dxl/present_position
```

## Building the PDF report

```bash
cd src/dynamixel_demo/docs
./build_pdf.sh        # needs latexmk or pdflatex; output in docs/build/
```

The PDF is **generated, not committed** — run the script to produce
`docs/build/dynamixel_demo.pdf`.
