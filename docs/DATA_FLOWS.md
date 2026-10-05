# Key Data Flows

## Teleoperation

Operator keyboard -> UDP JSON @ 20 Hz -> Communication Pi -> `/teleop/cmd_vel_raw` -> Safety Supervisor -> `/cmd_vel` -> Motor Pi/OpenCR.

## Safety override

Sensor Pi `/scan` -> Safety Supervisor. If obstacle/timeout/E-stop/mode blocks motion, Safety Supervisor publishes zero velocity regardless of the operator command.

## Telemetry

ROS 2 state -> Status Aggregator -> `/robot/status` -> Communication Pi ZeroMQ PUB -> Operator dashboard/logger subscribers.

## Photo

Operator `GET /api/snapshot` -> Communication Pi REST node -> ROS 2 `/camera/take_snapshot` -> Sensor Pi snapshot server -> latest `/camera/image_raw` -> JPEG bytes -> REST response -> operator snapshot file.

## Reliable command

Operator TCP JSON -> Communication Pi -> ROS 2 service (`set_mode`, `set_max_speed`, `set_estop`) -> service response -> TCP JSON ACK.

## Local maintenance

Technician laptop/phone -> Bluetooth RFCOMM -> Communication Pi -> cached `/robot/status` for diagnostics or local E-stop topic.
