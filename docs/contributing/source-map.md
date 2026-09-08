# ソース案内

PyLoN本体の変更箇所を探すための案内です。パスはリポジトリのルートからの相対パスです。

## Active vesselモデル

- `Source/PyLoN/Api/Ksp/PyLoNVesselModelManager.Geometry.cs`
- `Ros2/pylon_bridge/pylon_bridge/vessel_model.py`
- `Ros2/pylon_bridge/pylon_bridge/udp_bridge.py`

## RGBカメラ

- `GameData/PyLoN/Parts/RgbCamera/part.cfg`
- `Source/PyLoN/Api/Ksp/ModulePyLoNRgbCamera.cs`
- `Ros2/pylon_bridge/pylon_bridge/camera_packets.py`

## 2D / 3D LiDAR

- `GameData/PyLoN/Parts/Lidar2D/part.cfg`
- `GameData/PyLoN/Parts/Lidar3D/part.cfg`
- `Source/PyLoN/Api/Ksp/ModulePyLoNLidar.cs`
- `Ros2/pylon_bridge/pylon_bridge/packet_conversion.py`

## サーボ / リニアモーター

- `GameData/PyLoN/Parts/RosServo/part.cfg`
- `GameData/PyLoN/Parts/RosLinearMotor/part.cfg`
- `Source/PyLoN/Api/Ksp/PyLoNMotorSupport.cs`
- `Ros2/pylon_bridge/pylon_bridge/motor_packets.py`
- `Ros2/pylon_interfaces/msg/MotorCommand.msg`
- `Ros2/pylon_interfaces/msg/MotorState.msg`

## エンジン / RCS

- `Source/PyLoN/Api/Ksp/PyLoNVehicleSupport.cs`
- `Source/PyLoN/Api/Ksp/ActuatorTelemetry.cs`
- `Ros2/pylon_bridge/pylon_bridge/services/control.py`
- `Ros2/pylon_bridge/pylon_bridge/services/vehicle_state.py`
- `Ros2/pylon_bridge/pylon_bridge/udp_bridge.py`
- `Ros2/pylon_interfaces/msg/EngineCommand.msg`
- `Ros2/pylon_interfaces/msg/EngineState.msg`
- `Ros2/pylon_interfaces/msg/RcsCommand.msg`
- `Ros2/pylon_interfaces/msg/RcsState.msg`

## KSP標準ホイール

- `Source/PyLoN/Api/Ksp/PyLoNVehicleSupport.cs`
- `Ros2/pylon_interfaces/msg/WheelCommand.msg`
- `Ros2/pylon_interfaces/msg/WheelState.msg`
