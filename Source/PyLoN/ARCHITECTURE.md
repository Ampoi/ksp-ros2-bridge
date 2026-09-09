# PyLoN architecture

PyLoN is a KSP sensor, vessel-model and control platform. LiDAR is one sensor adapter. The plugin uses .NET 4.8; the bridge is one ROS 2 Jazzy node composed of explicit services.

| Area | Responsibility | Dependencies |
|---|---|---|
| Source/PyLoN/Domain | Authority, allocation, safety and identity rules | .NET base library |
| Source/PyLoN/Application | Control use cases | Domain |
| Source/PyLoN/Api/Ksp | KSP lifecycle, physics and rendering adapters | Application, Domain, KSP, Unity |
| Ros2/pylon_interfaces | Typed public ROS contracts | ROS message packages |
| Ros2/pylon_bridge | UDP and ROS adapters; domain session/time rules | Interfaces, standard ROS |
| Ros2/pylon_vehicle_control | Reusable control law and lease coordinator | Interfaces, standard ROS |
| Ros2/pylon_perception | Optional point-cloud primitives | NumPy |
| Demo | Three runnable examples | Public packages only |
| Migration | One-shot conversion and recognized-install retirement | Python standard library |
| Development | Ignored local probes, authoring and evidence | Never a production dependency |

## Services and state

The bridge composes sensor, camera, model, flight-session, control, vehicle-state and star-tracker services. The node owns ROS entities and their shared context. BridgeRuntime owns flight-scoped communication state, composing SessionTracker, SimulationClock and packet assemblers; BridgeConnection owns bounded polling and session-bound command sending. UdpTransport only receives and sends bytes. The application layer imports no ROS messages or node APIs. No dynamic method forwarding or mixin inheritance is used.

In KSP, RuntimeSession owns process identity and flight generations. A dedicated heartbeat works without LiDAR or ground truth. RuntimeSettings owns the common transport destination. CommandReceiver owns one bounded command socket independently of motor registration; CommandDispatcher decodes and validates the common envelope once before invoking command adapters. TelemetryPacketCodec adds the common v1 envelope for every producer, and UdpPacketSender shares LiDAR/model socket and endpoint handling. The model producer never reads LiDAR settings. VesselTelemetry owns truth origins and derivatives independently of control authority. ActuatorTelemetry owns read-only actuator publication and wheel geometry, with VesselParts providing shared enumeration. FrameConversions and JsonPacketWriter provide common frame/unit and packet-value conversion.

KspSceneRgbCapture orchestrates cameras; SourceCamera owns save/restore; RgbCaptureResources owns screen-sized intermediate targets, output buffers and readback. Existing temporal-effect compatibility remains isolated in the capture adapter and antialiasing guard.

See [Bridge communication boundary](../../Ros2/pylon_bridge/ARCHITECTURE.md) for ownership and the future middleware split.

## Wire and lifecycle contract

Every datagram has a `pylon_*` type and integer `version: 1`. Telemetry also carries `runtimeInstance`, `runtimeGeneration`, `runtimeEpoch`, and `runtimeVesselId`. Only `pylon_session` heartbeats introduce a session. Previously observed process IDs and lower generations are rejected. Session changes clear time mapping, incomplete camera/model transfers, sensor state, TF snapshots and pending control intent. Delayed packets cannot change the active vessel.

Sampling uses KSP universal time; receiving and expiration use monotonic wall time. Transport delays do not rebase a flight's timestamp mapping. Commands require a fresh session; mutating control commands additionally require exact vessel/controller/lease IDs and a positive increasing sequence. KSP remains the authority for timeouts, emergency stop and actuator restoration.

`/ksp_vessel` remains the vessel Topic prefix. Ground truth is optional evaluation output. It does not create lifecycle identity. `base_link` is the center-of-mass frame; fixed part/sensor mounts use `/tf_static`, moving mounts and the CoM-to-root edge use `/tf`. Sensor IDs remain configurable.

## Build boundary

`sync.sh` builds the three core ROS packages by default. `--demo` selects a named demo; `--all-demos` selects all three. Optional dependencies are never imported by core packages. The explicit C# compile list excludes local debugging sources. Public builds do not import Development, Tools, debug projects or their outputs.
