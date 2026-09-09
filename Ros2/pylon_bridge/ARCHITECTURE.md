# Bridge communication boundary

The package still ships as one ROS 2 node. Its communication use cases are middleware-independent so a future ROS 2 or Space ROS adapter can reuse the same session and wire behavior.

```text
KSP producers -> TelemetryPacketCodec -> UDP
                                        |
                                  UdpTransport
                                        |
                          BridgeConnection / BridgeRuntime
                          protocol, session, time, transfers
                                        |
                               ROS services -> DDS

DDS commands -> ControlService -> BridgeConnection.send_command(mapping)
             -> BridgeRuntime.encode_command -> UdpTransport -> UDP
             -> KSP CommandReceiver -> CommandDispatcher -> command adapters
```

## Ownership

| Owner | Responsibility | Excluded dependencies |
| --- | --- | --- |
| `protocol.py` | v1 object envelope decoding and JSON encoding | ROS, sockets, live session |
| `transport.py` | Socket creation, receive/send bytes, endpoint and close | ROS, CLI namespace, JSON, session rules |
| `domain/session.py` | Runtime identity, generation ordering, command freshness | ROS entities, sockets |
| `application/runtime.py` | Session admission, expiration, flight-scoped clock and transfer reset, command envelope binding | ROS messages, node, socket implementation |
| `application/model_transfer.py` | Model assembly, source policy, cleared-transfer retention | ROS publications and TF |
| `application/connection.py` | Bounded polling, receive diagnostics, command sending through an injected transport | ROS executor, ROS logging APIs |
| `udp_bridge.py` | ROS composition, packet-handler registration and timer callbacks | Wire decoding and raw socket access |
| `services/*` | Message conversion, topic/QoS/TF publication and ROS state cleanup | Socket ownership and command serialization |

`ReceivedPacket.session_changed` accompanies an accepted heartbeat. The runtime resets communication state first; `FlightService` updates the ROS lifecycle identity before clearing ROS publications. Expiration is owned by the runtime and flight service. Sensor topic cleanup does not decide whether a flight session is alive.

A transport implements `receive() -> (bytes, address)`, `send(bytes, endpoint) -> int`, and `close()`. An empty nonblocking receive raises `BlockingIOError`. The connection receives a monotonic clock, warning callback, and accepted-packet callback. It can run with a memory transport and no ROS installation.

Commands stay mappings until the runtime adds the currently observed session fields and encodes once. The caller's mapping is not mutated. The existing `encode_motor_command`, `encode_vehicle_command`, `encode_docking_port_command` and `packet_conversion.decode_datagram` imports remain available for existing Python callers.

## KSP boundary

`RuntimeSession` owns identity and availability. `PyLoNSessionPublisher` owns heartbeat scheduling. `TelemetryPacketCodec` observes the runtime, adds the same v1 envelope, and encodes UTF-8 for every producer. `UdpPacketSender` shares endpoint resolution and socket lifetime for LiDAR and model producers; their size limits and diagnostics stay with those producers.

`CommandReceiver` owns the bounded command socket. `CommandDispatcher` decodes the common envelope once and checks the runtime before passing that envelope to the vehicle/docking adapters or decoding the motor command. The adapters still validate their command bodies and control authority. KSP remains responsible for lease expiry, sequence checks, emergency stop, and actuator restoration.

## Compatibility and later separation

This refactor preserves packet types/fields/version, UDP ports and endpoint configuration, command identities and sequence values, ROS topics/messages/QoS, frame conventions, and the 256-datagram polling budget. Cleanup still expires a session after `age > timeout`; command sending requires `age < timeout`. Camera/model chunk formats, checksums, size limits and retention times are unchanged.

No new package, Space ROS implementation, command acknowledgement, retransmission, queue or protocol negotiation is introduced. The current ROS 2 executable and installation procedure remain in place.

For later separation, extract `application`, `domain`, the pure packet converters/assemblers and `protocol` into a shared Python package, with `transport` as a replaceable I/O adapter. Keep ROS entity creation, QoS, topic naming and ROS message conversion in each middleware adapter. Some pure converters still include historical ROS name normalization and topic helpers; these must be separated from wire values when defining that package's public API. ROS services also still share node-owned publication state. Those remaining boundaries should be addressed when the adapter interfaces are chosen, without copying session/transport logic into each bridge.
