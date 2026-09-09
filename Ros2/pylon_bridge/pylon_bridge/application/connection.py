"""Bounded receive/send use cases over an injected datagram transport."""

import time


class BridgeConnection:
    def __init__(self, transport, runtime, command_endpoint, session_timeout,
                 on_packet, warn, monotonic=time.monotonic):
        self.transport = transport
        self.runtime = runtime
        self.command_endpoint = command_endpoint
        self.session_timeout = session_timeout
        self.on_packet = on_packet
        self.warn = warn
        self.monotonic = monotonic

    def poll(self, limit=256) -> None:
        for _ in range(limit):
            try:
                data, address = self.transport.receive()
            except BlockingIOError:
                return
            except (OSError, ValueError) as exc:
                self.warn(f"UDP receive failed: {exc}")
                return
            try:
                event = self.runtime.receive(data, self.monotonic())
            except (ValueError, RecursionError) as exc:
                self.warn(f"Dropped invalid PyLoN packet: {exc}")
                continue
            if event is not None:
                self.on_packet(event, address)

    def send_command(self, command) -> int:
        payload = self.runtime.encode_command(
            command, self.monotonic(), self.session_timeout)
        return self.transport.send(payload, self.command_endpoint)

    def close(self) -> None:
        self.transport.close()
