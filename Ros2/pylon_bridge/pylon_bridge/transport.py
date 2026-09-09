"""Nonblocking UDP I/O. Protocol and flight-session policy live above this layer."""

import socket


class UdpTransport:
    def __init__(self, endpoint, max_datagram_bytes=65535):
        self.max_datagram_bytes = max_datagram_bytes
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind(endpoint)
            self._socket.setblocking(False)
        except BaseException:
            self._socket.close()
            raise

    @property
    def local_endpoint(self):
        return self._socket.getsockname()

    def receive(self):
        return self._socket.recvfrom(self.max_datagram_bytes)

    def send(self, payload, endpoint):
        return self._socket.sendto(payload, endpoint)

    def close(self):
        self._socket.close()
