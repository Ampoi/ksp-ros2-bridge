"""Bounded UDP transport. Outgoing commands are bound to the observed flight session."""
import json
import socket
import time

class UdpTransport:
    def __init__(self,args,session):
        self.args=args;self.session=session
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            self.socket.bind((args.host,args.port));self.socket.setblocking(False)
        except BaseException:
            self.socket.close();raise
    def send(self,payload,endpoint):
        packet=json.loads(payload.decode('utf-8'))
        packet.update(self.session.command_fields(time.monotonic(),self.args.topic_timeout_sec))
        return self.socket.sendto(json.dumps(packet,separators=(',',':'),allow_nan=False).encode(),endpoint)
    def close(self):self.socket.close()
