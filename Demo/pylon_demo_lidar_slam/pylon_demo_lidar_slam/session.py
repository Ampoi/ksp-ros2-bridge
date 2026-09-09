"""Keep mapping and control bound to one continuous flight session."""


class SessionGuard:
    def __init__(self):
        self.identity = None
        self.fault = ""

    @property
    def ready(self):
        return self.identity is not None and not self.fault

    def stop(self, reason):
        self.fault = self.fault or reason

    def observe(self, vessel_id, active, generation):
        if self.fault:
            return
        if not active or not vessel_id:
            if self.identity is not None:
                self.stop("session_unavailable")
            return
        identity = (vessel_id, generation)
        if self.identity is not None and identity != self.identity:
            self.stop("session_changed")
        else:
            self.identity = identity
