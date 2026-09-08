"""Pure lease lifecycle used by ROS adapters."""

from dataclasses import dataclass
import uuid
from typing import Optional


@dataclass(frozen=True)
class LeaseAction:
    action: str
    vessel_id: str
    controller_id: str
    lease_id: str


class LeaseCoordinator:
    def __init__(
        self,
        controller_id: str,
        renew_period_sec: float,
    ) -> None:
        if not controller_id or len(controller_id) > 64:
            raise ValueError("controller_id must contain 1 to 64 characters")
        self.controller_id = controller_id
        self.renew_period_sec = renew_period_sec
        self.vessel_id = ""
        self.generation = None
        self.lease_id = uuid.uuid4().hex
        self.owned = False
        self.next_request_at = 0.0

    def observe_vessel(self, vessel_id: str, active: bool, generation=None) -> bool:
        normalized = vessel_id if active else ""
        if normalized == self.vessel_id and generation == self.generation:
            return False
        self.generation = generation
        self.vessel_id = normalized
        self.lease_id = uuid.uuid4().hex
        self.owned = False
        self.next_request_at = 0.0
        return True

    def observe_authority(
        self,
        vessel_id: str,
        controller_id: str,
        lease_id: str,
        owned: bool,
    ) -> bool:
        was_owned = self.owned
        self.owned = bool(
            owned
            and vessel_id == self.vessel_id
            and controller_id == self.controller_id
            and lease_id == self.lease_id
        )

        return was_owned and not self.owned

    def due_action(self, now_sec: float) -> Optional[LeaseAction]:
        if not self.vessel_id or now_sec < self.next_request_at:
            return None
        self.next_request_at = now_sec + self.renew_period_sec
        return LeaseAction(
            "renew" if self.owned else "acquire",
            self.vessel_id,
            self.controller_id,
            self.lease_id,
        )

    def release_action(self) -> Optional[LeaseAction]:
        if not self.vessel_id or not self.owned:
            return None
        self.owned = False
        return LeaseAction("release", self.vessel_id, self.controller_id, self.lease_id)
