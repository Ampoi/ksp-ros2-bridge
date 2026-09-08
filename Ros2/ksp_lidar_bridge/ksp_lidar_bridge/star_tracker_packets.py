"""Strict, orientation-only star tracker UDP contract."""
from dataclasses import dataclass
import math
import re
from typing import Any, Mapping, Optional, Tuple

REASONS = frozenset(('tracking', 'acquiring', 'disabled', 'no_power', 'atmosphere',
                     'sun_exclusion', 'body_in_fov', 'occluded', 'slew_rate_exceeded',
                     'packed', 'sensor_unavailable', 'invalid_time', 'inactive'))


@dataclass(frozen=True)
class StarTrackerData:
    vessel_id: str
    sensor_id: str
    session_id: str
    sequence: int
    universal_time: float
    valid: bool
    reason: str
    orientation: Optional[Tuple[float, ...]]
    noise_std_rad: float
    angular_rate_deg_sec: float


def finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite numeric data')
    return float(value)


def star_tracker_from_packet(packet: Mapping[str, Any]) -> StarTrackerData:
    if packet.get('type') != 'ksp_star_tracker' or type(packet.get('version')) is not int or packet['version'] != 1:
        raise ValueError('unsupported star tracker packet')
    for field in ('vesselId', 'sessionId'):
        if not isinstance(packet.get(field), str) or re.fullmatch(r'[0-9a-f]{32}', packet[field]) is None:
            raise ValueError(f'invalid {field}')
    sensor = packet.get('sensorId')
    if not isinstance(sensor, str) or re.fullmatch(r'[a-z_][a-z0-9_]{0,63}', sensor) is None:
        raise ValueError('invalid sensorId')
    sequence = packet.get('sequence')
    if type(sequence) is not int or not 0 < sequence < 2**64:
        raise ValueError('invalid sequence')
    if packet.get('referenceFrame') != 'kerbol_inertial' or packet.get('measuredFrame') != 'base_link':
        raise ValueError('unsupported attitude reference frames')
    valid, reason = packet.get('valid'), packet.get('reason')
    if type(valid) is not bool or not isinstance(reason, str) or reason not in REASONS or valid != (reason == 'tracking'):
        raise ValueError('inconsistent validity/reason')
    orientation = packet.get('orientation')
    if valid:
        if not isinstance(orientation, list) or len(orientation) != 4:
            raise ValueError('valid attitude needs a quaternion')
        orientation = tuple(finite(v, 'orientation') for v in orientation)
        norm = math.sqrt(sum(v*v for v in orientation))
        if abs(norm - 1) > .001:
            raise ValueError('orientation must have unit norm')
        orientation = tuple(v/norm for v in orientation)
    elif orientation is not None:
        raise ValueError('invalid measurement must not carry orientation')
    sigma = finite(packet.get('noiseStdRad'), 'noiseStdRad')
    rate = finite(packet.get('angularRateDegSec'), 'angularRateDegSec')
    if not 0 < sigma <= 1 or rate < 0:
        raise ValueError('invalid uncertainty or angular rate')
    return StarTrackerData(packet['vesselId'], sensor, packet['sessionId'], sequence,
                           finite(packet.get('universalTime'), 'universalTime'), valid,
                           reason, orientation, sigma, rate)
