using System;
using System.Collections.Generic;

namespace PyLoN.Domain.Control
{
    internal enum ControlAuthorityMode
    {
        Unowned = 0,
        Owned = 1,
        EmergencyStop = 2
    }

    internal sealed class ControlAuthority
    {
        private readonly Dictionary<string, long> sequenceByLease =
            new Dictionary<string, long>(StringComparer.Ordinal);

        public ControlAuthorityMode Mode { get; private set; }
        public string ControllerId { get; private set; }
        public string LeaseId { get; private set; }
        public int Priority { get; private set; }
        public double ExpiresAt { get; private set; }
        public bool SuppressSas { get; private set; }
        public long LastSequence { get; private set; }
        public string Reason { get; private set; }

        public ControlAuthority()
        {
            ClearOwner("unowned");
        }

        public bool Acquire(
            string controllerId,
            string leaseId,
            int priority,
            double now,
            double duration,
            bool suppressSas,
            long sequence,
            out string reason)
        {
            Expire(now);
            if (!ValidIdentity(controllerId, leaseId) || !Finite(now) || !Finite(duration) ||
                duration < 0.1 || duration > 10.0)
            {
                reason = "invalid_lease";
                return false;
            }
            if (Mode == ControlAuthorityMode.EmergencyStop)
            {
                reason = "emergency_stop";
                return false;
            }
            var sameLease = Mode == ControlAuthorityMode.Owned &&
                ControllerId == controllerId && LeaseId == leaseId;
            if (!sameLease && Mode == ControlAuthorityMode.Owned && priority <= Priority)
            {
                reason = "authority_held_by_higher_or_equal_priority";
                return false;
            }
            if (!TryAdvanceSequence(controllerId, leaseId, sequence, out reason))
            {
                return false;
            }

            Mode = ControlAuthorityMode.Owned;
            ControllerId = controllerId;
            LeaseId = leaseId;
            Priority = priority;
            ExpiresAt = now + duration;
            SuppressSas = suppressSas;
            LastSequence = sequence;
            Reason = sameLease ? "lease_renewed" : "lease_acquired";
            reason = Reason;
            return true;
        }

        public bool Renew(
            string controllerId,
            string leaseId,
            double now,
            double duration,
            bool suppressSas,
            long sequence,
            out string reason)
        {
            Expire(now);
            if (!Owns(controllerId, leaseId))
            {
                reason = Mode == ControlAuthorityMode.EmergencyStop ? "emergency_stop" : "lease_not_owned";
                return false;
            }
            if (!Finite(duration) || duration < 0.1 || duration > 10.0)
            {
                reason = "invalid_lease";
                return false;
            }
            if (!TryAdvanceSequence(controllerId, leaseId, sequence, out reason))
            {
                return false;
            }
            ExpiresAt = now + duration;
            SuppressSas = suppressSas;
            LastSequence = sequence;
            Reason = "lease_renewed";
            reason = Reason;
            return true;
        }

        public bool Release(string controllerId, string leaseId, long sequence, out string reason)
        {
            if (!Owns(controllerId, leaseId))
            {
                reason = "lease_not_owned";
                return false;
            }
            if (!TryAdvanceSequence(controllerId, leaseId, sequence, out reason))
            {
                return false;
            }
            ClearOwner("lease_released");
            reason = Reason;
            return true;
        }

        public bool AcceptCommand(
            string controllerId,
            string leaseId,
            double now,
            long sequence,
            out string reason)
        {
            Expire(now);
            if (Mode == ControlAuthorityMode.EmergencyStop)
            {
                reason = "emergency_stop";
                return false;
            }
            if (!Owns(controllerId, leaseId))
            {
                reason = "lease_not_owned";
                return false;
            }
            if (!TryAdvanceSequence(controllerId, leaseId, sequence, out reason))
            {
                return false;
            }
            Reason = "command_accepted";
            reason = Reason;
            return true;
        }

        public bool EmergencyStop(
            string controllerId,
            string leaseId,
            long sequence,
            string requestedReason,
            out string reason)
        {
            if (!ValidIdentity(controllerId, leaseId))
            {
                reason = "invalid_lease";
                return false;
            }
            if (!TryAdvanceSequence(controllerId, leaseId, sequence, out reason))
            {
                return false;
            }
            Mode = ControlAuthorityMode.EmergencyStop;
            ControllerId = controllerId;
            LeaseId = leaseId;
            ExpiresAt = double.PositiveInfinity;
            SuppressSas = true;
            LastSequence = sequence;
            Reason = string.IsNullOrEmpty(requestedReason) ? "emergency_stop" : requestedReason;
            reason = Reason;
            return true;
        }

        public bool ClearEmergencyStop(
            string controllerId,
            string leaseId,
            long sequence,
            out string reason)
        {
            if (Mode != ControlAuthorityMode.EmergencyStop)
            {
                reason = "emergency_stop_not_active";
                return false;
            }
            if (!ValidIdentity(controllerId, leaseId) ||
                controllerId != ControllerId || leaseId != LeaseId)
            {
                reason = "emergency_stop_owner_mismatch";
                return false;
            }
            if (!TryAdvanceSequence(controllerId, leaseId, sequence, out reason))
            {
                return false;
            }
            ClearOwner("emergency_stop_cleared");
            reason = Reason;
            return true;
        }

        public bool Expire(double now)
        {
            if (Mode != ControlAuthorityMode.Owned || now <= ExpiresAt)
            {
                return false;
            }
            ClearOwner("lease_expired");
            return true;
        }

        public bool ReleaseForLifecycleChange(string reason)
        {
            if (Mode != ControlAuthorityMode.Owned)
            {
                return false;
            }
            ClearOwner(string.IsNullOrEmpty(reason) ? "lifecycle_changed" : reason);
            return true;
        }

        public bool Owns(string controllerId, string leaseId)
        {
            return Mode == ControlAuthorityMode.Owned &&
                ControllerId == controllerId && LeaseId == leaseId;
        }

        private void ClearOwner(string reason)
        {
            Mode = ControlAuthorityMode.Unowned;
            ControllerId = string.Empty;
            LeaseId = string.Empty;
            Priority = 0;
            ExpiresAt = 0.0;
            SuppressSas = false;
            Reason = reason;
        }

        private bool TryAdvanceSequence(
            string controllerId,
            string leaseId,
            long sequence,
            out string reason)
        {
            var key = controllerId + "\n" + leaseId;
            long previous;
            if (sequence <= 0 ||
                (sequenceByLease.TryGetValue(key, out previous) && sequence <= previous))
            {
                reason = "stale_sequence";
                return false;
            }
            sequenceByLease[key] = sequence;
            LastSequence = sequence;
            reason = string.Empty;
            return true;
        }

        private static bool ValidIdentity(string controllerId, string leaseId)
        {
            return !string.IsNullOrWhiteSpace(controllerId) && controllerId.Length <= 64 &&
                !string.IsNullOrWhiteSpace(leaseId) && leaseId.Length <= 64;
        }

        private static bool Finite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
