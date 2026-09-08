using PyLoN.Domain.Control;

namespace PyLoN.Application.Control
{
    /// <summary>Application boundary coordinating ownership and the in-process safety filter.</summary>
    internal sealed class VehicleControlApplication
    {
        public readonly ControlAuthority Authority;
        private readonly ControlSafetyFilter safety;

        public VehicleControlApplication(ControlSafetyPolicy policy)
        {
            Authority = new ControlAuthority();
            safety = new ControlSafetyFilter(policy);
        }

        public SafetyFilterResult AcceptWrench(
            string controllerId,
            string leaseId,
            long sequence,
            double now,
            WrenchValue requested,
            Vector3Value angularVelocity,
            out string rejectionReason)
        {
            if (!Authority.AcceptCommand(controllerId, leaseId, now, sequence, out rejectionReason))
            {
                return null;
            }
            return safety.Apply(requested, angularVelocity, now);
        }

        public void ResetSafety(double now)
        {
            safety.Reset(now);
        }
    }
}
