using System;

namespace PyLoN.Domain.Control
{
    internal sealed class ControlSafetyPolicy
    {
        public double MaximumForce = 250000.0;
        public double MaximumTorque = 100000.0;
        public double MaximumAngularSpeed = 0.35;
        public double MaximumForceSlew = 50000.0;
        public double MaximumTorqueSlew = 10000.0;
        public double MaximumContinuousActuation = 30.0;
        public double ResetIdleDuration = 0.5;
    }

    internal sealed class SafetyFilterResult
    {
        public readonly WrenchValue Wrench;
        public readonly bool Limited;
        public readonly string Reason;

        public SafetyFilterResult(WrenchValue wrench, bool limited, string reason)
        {
            Wrench = wrench;
            Limited = limited;
            Reason = reason;
        }
    }

    /// <summary>Last-resort limits that execute inside the KSP process.</summary>
    internal sealed class ControlSafetyFilter
    {
        private readonly ControlSafetyPolicy policy;
        private WrenchValue previous;
        private double previousAt;
        private double activeSince = -1.0;
        private double idleSince = -1.0;
        private bool continuousLimitTripped;

        public ControlSafetyFilter(ControlSafetyPolicy policy)
        {
            this.policy = policy ?? new ControlSafetyPolicy();
        }

        public SafetyFilterResult Apply(WrenchValue requested, Vector3Value angularVelocity, double now)
        {
            if (!requested.IsFinite || !angularVelocity.IsFinite || !Finite(now))
            {
                Reset(now);
                return new SafetyFilterResult(WrenchValue.Zero, true, "non_finite_command");
            }

            var reasons = string.Empty;
            var force = requested.Force.ClampMagnitude(policy.MaximumForce);
            var torque = requested.Torque.ClampMagnitude(policy.MaximumTorque);
            if ((force - requested.Force).SqrMagnitude > 1.0e-8 ||
                (torque - requested.Torque).SqrMagnitude > 1.0e-8)
            {
                reasons = AppendReason(reasons, "magnitude_limit");
            }
            if (force.SqrMagnitude + torque.SqrMagnitude < 1.0e-12)
            {
                if (idleSince < 0.0)
                {
                    idleSince = now;
                }
                if (now - idleSince >= policy.ResetIdleDuration)
                {
                    continuousLimitTripped = false;
                    activeSince = -1.0;
                }
            }
            else
            {
                idleSince = -1.0;
                if (activeSince < 0.0)
                {
                    activeSince = now;
                }
                if (now - activeSince > policy.MaximumContinuousActuation)
                {
                    continuousLimitTripped = true;
                }
            }

            if (continuousLimitTripped)
            {
                previous = WrenchValue.Zero;
                previousAt = now;
                return new SafetyFilterResult(WrenchValue.Zero, true, "continuous_actuation_limit");
            }

            var angularSpeed = angularVelocity.Magnitude;
            if (angularSpeed > policy.MaximumAngularSpeed && angularSpeed > 1.0e-9)
            {
                var axis = angularVelocity / angularSpeed;
                var acceleratingTorque = Vector3Value.Dot(torque, axis);
                if (acceleratingTorque > 0.0)
                {
                    torque -= axis * acceleratingTorque;
                    reasons = AppendReason(reasons, "angular_speed_limit");
                }
            }

            if (previousAt > 0.0 && now > previousAt)
            {
                var elapsed = Math.Min(1.0, now - previousAt);
                var forceDelta = (force - previous.Force).ClampMagnitude(policy.MaximumForceSlew * elapsed);
                var torqueDelta = (torque - previous.Torque).ClampMagnitude(policy.MaximumTorqueSlew * elapsed);
                var limitedForce = previous.Force + forceDelta;
                var limitedTorque = previous.Torque + torqueDelta;
                if ((limitedForce - force).SqrMagnitude > 1.0e-8 ||
                    (limitedTorque - torque).SqrMagnitude > 1.0e-8)
                {
                    reasons = AppendReason(reasons, "slew_rate_limit");
                }
                force = limitedForce;
                torque = limitedTorque;
            }

            previous = new WrenchValue(force, torque);
            previousAt = now;
            return new SafetyFilterResult(previous, reasons.Length > 0, reasons.Length > 0 ? reasons : "accepted");
        }

        public void Reset(double now)
        {
            previous = WrenchValue.Zero;
            previousAt = Finite(now) ? now : 0.0;
            activeSince = -1.0;
            idleSince = previousAt;
            continuousLimitTripped = false;
        }

        private static string AppendReason(string current, string next)
        {
            return current.Length == 0 ? next : current + "," + next;
        }

        private static bool Finite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
