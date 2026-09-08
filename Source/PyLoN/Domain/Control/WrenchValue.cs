namespace PyLoN.Domain.Control
{
    /// <summary>Force in newtons and torque in newton-metres, both in ROS body axes.</summary>
    internal struct WrenchValue
    {
        public static readonly WrenchValue Zero = new WrenchValue(Vector3Value.Zero, Vector3Value.Zero);

        public readonly Vector3Value Force;
        public readonly Vector3Value Torque;

        public WrenchValue(Vector3Value force, Vector3Value torque)
        {
            Force = force;
            Torque = torque;
        }

        public bool IsFinite
        {
            get { return Force.IsFinite && Torque.IsFinite; }
        }

        public bool IsNearlyZero
        {
            get { return Force.SqrMagnitude + Torque.SqrMagnitude <= 1.0e-12; }
        }

        public double WeightedNorm(double forceScale, double torqueScale)
        {
            var safeForceScale = forceScale > 1.0e-9 ? forceScale : 1.0;
            var safeTorqueScale = torqueScale > 1.0e-9 ? torqueScale : 1.0;
            return System.Math.Sqrt(
                Force.SqrMagnitude / (safeForceScale * safeForceScale) +
                Torque.SqrMagnitude / (safeTorqueScale * safeTorqueScale));
        }

        public static WrenchValue operator +(WrenchValue left, WrenchValue right)
        {
            return new WrenchValue(left.Force + right.Force, left.Torque + right.Torque);
        }

        public static WrenchValue operator -(WrenchValue left, WrenchValue right)
        {
            return new WrenchValue(left.Force - right.Force, left.Torque - right.Torque);
        }

        public static WrenchValue operator *(WrenchValue value, double scale)
        {
            return new WrenchValue(value.Force * scale, value.Torque * scale);
        }
    }
}
