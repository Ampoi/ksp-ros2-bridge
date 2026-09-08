using System;

namespace PyLoN.Domain
{
    // Availability model, deliberately independent of Unity and the transport.
    public sealed class StarTrackerLock
    {
        private double clearSince = double.NaN;
        private double previousTime = double.NaN;
        public void Reset() { clearSince = previousTime = double.NaN; }

        public string Update(double time, string obstruction, double acquisitionSeconds)
        {
            if (double.IsNaN(time) || double.IsInfinity(time)) { Reset(); return "invalid_time"; }
            if (!double.IsNaN(previousTime) && (time < previousTime || time - previousTime > 2.0))
                clearSince = double.NaN;
            previousTime = time;
            if (obstruction != "clear") { clearSince = double.NaN; return obstruction; }
            if (double.IsNaN(clearSince)) clearSince = time;
            return time - clearSince >= acquisitionSeconds ? "tracking" : "acquiring";
        }

        public static bool Excluded(double separationDegrees, double distance, double radius,
            double marginDegrees)
        {
            if (distance <= radius) return true;
            var angularRadius = Math.Asin(Math.Min(1.0, radius / distance)) * 180.0 / Math.PI;
            return separationDegrees <= angularRadius + marginDegrees;
        }
    }
}
