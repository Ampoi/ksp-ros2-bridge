using System;
using System.Collections.Generic;

namespace PyLoN.Domain.Control
{
    internal sealed class WrenchChannel
    {
        public string Name;
        public WrenchValue UnitWrench;
        public double Maximum = 1.0;
    }

    internal sealed class WrenchAllocation
    {
        public readonly double[] Commands;
        public readonly WrenchValue Allocated;
        public readonly WrenchValue Residual;

        public WrenchAllocation(double[] commands, WrenchValue allocated, WrenchValue residual)
        {
            Commands = commands;
            Allocated = allocated;
            Residual = residual;
        }
    }

    /// <summary>Projected coordinate descent for bounded, one-way actuator channels.</summary>
    internal static class BoundedWrenchAllocator
    {
        public static WrenchAllocation Solve(
            IList<WrenchChannel> channels,
            WrenchValue requested,
            double forceScale,
            double torqueScale,
            int iterations = 48)
        {
            var commands = new double[channels == null ? 0 : channels.Count];
            if (channels == null || channels.Count == 0 || !requested.IsFinite)
            {
                return new WrenchAllocation(commands, WrenchValue.Zero, requested);
            }

            var allocated = WrenchValue.Zero;
            var safeForceScale = forceScale > 1.0e-9 ? forceScale : 1.0;
            var safeTorqueScale = torqueScale > 1.0e-9 ? torqueScale : 1.0;
            var passCount = Math.Max(1, iterations);
            for (var pass = 0; pass < passCount; pass++)
            {
                var changed = false;
                for (var index = 0; index < channels.Count; index++)
                {
                    var channel = channels[index];
                    if (channel == null || !channel.UnitWrench.IsFinite || channel.Maximum <= 0.0)
                    {
                        continue;
                    }

                    var column = channel.UnitWrench;
                    var residual = allocated - requested;
                    var denominator = WeightedDot(column, column, safeForceScale, safeTorqueScale) + 1.0e-8;
                    var gradient = WeightedDot(column, residual, safeForceScale, safeTorqueScale) +
                        commands[index] * 1.0e-6;
                    var next = Clamp(commands[index] - gradient / denominator, 0.0, channel.Maximum);
                    var delta = next - commands[index];
                    if (Math.Abs(delta) <= 1.0e-8)
                    {
                        continue;
                    }
                    commands[index] = next;
                    allocated += column * delta;
                    changed = true;
                }
                if (!changed)
                {
                    break;
                }
            }
            return new WrenchAllocation(commands, allocated, requested - allocated);
        }

        private static double WeightedDot(
            WrenchValue left,
            WrenchValue right,
            double forceScale,
            double torqueScale)
        {
            return Vector3Value.Dot(left.Force, right.Force) / (forceScale * forceScale) +
                Vector3Value.Dot(left.Torque, right.Torque) / (torqueScale * torqueScale);
        }

        private static double Clamp(double value, double minimum, double maximum)
        {
            return value < minimum ? minimum : value > maximum ? maximum : value;
        }
    }
}
