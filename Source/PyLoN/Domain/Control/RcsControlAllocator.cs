using System;
using System.Collections.Generic;

namespace PyLoN.Domain.Control
{
    internal struct RcsControlInput
    {
        public double X;
        public double Y;
        public double Z;
        public double Pitch;
        public double Yaw;
        public double Roll;
    }

    internal sealed class RcsAllocation
    {
        public readonly RcsControlInput Control;
        public readonly WrenchValue Allocated;
        public readonly WrenchValue Residual;

        public RcsAllocation(RcsControlInput control, WrenchValue allocated, WrenchValue residual)
        {
            Control = control;
            Allocated = allocated;
            Residual = residual;
        }
    }

    /// <summary>
    /// Allocates a body wrench over the twelve signed KSP RCS control directions.
    /// The adapter supplies each direction's physical wrench from real nozzle geometry.
    /// </summary>
    internal static class RcsControlAllocator
    {
        private static readonly string[] AxisNames =
        {
            "x", "y", "z", "pitch", "yaw", "roll"
        };

        public static RcsAllocation Solve(
            WrenchValue requested,
            WrenchValue[] positiveResponses,
            WrenchValue[] negativeResponses)
        {
            if (positiveResponses == null || negativeResponses == null ||
                positiveResponses.Length != 6 || negativeResponses.Length != 6)
            {
                return new RcsAllocation(new RcsControlInput(), WrenchValue.Zero, requested);
            }

            var channels = new List<WrenchChannel>(12);
            var hasForceRequest = requested.Force.Magnitude > 1.0e-6;
            var hasTorqueRequest = requested.Torque.Magnitude > 1.0e-6;
            for (var axis = 0; axis < 6; axis++)
            {
                // Pure attitude commands stay on KSP's rotational channels;
                // pure translation stays on translation channels. This avoids
                // an off-centre translation channel winning the numerical
                // solve for torque and producing a surprising control axis.
                var channelEnabled = hasForceRequest == hasTorqueRequest ||
                    (hasForceRequest ? axis < 3 : axis >= 3);
                channels.Add(new WrenchChannel
                {
                    Name = AxisNames[axis] + "+",
                    UnitWrench = positiveResponses[axis],
                    Maximum = channelEnabled ? 1.0 : 0.0
                });
                channels.Add(new WrenchChannel
                {
                    Name = AxisNames[axis] + "-",
                    UnitWrench = negativeResponses[axis],
                    Maximum = channelEnabled ? 1.0 : 0.0
                });
            }

            var forceScale = Math.Max(1.0, requested.Force.Magnitude);
            var torqueScale = Math.Max(1.0, requested.Torque.Magnitude);
            var solution = BoundedWrenchAllocator.Solve(
                channels, requested, forceScale, torqueScale, 64);
            var values = new double[6];
            for (var axis = 0; axis < 6; axis++)
            {
                values[axis] = Clamp(solution.Commands[axis * 2] - solution.Commands[axis * 2 + 1]);
            }
            var control = new RcsControlInput
            {
                X = values[0],
                Y = values[1],
                Z = values[2],
                Pitch = values[3],
                Yaw = values[4],
                Roll = values[5]
            };
            return new RcsAllocation(control, solution.Allocated, solution.Residual);
        }

        private static double Clamp(double value)
        {
            return value < -1.0 ? -1.0 : value > 1.0 ? 1.0 : value;
        }
    }
}
