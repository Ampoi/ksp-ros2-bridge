using System;

namespace KerbalLiDAR.Domain.Control
{
    /// <summary>A deterministic right-handed vector value with no Unity dependency.</summary>
    internal struct Vector3Value
    {
        public static readonly Vector3Value Zero = new Vector3Value(0.0, 0.0, 0.0);

        public readonly double X;
        public readonly double Y;
        public readonly double Z;

        public Vector3Value(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public double SqrMagnitude
        {
            get { return X * X + Y * Y + Z * Z; }
        }

        public double Magnitude
        {
            get { return Math.Sqrt(SqrMagnitude); }
        }

        public bool IsFinite
        {
            get { return Finite(X) && Finite(Y) && Finite(Z); }
        }

        public Vector3Value Normalized
        {
            get
            {
                var magnitude = Magnitude;
                return magnitude <= 1.0e-12 ? Zero : this / magnitude;
            }
        }

        public Vector3Value ClampMagnitude(double maximum)
        {
            if (maximum <= 0.0)
            {
                return Zero;
            }
            var magnitude = Magnitude;
            return magnitude > maximum ? this * (maximum / magnitude) : this;
        }

        public static double Dot(Vector3Value left, Vector3Value right)
        {
            return left.X * right.X + left.Y * right.Y + left.Z * right.Z;
        }

        public static Vector3Value Cross(Vector3Value left, Vector3Value right)
        {
            return new Vector3Value(
                left.Y * right.Z - left.Z * right.Y,
                left.Z * right.X - left.X * right.Z,
                left.X * right.Y - left.Y * right.X);
        }

        public static Vector3Value operator +(Vector3Value left, Vector3Value right)
        {
            return new Vector3Value(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
        }

        public static Vector3Value operator -(Vector3Value left, Vector3Value right)
        {
            return new Vector3Value(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
        }

        public static Vector3Value operator -(Vector3Value value)
        {
            return new Vector3Value(-value.X, -value.Y, -value.Z);
        }

        public static Vector3Value operator *(Vector3Value value, double scale)
        {
            return new Vector3Value(value.X * scale, value.Y * scale, value.Z * scale);
        }

        public static Vector3Value operator /(Vector3Value value, double scale)
        {
            return new Vector3Value(value.X / scale, value.Y / scale, value.Z / scale);
        }

        private static bool Finite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
