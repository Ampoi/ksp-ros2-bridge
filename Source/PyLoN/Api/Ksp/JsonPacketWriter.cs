using System.Globalization;
using System.Text;
using UnityEngine;
using PyLoN.Domain.Control;
namespace PyLoN
{
    internal static class JsonPacketWriter
    {
        internal static void Prefix(StringBuilder builder, string name, bool first)
        {
            if (!first) builder.Append(',');
            JsonString(builder, name);
            builder.Append(':');
        }
        internal static void AppendString(StringBuilder builder, string name, string value, bool first = false)
        {
            Prefix(builder, name, first);
            JsonString(builder, value);
        }
        internal static void AppendNumber(StringBuilder builder, string name, double value)
        {
            Prefix(builder, name, false);
            builder.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }
        internal static void AppendNumber(StringBuilder builder, string name, long value)
        {
            Prefix(builder, name, false);
            builder.Append(value.ToString(CultureInfo.InvariantCulture));
        }
        internal static void AppendBoolean(StringBuilder builder, string name, bool value)
        {
            Prefix(builder, name, false);
            builder.Append(value ? "true" : "false");
        }
        internal static void AppendVector(StringBuilder builder, string name, Vector3 value)
        {
            Prefix(builder, name, false);
            builder.Append('[').Append(value.x.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.y.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.z.ToString("R", CultureInfo.InvariantCulture)).Append(']');
        }
        internal static void AppendDoubleVector(StringBuilder builder, string name, Vector3d value)
        {
            Prefix(builder, name, false);
            builder.Append('[').Append(value.x.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.y.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.z.ToString("R", CultureInfo.InvariantCulture)).Append(']');
        }
        internal static void AppendWrench(StringBuilder builder, string name, WrenchValue value)
        {
            Prefix(builder, name, false);
            builder.Append('{');
            AppendDomainVector(builder, "force", value.Force, true);
            AppendDomainVector(builder, "torque", value.Torque, false);
            builder.Append('}');
        }
        internal static void AppendQuaternion(StringBuilder builder, string name, Quaternion value)
        {
            Prefix(builder, name, false);
            builder.Append('[').Append(value.x.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.y.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.z.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.w.ToString("R", CultureInfo.InvariantCulture)).Append(']');
        }
        internal static void JsonString(StringBuilder builder, string value)
        {
            builder.Append('"');
            var source = value ?? string.Empty;
            for (var index = 0; index < source.Length; index++)
            {
                var character = source[index];
                if (character == '\\') builder.Append("\\\\");
                else if (character == '"') builder.Append("\\\"");
                else if (character == '\n') builder.Append("\\n");
                else if (character == '\r') builder.Append("\\r");
                else builder.Append(character);
            }
            builder.Append('"');
        }
        internal static void AppendDomainVector(
            StringBuilder builder, string name, Vector3Value value, bool first)
        {
            Prefix(builder, name, first);
            builder.Append('[').Append(value.X.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.Y.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.Z.ToString("R", CultureInfo.InvariantCulture)).Append(']');
        }
    }
}
