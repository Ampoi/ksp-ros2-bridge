using System;
using System.Globalization;
using System.Text;
using UnityEngine;

namespace PyLoN
{
    public sealed partial class PyLoNVesselModelManager
    {
        private const int JsonVersion = 1;

        private readonly UdpPacketSender udpSender = new UdpPacketSender();
        private double lastUdpWarningTime = -1000.0;
        private readonly StringBuilder packetBuilder = new StringBuilder(8192);

        private void SendUdp(string payload)
        {
            try
            {
                udpSender.Configure(udpHost, udpPort);

                var bytes = TelemetryPacketCodec.Encode(payload);
                if (bytes.Length > maxDatagramBytes)
                {
                    WarnUdpThrottled("Model UDP packet is " + bytes.Length + " bytes; reduce model chunk size.");
                    return;
                }

                udpSender.Send(bytes);
            }
            catch (Exception ex)
            {
                WarnUdpThrottled("Model UDP send failed: " + ex.Message);
            }
        }

        private void CloseUdpClient()
        {
            udpSender.Dispose();
        }

        private void WarnUdpThrottled(string message)
        {
            var now = Planetarium.GetUniversalTime();
            if (now - lastUdpWarningTime < 5.0)
            {
                return;
            }

            lastUdpWarningTime = now;
            Debug.LogWarning("[PyLoN] " + message);
        }

        private static void AppendProperty(StringBuilder builder, string name, string value, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            AppendJsonString(builder, value);
        }

        private static void AppendProperty(StringBuilder builder, string name, int value, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            builder.Append(value.ToString(CultureInfo.InvariantCulture));
        }

        private static void AppendProperty(StringBuilder builder, string name, long value, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            builder.Append(value.ToString(CultureInfo.InvariantCulture));
        }

        private static void AppendProperty(StringBuilder builder, string name, double value, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            builder.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        private static void AppendProperty(StringBuilder builder, string name, float value, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            AppendFloat(builder, value);
        }

        private static void AppendArrayProperty(StringBuilder builder, string name, Action<StringBuilder> appendArrayBody, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            builder.Append('[');
            appendArrayBody(builder);
            builder.Append(']');
        }

        private static void AppendRawArrayProperty(StringBuilder builder, string name, StringBuilder rawBody, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            builder.Append('[');
            builder.Append(rawBody);
            builder.Append(']');
        }

        private static void AppendPropertyPrefix(StringBuilder builder, string name, bool first)
        {
            if (!first)
            {
                builder.Append(',');
            }

            AppendJsonString(builder, name);
            builder.Append(':');
        }

        private static void AppendFloat(StringBuilder builder, float value)
        {
            builder.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        private static void AppendVector3OrNull(StringBuilder builder, Vector3 value, bool hasValue)
        {
            if (!hasValue)
            {
                builder.Append("null,null,null");
                return;
            }

            AppendFloat(builder, value.x);
            builder.Append(',');
            AppendFloat(builder, value.y);
            builder.Append(',');
            AppendFloat(builder, value.z);
        }

        private static void AppendQuaternion(StringBuilder builder, Quaternion value)
        {
            AppendFloat(builder, value.x);
            builder.Append(',');
            AppendFloat(builder, value.y);
            builder.Append(',');
            AppendFloat(builder, value.z);
            builder.Append(',');
            AppendFloat(builder, value.w);
        }

        private static void AppendJsonString(StringBuilder builder, string value)
        {
            builder.Append('"');
            if (value != null)
            {
                for (var i = 0; i < value.Length; i++)
                {
                    var c = value[i];
                    switch (c)
                    {
                        case '\\':
                            builder.Append("\\\\");
                            break;
                        case '"':
                            builder.Append("\\\"");
                            break;
                        case '\n':
                            builder.Append("\\n");
                            break;
                        case '\r':
                            builder.Append("\\r");
                            break;
                        case '\t':
                            builder.Append("\\t");
                            break;
                        default:
                            if (c < 32)
                            {
                                builder.Append("\\u");
                                builder.Append(((int)c).ToString("x4", CultureInfo.InvariantCulture));
                            }
                            else
                            {
                                builder.Append(c);
                            }
                            break;
                    }
                }
            }

            builder.Append('"');
        }
    }
}
