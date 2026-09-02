using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace KerbalLiDAR
{
    public partial class ModuleKerbalLidar
    {
        private const int JsonVersion = 1;

        private UdpClient udpClient;
        private IPEndPoint udpEndPoint;
        private string endpointKey;
        private double lastUdpWarningTime = -1000.0;
        private readonly StringBuilder packetBuilder = new StringBuilder(8192);

        private void SendUdp(string payload)
        {
            try
            {
                EnsureUdpClient();

                var bytes = Encoding.UTF8.GetBytes(payload);
                if (bytes.Length > maxDatagramBytes)
                {
                    WarnUdpThrottled("LiDAR UDP packet is " + bytes.Length + " bytes; reduce laser count or disable optional arrays.");
                    return;
                }

                udpClient.Send(bytes, bytes.Length, udpEndPoint);
            }
            catch (Exception ex)
            {
                WarnUdpThrottled("LiDAR UDP send failed: " + ex.Message);
            }
        }

        private void EnsureUdpClient()
        {
            var host = string.IsNullOrEmpty(udpHost) ? "127.0.0.1" : udpHost;
            var key = host + ":" + udpPort.ToString(CultureInfo.InvariantCulture);
            if (udpClient != null && udpEndPoint != null && endpointKey == key)
            {
                return;
            }

            CloseUdpClient();
            udpClient = new UdpClient();
            udpEndPoint = new IPEndPoint(ResolveAddress(host), udpPort);
            endpointKey = key;
        }

        private static IPAddress ResolveAddress(string host)
        {
            IPAddress address;
            if (IPAddress.TryParse(host, out address))
            {
                return address;
            }

            var addresses = Dns.GetHostAddresses(host);
            if (addresses.Length == 0)
            {
                throw new InvalidOperationException("Could not resolve UDP host '" + host + "'.");
            }

            return addresses[0];
        }

        private void CloseUdpClient()
        {
            if (udpClient != null)
            {
                udpClient.Close();
                udpClient = null;
            }

            udpEndPoint = null;
            endpointKey = null;
        }

        private void WarnUdpThrottled(string message)
        {
            var now = Planetarium.GetUniversalTime();
            if (now - lastUdpWarningTime < 5.0)
            {
                return;
            }

            lastUdpWarningTime = now;
            Debug.LogWarning("[KerbalLiDAR] " + message);
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
