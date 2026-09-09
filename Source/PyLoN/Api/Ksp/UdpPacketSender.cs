using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;

namespace PyLoN
{
    /// <summary>Reusable endpoint resolution and socket lifetime; payload policy stays with producers.</summary>
    internal sealed class UdpPacketSender : IDisposable
    {
        private UdpClient client;
        private IPEndPoint endpoint;
        private string endpointKey;

        public void Configure(string configuredHost, int port)
        {
            var host = string.IsNullOrEmpty(configuredHost) ? "127.0.0.1" : configuredHost;
            var key = host + ":" + port.ToString(CultureInfo.InvariantCulture);
            if (client != null && endpoint != null && endpointKey == key) return;
            Dispose();
            client = new UdpClient();
            endpoint = new IPEndPoint(ResolveAddress(host), port);
            endpointKey = key;
        }

        public void Send(byte[] bytes)
        {
            client.Send(bytes, bytes.Length, endpoint);
        }

        internal static IPAddress ResolveAddress(string host)
        {
            IPAddress address;
            if (IPAddress.TryParse(host, out address)) return address;
            var addresses = Dns.GetHostAddresses(host);
            if (addresses.Length == 0)
                throw new InvalidOperationException("Could not resolve UDP host '" + host + "'.");
            return addresses[0];
        }

        public void Dispose()
        {
            if (client != null) client.Close();
            client = null;
            endpoint = null;
            endpointKey = null;
        }
    }
}
