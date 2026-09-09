using System;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

namespace PyLoN
{
    /// <summary>One bounded command socket per flight, independent of actuator registration.</summary>
    internal sealed class CommandReceiver : IDisposable
    {
        private readonly CommandDispatcher dispatcher;
        private readonly Action<string> warn;
        private UdpClient client;
        private int port;

        public CommandReceiver(Action<PyLoNMotorCommand> motorCommand, Action<string> warn)
        {
            dispatcher = new CommandDispatcher(motorCommand);
            this.warn = warn;
        }

        public void Pump()
        {
            if (client == null || port != RuntimeSettings.CommandPort)
            {
                Dispose();
                try
                {
                    port = RuntimeSettings.CommandPort;
                    client = new UdpClient(new IPEndPoint(IPAddress.Any, port));
                    client.Client.Blocking = false;
                    Debug.Log("[PyLoN] Commands listening on UDP port " + port);
                }
                catch (Exception ex) { Dispose(); warn("Command bind failed: " + ex.Message); return; }
            }
            for (int count = 0; count < 256; count++)
            {
                try
                {
                    IPEndPoint sender = null;
                    var bytes = client.Receive(ref sender);
                    dispatcher.Dispatch(bytes, port);
                }
                catch (SocketException ex)
                {
                    if (ex.SocketErrorCode != SocketError.WouldBlock && ex.SocketErrorCode != SocketError.TryAgain)
                        warn("Command receive failed: " + ex.Message);
                    break;
                }
                catch (Exception ex) { warn("Invalid command: " + ex.Message); }
            }
        }

        public void Dispose()
        {
            if (client != null) client.Close();
            client = null;
        }
    }
}
