using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace PyLoN
{
    /// <summary>One bounded command socket per flight, independent of actuator registration.</summary>
    internal sealed class CommandReceiver : IDisposable
    {
        private readonly Action<PyLoNMotorCommand> motorCommand;
        private readonly Action<string> warn;
        private UdpClient client;
        private int port;

        public CommandReceiver(Action<PyLoNMotorCommand> motorCommand, Action<string> warn)
        { this.motorCommand = motorCommand; this.warn = warn; }

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
                    var json = Encoding.UTF8.GetString(bytes);
                    if (!RuntimeSession.Accept(JsonUtility.FromJson<PyLoNCommandEnvelope>(json))) continue;
                    if (PyLoNVehicleManager.TryDispatch(json, port) || PyLoNDockingManager.TryDispatch(json, port)) continue;
                    var command = JsonUtility.FromJson<PyLoNMotorCommand>(json);
                    if (command != null && command.type == "pylon_motor_command" && command.version == 1)
                        motorCommand(command);
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
