using System;
using System.Text;
using UnityEngine;

namespace PyLoN
{
    /// <summary>Decode and gate the envelope once before invoking a command adapter.</summary>
    internal sealed class CommandDispatcher
    {
        private readonly Action<PyLoNMotorCommand> motorCommand;

        public CommandDispatcher(Action<PyLoNMotorCommand> motorCommand)
        {
            this.motorCommand = motorCommand;
        }

        public void Dispatch(byte[] bytes, int port)
        {
            var json = Encoding.UTF8.GetString(bytes);
            var envelope = JsonUtility.FromJson<PyLoNCommandEnvelope>(json);
            if (!RuntimeSession.Accept(envelope)) return;
            if (PyLoNVehicleManager.TryDispatch(json, envelope, port) ||
                PyLoNDockingManager.TryDispatch(json, envelope, port)) return;
            if (envelope.type == "pylon_motor_command")
                motorCommand(JsonUtility.FromJson<PyLoNMotorCommand>(json));
        }
    }
}
