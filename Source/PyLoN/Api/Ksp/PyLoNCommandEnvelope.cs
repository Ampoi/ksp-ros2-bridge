using System;
namespace PyLoN
{
    [Serializable]
    public sealed class PyLoNCommandEnvelope
    {
        public string type;
        public int version;
        public string runtimeInstance, runtimeEpoch, runtimeVesselId;
        public long runtimeGeneration;
    }

}
