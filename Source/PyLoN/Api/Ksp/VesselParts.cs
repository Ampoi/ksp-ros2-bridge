using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using ModuleWheels;
using static PyLoN.JsonPacketWriter;
namespace PyLoN {
    internal sealed class VesselParts {
        private readonly Func<Vessel> current;
        public VesselParts(Func<Vessel> current) { this.current = current; }
        public IEnumerable<T> Get<T>() where T : class {
            var vessel = current();
            if (vessel == null || vessel.parts == null) yield break;
            foreach (var part in vessel.parts) {
                if (part == null) continue;
                foreach (PartModule module in part.Modules) {
                    var match = module as T;
                    if (match != null) yield return match;
                }
            }
        }
        public IEnumerable<PartModule> Separations() {
            foreach (var module in Get<PartModule>())
                if (module is ModuleDecouplerBase || module is ModuleProceduralFairing) yield return module;
        }
        internal static string SeparationMechanism(PartModule module)
        {
            if (module is ModuleDecouplerBase) return "decoupler";
            if (module is ModuleProceduralFairing) return "fairing";
            return string.Empty;
        }

        internal static bool SeparationAvailable(PartModule module)
        {
            var decoupler = module as ModuleDecouplerBase;
            if (decoupler != null)
            {
                var decoupleEvent = decoupler.Events != null && decoupler.Events.Contains("Decouple")
                    ? decoupler.Events["Decouple"]
                    : null;
                return !decoupler.isDecoupled && (decoupleEvent == null || decoupleEvent.active);
            }
            var fairing = module as ModuleProceduralFairing;
            var deployEvent = FairingDeployEvent(fairing);
            return deployEvent != null && deployEvent.active;
        }

        internal static bool SeparationComplete(PartModule module)
        {
            var decoupler = module as ModuleDecouplerBase;
            if (decoupler != null)
            {
                return decoupler.isDecoupled;
            }
            var fairing = module as ModuleProceduralFairing;
            var deployEvent = FairingDeployEvent(fairing);
            return fairing != null && deployEvent != null && !deployEvent.active;
        }

        internal static BaseEvent FairingDeployEvent(ModuleProceduralFairing fairing)
        {
            return fairing == null || fairing.Events == null || !fairing.Events.Contains("DeployFairing")
                ? null
                : fairing.Events["DeployFairing"];
        }

    }
}
