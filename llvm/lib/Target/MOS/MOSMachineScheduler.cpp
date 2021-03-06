#include "MOSMachineScheduler.h"
#include "llvm/CodeGen/MachineScheduler.h"

using namespace llvm;

MOSSchedStrategy::MOSSchedStrategy(const MachineSchedContext *C)
    : GenericScheduler(C) {}

void MOSSchedStrategy::tryCandidate(SchedCandidate &Cand,
                                        SchedCandidate &TryCand,
                                        SchedBoundary *Zone) const {
  // Note: This code is taken wholesale from GenericScheduler, but only register
  // pressure is modeled, as the 6502 has no cache, modern pipeline, or other
  // scheduling-dependent features. Scheduling is still very important, since
  // GPR and flag register pressure is very high.

  // Initialize the candidate if needed.
  if (!Cand.isValid()) {
    TryCand.Reason = NodeOrder;
    return;
  }

  // Avoid exceeding the target's limit.
  if (DAG->isTrackingPressure() && tryPressure(TryCand.RPDelta.Excess,
                                               Cand.RPDelta.Excess,
                                               TryCand, Cand, RegExcess, TRI,
                                               DAG->MF))
    return;

  // Avoid increasing the max critical pressure in the scheduled region.
  if (DAG->isTrackingPressure() && tryPressure(TryCand.RPDelta.CriticalMax,
                                               Cand.RPDelta.CriticalMax,
                                               TryCand, Cand, RegCritical, TRI,
                                               DAG->MF))
    return;

  // Avoid increasing the max pressure of the entire region.
  if (DAG->isTrackingPressure() && tryPressure(TryCand.RPDelta.CurrentMax,
                                               Cand.RPDelta.CurrentMax,
                                               TryCand, Cand, RegMax, TRI,
                                               DAG->MF))
    return;

  // We only compare a subset of features when comparing nodes between
  // Top and Bottom boundary. Some properties are simply incomparable, in many
  // other instances we should only override the other boundary if something
  // is a clear good pick on one boundary. Skip heuristics that are more
  // "tie-breaking" in nature.
  bool SameBoundary = Zone != nullptr;
  if (SameBoundary) {
    // Fall through to original instruction order.
    if ((Zone->isTop() && TryCand.SU->NodeNum < Cand.SU->NodeNum)
        || (!Zone->isTop() && TryCand.SU->NodeNum > Cand.SU->NodeNum)) {
      TryCand.Reason = NodeOrder;
    }
  }
}
