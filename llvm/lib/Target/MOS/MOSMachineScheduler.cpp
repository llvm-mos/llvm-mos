//===-- MOSMachineScheduler.cpp - MOS Instruction Scheduler ---------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS machine instruction scheduler.
//
// The 6502 has no cache in any of its common configurations. This means that
// every instruction is loaded from memory before being executed. Essentially,
// this means that its pipeline stalls after every instruction. Accordingly, it
// doesn't matter one bit for throughput what order instructions execute in, so
// most instruction scheduling concerns are totally irrelevant.
//
// There is one important exception: register pressure. The order of
// instructions can make enormous differences in the number of registers
// required to execute a basic block. For example, it's often possible to order
// arthmetic in such a way that temporaries are threaded through the
// instructions entirely in the A register. Order the instructions differently,
// and the live ranges may begin to overlap, requiring a huge number of
// additional temporary locations.
//
// Thus, the MOS scheduling strategy more or less copies just the register
// pressure parts of the standard Machine Scheduler.
//
//===----------------------------------------------------------------------===//

#include "MOSMachineScheduler.h"
#include "llvm/CodeGen/MachineScheduler.h"

using namespace llvm;

MOSSchedStrategy::MOSSchedStrategy(const MachineSchedContext *C)
    : GenericScheduler(C) {}

void MOSSchedStrategy::tryCandidate(SchedCandidate &Cand,
                                        SchedCandidate &TryCand,
                                        SchedBoundary *Zone) const {

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

  // NOTE: Unlike in the regular instruction scheduler, nothing special is done
  // for physical registers. They're not exactly a rarity in our case like they
  // are in general, so they're not worth special-casing.
}
