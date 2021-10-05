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
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOSRegisterInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineScheduler.h"

using namespace llvm;

MOSSchedStrategy::MOSSchedStrategy(const MachineSchedContext *C)
    : GenericScheduler(C) {}

bool MOSSchedStrategy::tryCandidate(SchedCandidate &Cand,
                                    SchedCandidate &TryCand,
                                    SchedBoundary *Zone) const {

  // Initialize the candidate if needed.
  if (!Cand.isValid()) {
    TryCand.Reason = NodeOrder;
    return true;
  }

  if (tryLess(
          registerClassPressureDiff(MOS::AcRegClass, TryCand.SU, TryCand.AtTop),
          registerClassPressureDiff(MOS::AcRegClass, Cand.SU, Cand.AtTop),
          TryCand, Cand, PhysReg))
    return TryCand.Reason != NoCand;

  if (tryLess(
          registerClassPressureDiff(MOS::XYRegClass, TryCand.SU, TryCand.AtTop),
          registerClassPressureDiff(MOS::XYRegClass, Cand.SU, Cand.AtTop),
          TryCand, Cand, PhysReg))
    return TryCand.Reason != NoCand;

  if (tryLess(
          registerClassPressureDiff(MOS::Imag8RegClass, TryCand.SU,
                                    TryCand.AtTop),
          registerClassPressureDiff(MOS::Imag8RegClass, Cand.SU, Cand.AtTop),
          TryCand, Cand, PhysReg))
    return TryCand.Reason != NoCand;

  // Avoid exceeding the target's limit.
  if (DAG->isTrackingPressure() &&
      tryPressure(TryCand.RPDelta.Excess, Cand.RPDelta.Excess, TryCand, Cand,
                  RegExcess, TRI, DAG->MF))
    return TryCand.Reason != NoCand;

  // Avoid increasing the max critical pressure in the scheduled region.
  if (DAG->isTrackingPressure() &&
      tryPressure(TryCand.RPDelta.CriticalMax, Cand.RPDelta.CriticalMax,
                  TryCand, Cand, RegCritical, TRI, DAG->MF))
    return TryCand.Reason != NoCand;

  // Avoid increasing the max pressure of the entire region.
  if (DAG->isTrackingPressure() &&
      tryPressure(TryCand.RPDelta.CurrentMax, Cand.RPDelta.CurrentMax, TryCand,
                  Cand, RegMax, TRI, DAG->MF))
    return TryCand.Reason != NoCand;

  // We only compare a subset of features when comparing nodes between
  // Top and Bottom boundary. Some properties are simply incomparable, in many
  // other instances we should only override the other boundary if something
  // is a clear good pick on one boundary. Skip heuristics that are more
  // "tie-breaking" in nature.
  bool SameBoundary = Zone != nullptr;
  if (SameBoundary) {
    // Fall through to original instruction order.
    if ((Zone->isTop() && TryCand.SU->NodeNum < Cand.SU->NodeNum) ||
        (!Zone->isTop() && TryCand.SU->NodeNum > Cand.SU->NodeNum)) {
      TryCand.Reason = NodeOrder;
      return true;
    }
  }

  return false;
}

// Returns the change in pressure in a SU for a physical register.
int MOSSchedStrategy::registerClassPressureDiff(const TargetRegisterClass &RC,
                                                const SUnit *SU,
                                                bool IsTop) const {
  const MachineInstr *MI = SU->getInstr();

  int PressureDiff = 0;
  for (const MachineOperand &MO : MI->operands()) {
    if (!MO.isReg() || !MO.getReg().isPhysical() || !RC.contains(MO.getReg()))
      continue;
    if (MO.isDef()) {
      PressureDiff += IsTop ? 1 : -1;
    } else {
      PressureDiff += IsTop ? -1 : 1;
    }
  }
  return PressureDiff;
}
