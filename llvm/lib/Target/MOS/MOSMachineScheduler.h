//===-- MOSMachineScheduler.h - MOS Instruction Scheduler -------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS machine instruction scheduler.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOS_MACHINESCHEDULER_H
#define LLVM_LIB_TARGET_MOS_MOS_MACHINESCHEDULER_H

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineScheduler.h"

namespace llvm {

class MOSSchedStrategy : public GenericScheduler {
  // Track the defining node rather than the virtual register: two-address
  // carry chains may redefine the same register at each arithmetic step.
  struct CarryValue {
    const SUnit *Def;
    SmallVector<const SUnit *, 4> Users;
  };
  SmallVector<CarryValue, 16> Carries;
  DenseMap<const SUnit *, SmallVector<unsigned, 2>> CarryRefs;
  SmallPtrSet<const SUnit *, 32> TopNodes, BottomNodes;
  int TopCarries = 0, BottomCarries = 0;

  bool isCarryLive(const CarryValue &Value, bool AtTop,
                   const SUnit *Trial = nullptr) const;
  int carryPressureDiff(const SUnit *SU, bool AtTop) const;
  int carryExcessDiff(const SUnit *SU, bool AtTop) const;

public:
  MOSSchedStrategy(const MachineSchedContext *C);

  void initialize(ScheduleDAGMI *DAG) override;
  void schedNode(SUnit *SU, bool IsTopNode) override;

  bool tryCandidate(SchedCandidate &Cand, SchedCandidate &TryCand,
                    SchedBoundary *Zone) const override;

  int registerClassPressureDiff(const TargetRegisterClass &RC, const SUnit *SU,
                                bool IsTop) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOS_MACHINESCHEDULER_H
