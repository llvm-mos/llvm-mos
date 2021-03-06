#ifndef LLVM_LIB_TARGET_MOS_MOS_MACHINESCHEDULER_H
#define LLVM_LIB_TARGET_MOS_MOS_MACHINESCHEDULER_H

#include "llvm/CodeGen/MachineScheduler.h"

namespace llvm {

class MOSSchedStrategy : public GenericScheduler {
public:
  MOSSchedStrategy(const MachineSchedContext *C);

  void tryCandidate(SchedCandidate &Cand, SchedCandidate &TryCand,
                    SchedBoundary *Zone) const override;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOS_MACHINESCHEDULER_H
