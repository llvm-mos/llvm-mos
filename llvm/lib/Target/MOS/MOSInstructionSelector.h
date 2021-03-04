#ifndef LLVM_LIB_TARGET_MOS_MOSINSTRUCTIONSELECTOR_H
#define LLVM_LIB_TARGET_MOS_MOSINSTRUCTIONSELECTOR_H

#include "MOSTargetMachine.h"
#include "MOSRegisterBankInfo.h"
#include "MOSSubtarget.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"

namespace llvm {

InstructionSelector *
createMOSInstructionSelector(const MOSTargetMachine &TM,
                                 MOSSubtarget &STI,
                                 MOSRegisterBankInfo &RBI);

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSINSTRUCTIONSELECTOR_H
