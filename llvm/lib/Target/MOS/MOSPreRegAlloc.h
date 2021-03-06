#ifndef LLVM_LIB_TARGET_MOS_MOSPREREGALLOC_H
#define LLVM_LIB_TARGET_MOS_MOSPREREGALLOC_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMOSPreRegAlloc();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSPREREGALLOC_H
