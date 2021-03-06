#ifndef LLVM_LIB_TARGET_MOS_MOSSTATICSTACKALLOC_H
#define LLVM_LIB_TARGET_MOS_MOSSTATICSTACKALLOC_H

#include "llvm/Pass.h"

namespace llvm {

ModulePass *createMOSStaticStackAllocPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSSTATICSTACKALLOC_H
