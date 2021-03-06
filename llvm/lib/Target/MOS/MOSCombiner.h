#ifndef LLVM_LIB_TARGET_MOS_MOSCOMBINER_H
#define LLVM_LIB_TARGET_MOS_MOSCOMBINER_H

#include "llvm/Pass.h"

namespace llvm {

FunctionPass *createMOSCombiner();

} // end namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSCOMBINER_H
