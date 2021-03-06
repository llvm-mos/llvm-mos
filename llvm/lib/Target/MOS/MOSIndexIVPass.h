#ifndef LLVM_LIB_TARGET_MOS_MOSINDEXIV_H
#define LLVM_LIB_TARGET_MOS_MOSINDEXIV_H

#include "llvm/Analysis/LoopPass.h"

namespace llvm {

LoopPass *createMOSIndexIVPass();

} // end namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSINDEXIV_H
