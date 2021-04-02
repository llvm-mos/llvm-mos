//===-- MOSStaticStackAlloc.h - MOS Static Stack Allocation -----*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS static stack allocation pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSSTATICSTACKALLOC_H
#define LLVM_LIB_TARGET_MOS_MOSSTATICSTACKALLOC_H

#include "llvm/Pass.h"

namespace llvm {

ModulePass *createMOSStaticStackAllocPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSSTATICSTACKALLOC_H
