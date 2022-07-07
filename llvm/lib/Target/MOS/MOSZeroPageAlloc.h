//===-- MOSZeroPageAlloc.h - MOS Zero Page Allocation -----------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file This file declares the MOS zero page allocation pass.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSZEROPAGEALLOC_H
#define LLVM_LIB_TARGET_MOS_MOSZEROPAGEALLOC_H

#include "llvm/Pass.h"

namespace llvm {

ModulePass *createMOSZeroPageAllocPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSZEROPAGEALLOC_H
