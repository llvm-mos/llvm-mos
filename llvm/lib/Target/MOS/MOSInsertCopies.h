//===-- MOSLateOptimization.h - MOS Copy Insertion --------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS copy insertion pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSINSERTCOPIES_H
#define LLVM_LIB_TARGET_MOS_MOSINSERTCOPIES_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMOSInsertCopiesPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSINSERTCOPIES_H
