//===-- MOSShiftRotateChain.h - MOS Shift/Rotate Chaining -------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS Shift/Rotate chaining pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSSHIFTROTATECHAIN_H
#define LLVM_LIB_TARGET_MOS_MOSSHIFTROTATECHAIN_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMOSShiftRotateChainPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSSHIFTROTATECHAIN_H
