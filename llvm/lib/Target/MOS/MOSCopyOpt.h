//===-- MOSCopyOpt.h - MOS Copy Optimization --------------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS pass to optimize COPY operations before lowering.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSCOPYOPT_H
#define LLVM_LIB_TARGET_MOS_MOSCOPYOPT_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMOSCopyOptPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSCOPYOPT_H
