//===-- MOS.h - Top-level interface for MOS representation ------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// MOS back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOS_H
#define LLVM_LIB_TARGET_MOS_MOS_H

#include "llvm/Pass.h"

namespace llvm {

void initializeMOSCombinerPass(PassRegistry &);
void initializeMOSIndexIVPass(PassRegistry &);
void initializeMOSLowerSelectPass(PassRegistry &);
void initializeMOSNoRecursePass(PassRegistry &);
void initializeMOSStaticStackAllocPass(PassRegistry &);

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOS_H
