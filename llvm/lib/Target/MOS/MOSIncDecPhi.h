//===-- MOSIncDecPhi.h - MOS Increment Decrement PHI ------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS pass to separate an increment/decrement from an
// ADC of a PHI of -1 or 1.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSINCDECPHI_H
#define LLVM_LIB_TARGET_MOS_MOSINCDECPHI_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMOSIncDecPhiPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSINCDECPHI_H
