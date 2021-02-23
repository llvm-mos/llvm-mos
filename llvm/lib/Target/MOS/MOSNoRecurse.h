//===-- MOSNoRecurse.h - MOS NoRecurse Pass ---------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS NoRecurse pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSNORECURSE_H
#define LLVM_LIB_TARGET_MOS_MOSNORECURSE_H

#include "llvm/Analysis/CallGraphSCCPass.h"

namespace llvm {

CallGraphSCCPass *createMOSNoRecursePass();

} // end namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSNORECURSE_H
