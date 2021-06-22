//===-- MOSPostRAScavenging.h - MOS Post RA Register Scavenging -*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS post-register-allocation register scavenging pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSPOSTRASCAVENGING_H
#define LLVM_LIB_TARGET_MOS_MOSPOSTRASCAVENGING_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMOSPostRAScavengingPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSPOSTRASCAVENGING_H
