//===-- MOSCallTreeLibcalls.h - MOS Call Tree Libcalls ---------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS call tree libcall redirect pass, a
// MachineFunctionPass that rewrites the external-symbol operands emitted by the
// GlobalISel legalizer for runtime libcalls in suffixed-ISR functions so that
// they refer to the per-suffix clones produced by MOSCallTreeClone.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSCALLTREELIBCALLS_H
#define LLVM_LIB_TARGET_MOS_MOSCALLTREELIBCALLS_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMOSCallTreeLibcallsPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSCALLTREELIBCALLS_H
