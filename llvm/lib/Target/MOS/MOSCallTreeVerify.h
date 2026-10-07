//===-- MOSCallTreeVerify.h - MOS Call Tree Verification -------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS call tree verification pass, which checks after
// legalization that everything reachable from a suffixed interrupt root runs
// in that root's private imaginary-register set.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSCALLTREEVERIFY_H
#define LLVM_LIB_TARGET_MOS_MOSCALLTREEVERIFY_H

namespace llvm {

class ModulePass;

ModulePass *createMOSCallTreeVerifyPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSCALLTREEVERIFY_H
