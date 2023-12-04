//===-- MOSInternalize.h - MOS Libcall Internalization ----------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.n with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MOS library call internalization pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSINTERNALIZE_H
#define LLVM_LIB_TARGET_MOS_MOSINTERNALIZE_H

namespace llvm {

class ModulePass;

ModulePass *createMOSInternalizePass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSINTERNALIZE_H
