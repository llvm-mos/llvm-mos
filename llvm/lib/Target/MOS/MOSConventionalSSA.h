//===----------------------------------------------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSCONVENTIONALSSA_H
#define LLVM_LIB_TARGET_MOS_MOSCONVENTIONALSSA_H

namespace llvm {

class MachineFunctionPass;

MachineFunctionPass *createMOSConventionalSSAPass();

} // namespace llvm

#endif // LLVM_LIB_TARGET_MOS_MOSCONVENTIONALSSA_H
