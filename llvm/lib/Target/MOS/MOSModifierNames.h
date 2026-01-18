//===-- MOSModifierNames.h - MOS modifier name table ------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSMODIFIERNAMES_H
#define LLVM_LIB_TARGET_MOS_MOSMODIFIERNAMES_H

#include "MCTargetDesc/MOSMCExpr.h"
#include "llvm/ADT/ArrayRef.h"

namespace llvm {
namespace MOS {

struct ModifierEntry {
  const char *const Spelling;
  MOSMCExpr::VariantKind VariantKind;
  bool ImmediateOnly = false;
};

llvm::ArrayRef<ModifierEntry> modifierNames();

} // namespace MOS
} // namespace llvm

#endif // LLVM_LIB_TARGET_MOS_MOSMODIFIERNAMES_H
