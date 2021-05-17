//===--- MOS.h - Declare MOS target feature support -------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares MOS TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_LIB_BASIC_TARGETS_MOS_H
#define LLVM_CLANG_LIB_BASIC_TARGETS_MOS_H

#include "clang/Basic/TargetInfo.h"
#include "clang/Basic/TargetOptions.h"
#include "llvm/ADT/Triple.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/Support/Compiler.h"

namespace clang {
namespace targets {

class MOSTargetInfo : public TargetInfo {
public:
  MOSTargetInfo(const llvm::Triple &Triple, const TargetOptions &);

  void getTargetDefines(const LangOptions &Opts,
                        MacroBuilder &Builder) const override {}

  ArrayRef<Builtin::Info> getTargetBuiltins() const override { return None; }

  BuiltinVaListKind getBuiltinVaListKind() const override {
    return TargetInfo::VoidPtrBuiltinVaList;
  }

  bool validateAsmConstraint(const char *&Name,
                             TargetInfo::ConstraintInfo &Info) const override;

  const char *getClobbers() const override { return ""; }

  ArrayRef<const char *> getGCCRegNames() const override;
  ArrayRef<TargetInfo::GCCRegAlias> getGCCRegAliases() const override {
    return None;
  }
  unsigned getRegisterWidth() const override { return 8; }
};

} // namespace targets
} // namespace clang

#endif // LLVM_CLANG_LIB_BASIC_TARGETS_MOS_H
