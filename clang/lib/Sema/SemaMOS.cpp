//===------ SemaMOS.cpp ------------ MOS target-specific routines ---------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//  This file implements semantic analysis functions specific to MOS.
//
//===----------------------------------------------------------------------===//

#include "clang/Sema/SemaMOS.h"

#include "clang/AST/ASTContext.h"
#include "clang/Sema/Attr.h"

using namespace llvm;

namespace clang {

void SemaMOS::handleInterruptAttr(Decl *D, const ParsedAttr &AL) {
  if (!isFuncOrMethodForAttrSubject(D)) {
    Diag(D->getLocation(), diag::warn_attribute_wrong_decl_type)
        << "'interrupt'" << ExpectedFunction;
    return;
  }

  if (!AL.checkExactlyNumArgs(SemaRef, 0))
    return;

  handleSimpleAttribute<MOSInterruptAttr>(*this, D, AL);
}

void SemaMOS::handleInterruptNorecurseAttr(Decl *D, const ParsedAttr &AL) {
  if (!isFuncOrMethodForAttrSubject(D)) {
    Diag(D->getLocation(), diag::warn_attribute_wrong_decl_type)
        << "'interrupt_norecurse'" << ExpectedFunction;
    return;
  }

  if (!AL.checkExactlyNumArgs(SemaRef, 0))
    return;

  handleSimpleAttribute<MOSInterruptNorecurseAttr>(*this, D, AL);
}

void SemaMOS::handleInterruptNoISRAttr(Decl *D, const ParsedAttr &AL) {
  if (!isFuncOrMethodForAttrSubject(D)) {
    Diag(D->getLocation(), diag::warn_attribute_wrong_decl_type)
        << "'no_isr'" << ExpectedFunction;
    return;
  }

  if (!AL.checkExactlyNumArgs(SemaRef, 0))
    return;

  handleSimpleAttribute<MOSNoISRAttr>(*this, D, AL);
}

SemaMOS::SemaMOS(Sema &S) : SemaBase(S) {}

} // namespace clang
