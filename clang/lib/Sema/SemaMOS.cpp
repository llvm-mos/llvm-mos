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
#include "clang/Sema/Sema.h"

using namespace llvm;

namespace clang {

namespace {
// A suffix becomes part of an assembler symbol name (__rcN<sfx>), so it must be
// a valid identifier: [A-Za-z_][A-Za-z0-9_]*. The empty string is rejected
// here; the unsuffixed form is represented by no argument at all.
bool isValidRCSuffix(StringRef S) {
  if (S.empty())
    return false;
  auto isIdentStart = [](char C) {
    return C == '_' || (C >= 'a' && C <= 'z') || (C >= 'A' && C <= 'Z');
  };
  auto isIdentCont = [&isIdentStart](char C) {
    return isIdentStart(C) || (C >= '0' && C <= '9');
  };
  if (!isIdentStart(S[0]))
    return false;
  for (char C : S.drop_front())
    if (!isIdentCont(C))
      return false;
  return true;
}
} // namespace

// Parses the optional suffix argument of an interrupt attribute. Returns false
// (after diagnosing) if the argument is malformed.
static bool parseRCSuffix(Sema &S, const ParsedAttr &AL, StringRef &Suffix,
                          SourceLocation &ArgLoc) {
  if (AL.getNumArgs() == 0)
    return true;
  if (!S.checkStringLiteralArgumentAttr(AL, 0, Suffix, &ArgLoc))
    return false;
  if (!isValidRCSuffix(Suffix)) {
    S.Diag(ArgLoc, diag::err_mos_interrupt_invalid_suffix) << AL << Suffix;
    return false;
  }
  return true;
}

void SemaMOS::handleInterruptAttr(Decl *D, const ParsedAttr &AL) {
  if (!isFuncOrMethodForAttrSubject(D)) {
    Diag(D->getLocation(), diag::warn_attribute_wrong_decl_type)
        << "'interrupt'" << ExpectedFunction;
    return;
  }

  if (!AL.checkAtMostNumArgs(SemaRef, 1))
    return;

  // A private register set is only supported for norecurse handlers: a
  // reentrant handler would need its private soft stack initialized before
  // the first interrupt, and could clobber its own registers when re-entered.
  StringRef Suffix;
  SourceLocation ArgLoc;
  if (!parseRCSuffix(SemaRef, AL, Suffix, ArgLoc))
    return;
  if (!Suffix.empty()) {
    Diag(ArgLoc, diag::err_mos_interrupt_suffix_requires_norecurse)
        << AL << Suffix;
    return;
  }

  D->addAttr(::new (getASTContext())
                 MOSInterruptAttr(getASTContext(), AL, Suffix));
}

void SemaMOS::handleInterruptNorecurseAttr(Decl *D, const ParsedAttr &AL) {
  if (!isFuncOrMethodForAttrSubject(D)) {
    Diag(D->getLocation(), diag::warn_attribute_wrong_decl_type)
        << "'interrupt_norecurse'" << ExpectedFunction;
    return;
  }

  if (!AL.checkAtMostNumArgs(SemaRef, 1))
    return;

  StringRef Suffix;
  SourceLocation ArgLoc;
  if (!parseRCSuffix(SemaRef, AL, Suffix, ArgLoc))
    return;

  D->addAttr(::new (getASTContext())
                 MOSInterruptNorecurseAttr(getASTContext(), AL, Suffix));
}

MOSInterruptNorecurseAttr *
SemaMOS::mergeInterruptNorecurseAttr(Decl *D,
                                     const MOSInterruptNorecurseAttr &AL) {
  if (const auto *Existing = D->getAttr<MOSInterruptNorecurseAttr>()) {
    // The suffix selects the register set, so all redeclarations must agree.
    if (Existing->getSuffix() != AL.getSuffix()) {
      Diag(Existing->getLocation(), diag::err_mos_interrupt_suffix_mismatch)
          << Existing << Existing->getSuffix() << AL.getSuffix();
      Diag(AL.getLocation(), diag::note_previous_attribute);
    }
    return nullptr;
  }
  return ::new (getASTContext())
      MOSInterruptNorecurseAttr(getASTContext(), AL, AL.getSuffix());
}

void SemaMOS::handleInterruptSafeAttr(Decl *D, const ParsedAttr &AL) {
  if (!isFuncOrMethodForAttrSubject(D)) {
    Diag(D->getLocation(), diag::warn_attribute_wrong_decl_type)
        << "'interrupt_safe'" << ExpectedFunction;
    return;
  }

  if (!AL.checkExactlyNumArgs(SemaRef, 0))
    return;

  // Definitions are diagnosed in ActOnStartOfFunctionDef, since whether this
  // declaration has a body isn't known yet.
  handleSimpleAttribute<MOSInterruptSafeAttr>(*this, D, AL);
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
