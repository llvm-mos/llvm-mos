//===-- MOSMCExpr.cpp - MOS specific MC expression classes ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MOSMCExpr.h"

#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCValue.h"

namespace llvm {

namespace {

struct ModifierEntry {
  const char *const Spelling;
  MOSMCExpr::VariantKind VariantKind;
};

const ModifierEntry ModifierNames[] = {{"none", MOSMCExpr::VK_MOS_None}};

} // end of anonymous namespace

const MOSMCExpr *MOSMCExpr::create(VariantKind Kind, const MCExpr *Expr,
                                   bool Negated, MCContext &Ctx) {
  return new (Ctx) MOSMCExpr(Kind, Expr, Negated);
}

} // end of namespace llvm
