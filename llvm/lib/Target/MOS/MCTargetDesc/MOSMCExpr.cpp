//===-- MOSMCExpr.cpp - MOS specific MC expression classes ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSMCExpr.h"
#include "MOSFixupKinds.h"

#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCValue.h"

namespace llvm {

namespace {

const struct ModifierEntry {
  const char * const Spelling;
  MOSMCExpr::VariantKind VariantKind;
} ModifierNames[] = {
    {"mos16lo", MOSMCExpr::VK_MOS_ADDR16_LO},
    {"mos16hi", MOSMCExpr::VK_MOS_ADDR16_HI},
    {"mos24segment", MOSMCExpr::VK_MOS_ADDR24_SEGMENT},
    {"mos24bank", MOSMCExpr::VK_MOS_ADDR24_BANK},
    {"mos24banklo", MOSMCExpr::VK_MOS_ADDR24_BANK_LO},
    {"mos24bankhi", MOSMCExpr::VK_MOS_ADDR24_BANK_HI},
};

} // end of anonymous namespace

const MOSMCExpr *MOSMCExpr::create(VariantKind Kind, const MCExpr *Expr,
                                   bool Negated, MCContext &Ctx) {
  return new (Ctx) MOSMCExpr(Kind, Expr, Negated);
}

void MOSMCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  assert(Kind != VK_MOS_NONE);

  if (isNegated())
    OS << '-';

  OS << getName() << '(';
  getSubExpr()->print(OS, MAI);
  OS << ')';
}

bool MOSMCExpr::evaluateAsConstant(int64_t &Result) const {
  MCValue Value;

  bool isRelocatable =
      getSubExpr()->evaluateAsRelocatable(Value, nullptr, nullptr);

  if (!isRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = evaluateAsInt64(Value.getConstant());
    return true;
  }

  return false;
}

bool MOSMCExpr::evaluateAsRelocatableImpl(MCValue &Result,
                                          const MCAsmLayout *Layout,
                                          const MCFixup *Fixup) const {
  MCValue Value;
  bool isRelocatable = SubExpr->evaluateAsRelocatable(Value, Layout, Fixup);

  if (!isRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = MCValue::get(evaluateAsInt64(Value.getConstant()));
  } else {
    if (!Layout) return false;

    MCContext &Context = Layout->getAssembler().getContext();
    const MCSymbolRefExpr *Sym = Value.getSymA();
    MCSymbolRefExpr::VariantKind Modifier = Sym->getKind();
    if (Modifier != MCSymbolRefExpr::VK_None)
      return false;

    Sym = MCSymbolRefExpr::create(&Sym->getSymbol(), Modifier, Context);
    Result = MCValue::get(Sym, Value.getSymB(), Value.getConstant());
  }

  return true;
}

int64_t MOSMCExpr::evaluateAsInt64(int64_t Value) const {
  if (Negated)
    Value *= -1;

  switch (Kind) {
  case MOSMCExpr::VK_MOS_ADDR16_LO:
  case MOSMCExpr::VK_MOS_ADDR24_BANK_LO:
    Value &= 0xff;
    break;
  case MOSMCExpr::VK_MOS_ADDR16_HI:
  case MOSMCExpr::VK_MOS_ADDR24_BANK_HI:
    Value &= 0xff00;
    Value >>= 8;
    break;
  case MOSMCExpr::VK_MOS_ADDR24_SEGMENT:
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case MOSMCExpr::VK_MOS_ADDR24_BANK:
    Value &= 0xffff;
    break;

  case MOSMCExpr::VK_MOS_NONE:
    llvm_unreachable("Uninitialized expression.");
  }
  return static_cast<uint64_t>(Value);
}

MOS::Fixups MOSMCExpr::getFixupKind() const {
  MOS::Fixups Kind = MOS::Fixups::LastTargetFixupKind;

  switch (getKind()) {
  case VK_MOS_ADDR16_HI:
    Kind = MOS::Addr16_High;
    break;
  case VK_MOS_ADDR16_LO:
    Kind = MOS::Addr16_Low;
    break;
  case VK_MOS_ADDR24_BANK:
    Kind = MOS::Addr24_Bank;
    break;
  case VK_MOS_ADDR24_SEGMENT:
    Kind = MOS::Addr24_Segment;
    break;
  case VK_MOS_ADDR24_BANK_HI:
    Kind = MOS::Addr24_Bank_High;
    break;
  case VK_MOS_ADDR24_BANK_LO:
    Kind = MOS::Addr24_Bank_Low;
    break;
  case VK_MOS_NONE:
    llvm_unreachable("Uninitialized expression");
  }

  return Kind;
}

void MOSMCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}

const char *MOSMCExpr::getName() const {
  const auto &Modifier = std::find_if(
      std::begin(ModifierNames), std::end(ModifierNames),
      [this](ModifierEntry const &Mod) { return Mod.VariantKind == Kind; });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->Spelling;
  }
  return nullptr;
}

MOSMCExpr::VariantKind MOSMCExpr::getKindByName(StringRef Name) {
  const auto &Modifier = std::find_if(
      std::begin(ModifierNames), std::end(ModifierNames),
      [&Name](ModifierEntry const &Mod) { return Mod.Spelling == Name; });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->VariantKind;
  }
  return VK_MOS_NONE;
}

} // end of namespace llvm

