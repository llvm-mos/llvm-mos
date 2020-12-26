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

const struct ModifierEntry {
  const char * const Spelling;
  MOSMCExpr::VariantKind VariantKind;
} ModifierNames[] = {
    {"lo8", MOSMCExpr::VK_MOS_LO8},       {"hi8", MOSMCExpr::VK_MOS_HI8},
    {"hh8", MOSMCExpr::VK_MOS_HH8}, // synonym with hlo8
    {"hlo8", MOSMCExpr::VK_MOS_HH8},      {"hhi8", MOSMCExpr::VK_MOS_HHI8},

    {"pm_lo8", MOSMCExpr::VK_MOS_PM_LO8}, {"pm_hi8", MOSMCExpr::VK_MOS_PM_HI8},
    {"pm_hh8", MOSMCExpr::VK_MOS_PM_HH8},

    {"lo8_gs", MOSMCExpr::VK_MOS_LO8_GS}, {"hi8_gs", MOSMCExpr::VK_MOS_HI8_GS},
    {"gs", MOSMCExpr::VK_MOS_GS},
};

} // end of anonymous namespace

const MOSMCExpr *MOSMCExpr::create(VariantKind Kind, const MCExpr *Expr,
                                   bool Negated, MCContext &Ctx) {
  return new (Ctx) MOSMCExpr(Kind, Expr, Negated);
}

void MOSMCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  assert(Kind != VK_MOS_None);

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
  case MOSMCExpr::VK_MOS_LO8:
    Value &= 0xff;
    break;
  case MOSMCExpr::VK_MOS_HI8:
    Value &= 0xff00;
    Value >>= 8;
    break;
  case MOSMCExpr::VK_MOS_HH8:
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case MOSMCExpr::VK_MOS_HHI8:
    Value &= 0xff000000;
    Value >>= 24;
    break;
  case MOSMCExpr::VK_MOS_PM_LO8:
  case MOSMCExpr::VK_MOS_LO8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff;
    break;
  case MOSMCExpr::VK_MOS_PM_HI8:
  case MOSMCExpr::VK_MOS_HI8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff00;
    Value >>= 8;
    break;
  case MOSMCExpr::VK_MOS_PM_HH8:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case MOSMCExpr::VK_MOS_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    break;

  case MOSMCExpr::VK_MOS_None:
    llvm_unreachable("Uninitialized expression.");
  }
  return static_cast<uint64_t>(Value) & 0xff;
}

MOS::Fixups MOSMCExpr::getFixupKind() const {
  MOS::Fixups Kind = MOS::Fixups::LastTargetFixupKind;

  switch (getKind()) {
  case VK_MOS_LO8:
    Kind = isNegated() ? MOS::fixup_lo8_ldi_neg : MOS::fixup_lo8_ldi;
    break;
  case VK_MOS_HI8:
    Kind = isNegated() ? MOS::fixup_hi8_ldi_neg : MOS::fixup_hi8_ldi;
    break;
  case VK_MOS_HH8:
    Kind = isNegated() ? MOS::fixup_hh8_ldi_neg : MOS::fixup_hh8_ldi;
    break;
  case VK_MOS_HHI8:
    Kind = isNegated() ? MOS::fixup_ms8_ldi_neg : MOS::fixup_ms8_ldi;
    break;

  case VK_MOS_PM_LO8:
    Kind = isNegated() ? MOS::fixup_lo8_ldi_pm_neg : MOS::fixup_lo8_ldi_pm;
    break;
  case VK_MOS_PM_HI8:
    Kind = isNegated() ? MOS::fixup_hi8_ldi_pm_neg : MOS::fixup_hi8_ldi_pm;
    break;
  case VK_MOS_PM_HH8:
    Kind = isNegated() ? MOS::fixup_hh8_ldi_pm_neg : MOS::fixup_hh8_ldi_pm;
    break;
  case VK_MOS_GS:
    Kind = MOS::fixup_16_pm;
    break;
  case VK_MOS_LO8_GS:
    Kind = MOS::fixup_lo8_ldi_gs;
    break;
  case VK_MOS_HI8_GS:
    Kind = MOS::fixup_hi8_ldi_gs;
    break;

  case VK_MOS_None:
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
  return VK_MOS_None;
}

} // end of namespace llvm

