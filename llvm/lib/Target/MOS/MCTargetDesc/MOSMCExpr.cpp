//===-- MOSMCExpr.cpp - MOS specific MC expression classes ----------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSMCExpr.h"
#include "MOSFixupKinds.h"

#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCValue.h"

namespace llvm {

namespace {

const struct ModifierEntry {
  const char *const Spelling;
  MOSMCExpr::VariantKind VariantKind;
  bool ImmediateOnly = false;
} ModifierNames[] = {
    // Define immediate variants of mos8() and mos16() first.
    {"mos8", MOSMCExpr::VK_MOS_IMM8, true},
    {"mos16", MOSMCExpr::VK_MOS_IMM16, true},
    {"mos8", MOSMCExpr::VK_MOS_ADDR8},
    {"mos16", MOSMCExpr::VK_MOS_ADDR16},
    {"mos16lo", MOSMCExpr::VK_MOS_ADDR16_LO},
    {"mos16hi", MOSMCExpr::VK_MOS_ADDR16_HI},
    {"mos24", MOSMCExpr::VK_MOS_ADDR24},
    {"mos24bank", MOSMCExpr::VK_MOS_ADDR24_BANK},
    {"mos24segment", MOSMCExpr::VK_MOS_ADDR24_SEGMENT},
    {"mos24segmentlo", MOSMCExpr::VK_MOS_ADDR24_SEGMENT_LO},
    {"mos24segmenthi", MOSMCExpr::VK_MOS_ADDR24_SEGMENT_HI},
    {"mos13", MOSMCExpr::VK_MOS_ADDR13},
};

} // end of anonymous namespace

const MOSMCExpr *MOSMCExpr::create(VariantKind Kind, const MCExpr *Expr,
                                   bool Negated, MCContext &Ctx) {
  return new (Ctx) MOSMCExpr(Kind, Expr, Negated);
}

void MOSMCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  assert(Kind != VK_MOS_NONE);

  if (isNegated()) {
    OS << '-';
  }

  OS << getName() << '(';
  getSubExpr()->print(OS, MAI);
  OS << ')';
}

bool MOSMCExpr::evaluateAsConstant(int64_t &Result) const {
  MCValue Value;

  bool IsRelocatable =
      getSubExpr()->evaluateAsRelocatable(Value, nullptr, nullptr);

  if (!IsRelocatable) {
    return false;
  }

  if (Value.isAbsolute()) {
    Result = evaluateAsInt64(Value.getConstant());
    return true;
  }

  return false;
}

bool MOSMCExpr::evaluateAsRelocatableImpl(MCValue &Result,
                                          const MCAssembler *Asm,
                                          const MCFixup *Fixup) const {
  MCValue Value;
  bool IsRelocatable = SubExpr->evaluateAsRelocatable(Value, Asm, Fixup);

  if (!IsRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = MCValue::get(evaluateAsInt64(Value.getConstant()));
  } else {
    if (!Asm)
      return false;

    MCContext &Context = Asm->getContext();
    const MCSymbolRefExpr *Sym = Value.getSymA();
    MCSymbolRefExpr::VariantKind Modifier = Sym->getKind();
    if (Modifier != MCSymbolRefExpr::VK_None) {
      return false;
    }

    Sym = MCSymbolRefExpr::create(&Sym->getSymbol(), Modifier, Context);
    Result = MCValue::get(Sym, Value.getSymB(), Value.getConstant());
  }

  return true;
}

int64_t MOSMCExpr::evaluateAsInt64(int64_t Value) const {
  if (Negated) {
    Value *= -1;
  }

  switch (Kind) {
  case MOSMCExpr::VK_MOS_IMM8:
  case MOSMCExpr::VK_MOS_ADDR8:
  case MOSMCExpr::VK_MOS_ADDR16_LO:
  case MOSMCExpr::VK_MOS_ADDR24_SEGMENT_LO:
    Value &= 0xff;
    break;
  case MOSMCExpr::VK_MOS_ADDR16_HI:
  case MOSMCExpr::VK_MOS_ADDR24_SEGMENT_HI:
    Value &= 0xff00;
    Value >>= 8;
    break;
  case MOSMCExpr::VK_MOS_ADDR24_BANK:
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case MOSMCExpr::VK_MOS_IMM16:
  case MOSMCExpr::VK_MOS_ADDR16:
  case MOSMCExpr::VK_MOS_ADDR24_SEGMENT:
    Value &= 0xffff;
    break;
  case MOSMCExpr::VK_MOS_ADDR24:
    Value &= 0xffffff;
    break;
  case MOSMCExpr::VK_MOS_ADDR13:
    Value &= 0x1fff;
    break;

  case MOSMCExpr::VK_MOS_ADDR_ASCIZ:
    llvm_unreachable("Unable to evaluate VK_MOS_ADDR_ASCIZ as int64.");

  case MOSMCExpr::VK_MOS_NONE:
    llvm_unreachable("Uninitialized expression.");
  }
  return static_cast<uint64_t>(Value);
}

MOS::Fixups MOSMCExpr::getFixupKind() const {
  MOS::Fixups Kind = MOS::Fixups::LastTargetFixupKind;

  switch (getKind()) {
  case VK_MOS_IMM8:
    Kind = MOS::Imm8;
    break;
  case VK_MOS_IMM16:
    Kind = MOS::Imm16;
    break;
  case VK_MOS_ADDR8:
    Kind = MOS::Addr8;
    break;
  case VK_MOS_ADDR16:
    Kind = MOS::Addr16;
    break;
  case VK_MOS_ADDR16_HI:
    Kind = MOS::Addr16_High;
    break;
  case VK_MOS_ADDR16_LO:
    Kind = MOS::Addr16_Low;
    break;
  case VK_MOS_ADDR24:
    Kind = MOS::Addr24;
    break;
  case VK_MOS_ADDR24_BANK:
    Kind = MOS::Addr24_Bank;
    break;
  case VK_MOS_ADDR24_SEGMENT:
    Kind = MOS::Addr24_Segment;
    break;
  case VK_MOS_ADDR24_SEGMENT_HI:
    Kind = MOS::Addr24_Segment_High;
    break;
  case VK_MOS_ADDR24_SEGMENT_LO:
    Kind = MOS::Addr24_Segment_Low;
    break;
  case VK_MOS_ADDR13:
    Kind = MOS::Addr13;
    break;
  case VK_MOS_ADDR_ASCIZ:
    Kind = MOS::AddrAsciz;
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

MOSMCExpr::VariantKind MOSMCExpr::getKindByName(StringRef Name,
                                                bool IsImmediate) {
  const auto &Modifier =
      std::find_if(std::begin(ModifierNames), std::end(ModifierNames),
                   [&Name, IsImmediate](ModifierEntry const &Mod) {
                     if (Mod.ImmediateOnly && !IsImmediate)
                       return false;
                     return Mod.Spelling == Name;
                   });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->VariantKind;
  }
  return VK_MOS_NONE;
}

} // end of namespace llvm
