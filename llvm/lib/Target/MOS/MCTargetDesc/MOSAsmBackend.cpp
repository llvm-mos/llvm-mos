//===-- MOSAsmBackend.cpp - MOS Asm Backend  ------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the MOSAsmBackend class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MOSAsmBackend.h"
#include "MCTargetDesc/MOSELFObjectWriter.h"
#include "MCTargetDesc/MOSFixupKinds.h"
#include "MCTargetDesc/MOSMCExpr.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOSSubtarget.h"

#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"

#include <climits>
#include <cstdint>
#include <memory>
#include <string>

namespace llvm {
namespace MOS {
struct InstructionRelaxationEntry {
  unsigned From;
  unsigned To;
};

#define GET_ZeroPageInstructionRelaxation_DECL
#define GET_ZeroPageInstructionRelaxation_IMPL
#define GET_ZeroBankInstructionRelaxation_DECL
#define GET_ZeroBankInstructionRelaxation_IMPL
#define GET_BranchInstructionRelaxation_DECL
#define GET_BranchInstructionRelaxation_IMPL

struct MOSSPC700Entry {
  unsigned From;
  unsigned To;
};

#define GET_MOSSPC700Table_DECL
#define GET_MOSSPC700Table_IMPL

#include "MOSGenSearchableTables.inc"
} // namespace MOS

static cl::opt<bool> ForcePCRelReloc(
    "mos-force-pcrel-reloc",
    cl::desc("Force relocation entries to be emitted for PCREL fixups."),
    cl::init(false), cl::Hidden);

MCAsmBackend *createMOSAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO) {
  return new MOSAsmBackend(STI.getTargetTriple().getOS());
}

bool MOSAsmBackend::fixupNeedsRelaxation(const MCFixup &Fixup,
                                         uint64_t Value) const {
  return true;
}

static int getRelativeMOSPCCorrection(const bool IsPCRel16,
                                      const bool Is65CE02 = false) {
  // LLVM measures PC-relative fixups from the fixup location (opcode_addr + 1).
  // 8-bit branches: offset from opcode_addr + 2 → correction -1.
  // 65CE02 16-bit branches: also offset from opcode_addr + 2 → correction -1.
  // 65816 BRL: offset from opcode_addr + 3 (past full instruction) → correction -2.
  if (IsPCRel16 && !Is65CE02)
    return -2;
  return -1;
}

static bool fitsIntoFixup(const int64_t SignedValue, const bool IsPCRel16) {
  return SignedValue >= (IsPCRel16 ? INT16_MIN : INT8_MIN) &&
         SignedValue <= (IsPCRel16 ? INT16_MAX : INT8_MAX);
}

// Derived from findAssociatedFragment.
bool isBasedOnZeroPageSymbol(const MCExpr *E) {
  switch (E->getKind()) {
  case MCExpr::Target:
    return isBasedOnZeroPageSymbol(
        static_cast<const MOSMCExpr *>(E)->getSubExpr());

  case MCExpr::Constant:
  case MCExpr::Specifier:
    return false;

  case MCExpr::SymbolRef:
    return static_cast<const MCSymbolELF &>(
               cast<MCSymbolRefExpr>(E)->getSymbol())
               .getOther() &
           ELF::STO_MOS_ZEROPAGE;

  case MCExpr::Unary:
    return cast<MCUnaryExpr>(E)->getSubExpr()->findAssociatedFragment();

  case MCExpr::Binary: {
    const MCBinaryExpr *BE = cast<MCBinaryExpr>(E);
    return isBasedOnZeroPageSymbol(BE->getLHS()) ||
           isBasedOnZeroPageSymbol(BE->getRHS());
  }
  }
  llvm_unreachable("Invalid assembly expression kind!");
}

bool MOSAsmBackend::fixupNeedsRelaxationAdvanced(const MCFragment &F,
                                                 const MCFixup &Fixup,
                                                 const MCValue &Target,
                                                 uint64_t Value,
                                                 bool Resolved) const {
  // On 65816, it is possible to zero-bank relax from Addr16 to Addr24. The
  // assembler relaxes in a loop until instructions cannot be relaxed further,
  // so this is able to follow zero-page relaxation.
  bool BankRelax = false;
  MOSAsmBackend::relaxInstructionTo(RelaxedOpcode, *RelaxedSTI, BankRelax);

  auto Info = getFixupKindInfo(Fixup.getKind());
  const auto *MME = dyn_cast<MOSMCExpr>(Fixup.getValue());
  // If this is a target-specific relaxation, e.g. a modifier, then the Info
  // field already knows the exact width of the answer, so decide now.
  if (MME != nullptr)
    return (Info.TargetSize > (BankRelax ? 16 : 8));

  // Now the fixup kind is not target-specific.  Yet, if it requires more than
  // 8 (or 16) bits, then relaxation is needed.
  if (Info.TargetSize > (BankRelax ? 16 : 8))
    return true;

  if (Fixup.isPCRel()) {
    const bool IsPCRel16 = Fixup.getKind() == (MCFixupKind)MOS::PCRel16;
    assert((IsPCRel16 || Fixup.getKind() == (MCFixupKind)MOS::PCRel8) &&
           "unexpected target fixup kind");
    const MCSubtargetInfo *STI = F.getSubtargetInfo();
    bool Is65CE02 = STI && STI->hasFeature(MOS::Feature65CE02);
    // This fixup concerns a relative branch.
    // If the fixup is unresolved, we can't know if relaxation is needed.
    return !Resolved ||
           !fitsIntoFixup(
               Value + getRelativeMOSPCCorrection(IsPCRel16, Is65CE02), false);
  }

  // See if the expression is derived from a zero page symbol.
  if (isBasedOnZeroPageSymbol(Fixup.getValue()))
    return false;

  // In order to resolve an eight to sixteen bit possible relaxation, we need
  // to figure out whether the symbol in question is in zero page or not.  If
  // it is in zero page, then we don't need to do anything.  If not, we need
  // to relax the instruction to 16 bits.
  // If we're not writing to ELF, punt on this whole idea, just do the
  // relaxation for safety's sake
  MCFragment *Frag = Fixup.getValue()->findAssociatedFragment();
  if (!Frag)
    return true;
  const auto *Sec = static_cast<const MCSectionELF *>(Frag->getParent());
  if (!Sec)
    return true;

  // If the section of the symbol is marked with special zero-page flag
  // then this is an 8 bit instruction and it doesn't need
  // relaxation.
  if (Sec->getFlags() & ELF::SHF_MOS_ZEROPAGE)
    return false;

  return !MOS::isZeroPageSectionName(Sec->getName());
}

MCFixupKindInfo MOSAsmBackend::getFixupKindInfo(MCFixupKind Kind) const {
  if (Kind < FirstTargetFixupKind) {
    return MCAsmBackend::getFixupKindInfo(Kind);
  }

  return MOSFixupKinds::getFixupKindInfo(static_cast<MOS::Fixups>(Kind), this);
}

bool MOSAsmBackend::shouldForceRelocation(const MCFixup &F, const MCValue &V) {
  return ForcePCRelReloc && F.isPCRel();
}

void MOSAsmBackend::applyFixup(const MCFragment &F, const MCFixup &Fixup,
                               const MCValue &Target, uint8_t *Data,
                               uint64_t Value, bool IsResolved) {
  if (IsResolved && shouldForceRelocation(Fixup, Target))
    IsResolved = false;
  if (!IsResolved)
    Asm->getWriter().recordRelocation(F, Fixup, Target, Value);

  unsigned int Kind = Fixup.getKind();

  switch (Kind) {
  case MOS::AddrAsciz: {
    std::string ValueStr = utostr(Value);
    std::copy(ValueStr.begin(), ValueStr.end(), Data);
    Data[ValueStr.size()] = '\0';
    return;
  }
  case MOS::PCRel8:
  case MOS::PCRel16: {
    const MCSubtargetInfo *STI = F.getSubtargetInfo();
    bool Is65CE02 = STI && STI->hasFeature(MOS::Feature65CE02);
    Value += getRelativeMOSPCCorrection(Kind == MOS::PCRel16, Is65CE02);
    break;
  }
  default:
    break;
  }

  unsigned int Bytes = 0;
  switch (Kind) {
  case MOS::Imm8:
  case MOS::Addr8:
  case MOS::Addr16_High:
  case MOS::Addr16_Low:
  case MOS::Addr24_Bank:
  case MOS::Addr24_Segment_Low:
  case MOS::Addr24_Segment_High:
  case MOS::PCRel8:
  case FK_Data_1:
    Bytes = 1;
    break;
  case FK_Data_2:
  case MOS::Imm16:
  case MOS::Addr13:
  case MOS::Addr16:
  case MOS::Addr24_Segment:
  case MOS::PCRel16:
    Bytes = 2;
    break;
  case MOS::Addr24:
    Bytes = 3;
    break;
  case FK_Data_4:
    Bytes = 4;
    break;
  case FK_Data_8:
    Bytes = 8;
    break;
  default:
    llvm_unreachable("unknown fixup kind");
    return;
  }

  for (uint8_t &Out : make_range(Data, Data + Bytes)) {
    Out = Value & 0xff;
    Value = Value >> 8;
  }
}

unsigned MOSAsmBackend::relaxInstructionTo(unsigned Opcode,
                                           const MCSubtargetInfo &STI,
                                           bool &BankRelax) {
  // Attempt branch relaxation.
  const auto *BIRE = MOS::getBranchInstructionRelaxationEntry(Opcode);
  if (BIRE) {
    if (STI.hasFeature(MOS::FeatureW65816)) {
      if (BIRE->To == MOS::BRA_Relative16)
        return MOS::BRL_Relative16;
      return 0;
    }

    if (STI.hasFeature(MOS::Feature65CE02)) {
      return BIRE->To;
    }

    return 0;
  }

  // Attempt zero page/bank relaxation.
  const auto *ZPIRE = MOS::getZeroPageInstructionRelaxationEntry(Opcode);
  if (ZPIRE)
    return ZPIRE->To;

  if (STI.hasFeature(MOS::FeatureW65816)) {
    // Attempt zero-bank relaxation on 65816.
    const auto *ZBIRE = MOS::getZeroBankInstructionRelaxationEntry(Opcode);
    if (ZBIRE) {
      BankRelax = true;
      return ZBIRE->To;
    }
  }

  return 0;
}

template <typename Fn>
static bool visitRelaxableOperand(unsigned Opcode, ArrayRef<MCOperand> Operands,
                                  const MCSubtargetInfo &STI, Fn Visit) {
  bool BankRelax = false;
  unsigned RelaxTo = MOSAsmBackend::relaxInstructionTo(Opcode, STI, BankRelax);

  return RelaxTo && Operands.size() <= 2 &&
         Visit(Operands[Operands.size() - 1], RelaxTo, BankRelax);
}

static bool isImmediateBankRelaxable(const MCSubtargetInfo &STI, int64_t Imm,
                                     bool BankRelax) {
  if (BankRelax)
    return Imm >= 0 && Imm <= UINT16_MAX;

  uint32_t ZpAddrOffset = STI.hasFeature(MOS::FeatureHUC6280) ? 0x2000 : 0;
  return Imm >= ZpAddrOffset && Imm <= ZpAddrOffset + 0xFF;
}

void MOSAsmBackend::relaxForImmediate(MCInst &Inst,
                                      const MCSubtargetInfo &STI) {
  // Two steps are required for zero-bank relaxation on 65816.
  while (visitRelaxableOperand(Inst.getOpcode(), Inst.getOperands(), STI,
                               [&Inst, &STI](const MCOperand &Operand,
                                             unsigned RelaxTo, bool BankRelax) {
                                 int64_t Imm;
                                 if (!Operand.evaluateAsConstantImm(Imm) ||
                                     isImmediateBankRelaxable(STI, Imm,
                                                              BankRelax)) {
                                   // If the expression evaluates cleanly to an
                                   // 8-bit value (or 16-bit for bank
                                   // relaxation), then it doesn't need
                                   // relaxation.
                                   return false;
                                 }
                                 // This instruction can be relaxed, do it now.
                                 Inst.setOpcode(RelaxTo);
                                 return true;
                               }))
    ;
}

bool MOSAsmBackend::mayNeedRelaxation(unsigned Opcode,
                                      ArrayRef<MCOperand> Operands,
                                      const MCSubtargetInfo &STI) const {
  RelaxedOpcode = Opcode;
  RelaxedSTI = &STI;
  return visitRelaxableOperand(Opcode, Operands, STI,
                               [](const MCOperand &Operand, unsigned RelaxTo,
                                  bool BankRelax) { return Operand.isExpr(); });
}

void MOSAsmBackend::relaxInstruction(MCInst &Inst,
                                     const MCSubtargetInfo &STI) const {
  unsigned Opcode = relaxInstructionTo(Inst.getOpcode(), STI);
  if (Opcode != 0) {
    Inst.setOpcode(Opcode);
  }
}

bool MOSAsmBackend::writeNopData(raw_ostream &OS, uint64_t Count,
                                 const MCSubtargetInfo *STI) const {
  while (Count--) {
    // Sports. It's in the game. Knowing the 6502 hexadecimal representation of
    // a NOP on 6502 used to be an interview question at Electronic Arts.
    OS << '\xea';
  }
  return true;
}

void MOSAsmBackend::translateOpcodeToSubtarget(MCInst &Inst,
                                               const MCSubtargetInfo &STI) {
  if (STI.hasFeature(MOS::FeatureSPC700)) {
    // Convert from 6502 to SPC700 MIs.
    const auto *MOSSPC = MOS::getMOSSPC700Entry(Inst.getOpcode());
    if (MOSSPC)
      Inst.setOpcode(MOSSPC->To);
  }
}

std::unique_ptr<llvm::MCObjectTargetWriter>
MOSAsmBackend::createObjectTargetWriter() const {
  return std::make_unique<MOSELFObjectWriter>(OSType);
}

} // namespace llvm
