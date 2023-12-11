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
#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCFragment.h"
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

struct MOSSPC700Entry {
  unsigned From;
  unsigned To;
};

#define GET_MOSSPC700Table_DECL
#define GET_MOSSPC700Table_IMPL

#include "MOSGenSearchableTables.inc"
} // namespace MOS

MCAsmBackend *createMOSAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO) {
  return new MOSAsmBackend(STI.getTargetTriple().getOS());
}

void MOSAsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                               const MCValue &Target,
                               MutableArrayRef<char> Data, uint64_t Value,
                               bool IsResolved,
                               const MCSubtargetInfo *STI) const {
  unsigned int Kind = Fixup.getKind();
  uint32_t Offset = Fixup.getOffset();

  if (Kind == MOS::AddrAsciz) {
    std::string ValueStr = utostr(Value);
    assert(((ValueStr.size() + 1 + Offset) <= Data.size()) &&
           "Invalid offset within MOS instruction for modifier!");
    std::copy(ValueStr.begin(), ValueStr.end(), Data.begin() + Offset);
    Data[Offset + ValueStr.size()] = '\0';
    return;
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

  assert(((Bytes + Offset) <= Data.size()) &&
         "Invalid offset within MOS instruction for modifier!");
  char *RangeStart = Data.begin() + Offset;

  for (char &Out : make_range(RangeStart, RangeStart + Bytes)) {
    Out = Value & 0xff;
    Value = Value >> 8;
  }
}

bool MOSAsmBackend::fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                                         const MCRelaxableFragment *DF,
                                         const MCAsmLayout &Layout) const {
  return true;
}

static cl::opt<bool> ForcePCRelReloc(
    "mos-force-pcrel-reloc",
    cl::desc("Force relocation entries to be emitted for PCREL fixups."),
    cl::init(false), cl::Hidden);

static int getRelativeMOSPCCorrection(const bool IsPCRel16) {
  // MOS's PC relative addressing is off by one or two from the standard LLVM
  // PC relative convention.
  return IsPCRel16 ? -2 : -1;
}

static bool fitsIntoFixup(const int64_t SignedValue, const bool IsPCRel16) {
  return SignedValue >= (IsPCRel16 ? INT16_MIN : INT8_MIN) &&
         SignedValue <= (IsPCRel16 ? INT16_MAX : INT8_MAX);
}

bool MOSAsmBackend::evaluateTargetFixup(
    const MCAssembler &Asm, const MCAsmLayout &Layout, const MCFixup &Fixup,
    const MCFragment *DF, const MCValue &Target, const MCSubtargetInfo *STI,
    uint64_t &Value, bool &WasForced) {
  // ForcePCRelReloc is a CLI option to force relocation emit, primarily for
  // testing R_MOS_PCREL_*.
  WasForced = ForcePCRelReloc;

  const bool IsPCRel16 = Fixup.getKind() == (MCFixupKind)MOS::PCRel16;
  assert((IsPCRel16 || Fixup.getKind() == (MCFixupKind)MOS::PCRel8) &&
         "unexpected target fixup kind");

  // Logic taken from MCAssembler::evaluateFixup.
  bool IsResolved = false;
  if (Target.getSymB()) {
    IsResolved = false;
  } else if (!Target.getSymA()) {
    IsResolved = false;
  } else {
    const MCSymbolRefExpr *A = Target.getSymA();
    const MCSymbol &SA = A->getSymbol();
    if (A->getKind() != MCSymbolRefExpr::VK_None || SA.isUndefined()) {
      IsResolved = false;
    } else if (auto *Writer = Asm.getWriterPtr()) {
      IsResolved = Writer->isSymbolRefDifferenceFullyResolvedImpl(Asm, SA, *DF,
                                                                  false, true);
    }
  }

  Value = Target.getConstant();

  if (const MCSymbolRefExpr *A = Target.getSymA()) {
    const MCSymbol &Sym = A->getSymbol();
    if (Sym.isDefined())
      Value += Layout.getSymbolOffset(Sym);
  }
  if (const MCSymbolRefExpr *B = Target.getSymB()) {
    const MCSymbol &Sym = B->getSymbol();
    if (Sym.isDefined())
      Value -= Layout.getSymbolOffset(Sym);
  }

  Value -= Layout.getFragmentOffset(DF) + Fixup.getOffset();
  Value += getRelativeMOSPCCorrection(IsPCRel16);

  return IsResolved && !WasForced && fitsIntoFixup(Value, IsPCRel16);
}

// Derived from findAssociatedFragment.
bool isBasedOnZeroPageSymbol(const MCExpr *E) {
  switch (E->getKind()) {
  case MCExpr::Target:
    return isBasedOnZeroPageSymbol(cast<MOSMCExpr>(E)->getSubExpr());

  case MCExpr::Constant:
    return false;

  case MCExpr::SymbolRef:
    return cast<MCSymbolELF>(cast<MCSymbolRefExpr>(E)->getSymbol()).getOther() &
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

bool MOSAsmBackend::fixupNeedsRelaxationAdvanced(const MCFixup &Fixup,
                                                 bool Resolved, uint64_t Value,
                                                 const MCRelaxableFragment *DF,
                                                 const MCAsmLayout &Layout,
                                                 const bool WasForced) const {
  // On 65816, it is possible to zero-bank relax from Addr16 to Addr24. The
  // assembler relaxes in a loop until instructions cannot be relaxed further,
  // so this is able to follow zero-page relaxation.
  bool BankRelax = false;
  MOSAsmBackend::relaxInstructionTo(DF->getInst(), *DF->getSubtargetInfo(),
                                    BankRelax);

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

  // See if the expression is derived from a zero page symbol.
  if (isBasedOnZeroPageSymbol(Fixup.getValue()))
    return false;

  // In order to resolve an eight to sixteen bit possible relaxation, we need
  // to figure out whether the symbol in question is in zero page or not.  If
  // it is in zero page, then we don't need to do anything.  If not, we need
  // to relax the instruction to 16 bits.
  MCFragment *Frag = Fixup.getValue()->findAssociatedFragment();
  if (!Frag)
    return true;

  // If we're not writing to ELF, punt on this whole idea, just do the
  // relaxation for safety's sake
  const auto *Sec = dyn_cast_if_present<MCSectionELF>(Frag->getParent());
  if (!Sec)
    return true;

  // If the section of the symbol is marked with special zero-page flag
  // then this is an 8 bit instruction and it doesn't need
  // relaxation.
  if (Sec->getFlags() & ELF::SHF_MOS_ZEROPAGE)
    return false;

  return !MOS::isZeroPageSectionName(Sec->getName());
}

MCFixupKindInfo const &MOSAsmBackend::getFixupKindInfo(MCFixupKind Kind) const {
  if (Kind < FirstTargetFixupKind) {
    return MCAsmBackend::getFixupKindInfo(Kind);
  }

  return MOSFixupKinds::getFixupKindInfo(static_cast<MOS::Fixups>(Kind), this);
}

unsigned MOSAsmBackend::getNumFixupKinds() const {
  return MOS::Fixups::NumTargetFixupKinds;
}

unsigned MOSAsmBackend::relaxInstructionTo(const MCInst &Inst,
                                           const MCSubtargetInfo &STI,
                                           bool &BankRelax) {
  const auto *ZPIRE =
      MOS::getZeroPageInstructionRelaxationEntry(Inst.getOpcode());
  if (ZPIRE)
    return ZPIRE->To;

  if (STI.hasFeature(MOS::FeatureW65816)) {
    // Attempt zero-bank relaxation on 65816.
    const auto *ZBIRE =
        MOS::getZeroBankInstructionRelaxationEntry(Inst.getOpcode());
    if (ZBIRE) {
      BankRelax = true;
      return ZBIRE->To;
    }
  }

  return 0;
}

template <typename Fn>
static bool visitRelaxableOperand(const MCInst &Inst,
                                  const MCSubtargetInfo &STI, Fn Visit) {
  bool BankRelax = false;
  unsigned RelaxTo = MOSAsmBackend::relaxInstructionTo(Inst, STI, BankRelax);

  return RelaxTo && Inst.getNumOperands() <= 2 &&
         Visit(Inst.getOperand(Inst.getNumOperands() - 1), RelaxTo, BankRelax);
}

static bool isImmediateBankRelaxable(const MCSubtargetInfo &STI, int64_t Imm,
                                     bool BankRelax) {
  if (BankRelax)
    return Imm >= 0 && Imm <= UINT16_MAX;

  uint32_t ZpAddrOffset =
      static_cast<const MOSSubtarget &>(STI).getZeroPageOffset();
  return Imm >= ZpAddrOffset && Imm <= ZpAddrOffset + 0xFF;
}

void MOSAsmBackend::relaxForImmediate(MCInst &Inst,
                                      const MCSubtargetInfo &STI) {
  // Two steps are required for zero-bank relaxation on 65816.
  while (visitRelaxableOperand(Inst, STI,
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

bool MOSAsmBackend::mayNeedRelaxation(const MCInst &Inst,
                                      const MCSubtargetInfo &STI) const {
  return visitRelaxableOperand(Inst, STI,
                               [](const MCOperand &Operand, unsigned RelaxTo,
                                  bool BankRelax) { return Operand.isExpr(); });
}

void MOSAsmBackend::relaxInstruction(MCInst &Inst,
                                     const MCSubtargetInfo &STI) const {
  unsigned Opcode = relaxInstructionTo(Inst, STI);
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
