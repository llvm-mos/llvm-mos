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
#define GET_ZeroPageSectionTable_DECL
#define GET_ZeroPageSectionTable_IMPL
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

static uint64_t getSymbolOffset(const MCSymbolRefExpr *SymRefExpr, const MCAsmLayout &Layout) {
  if (!SymRefExpr) {
      return 0;
  }

  const MCSymbol &Sym = SymRefExpr->getSymbol();

  return Sym.isDefined() ? Layout.getSymbolOffset(Sym) : 0;
}

static auto getCumulativeSymbolOffset(const MCValue &Target, const MCAsmLayout &Layout) {
  return getSymbolOffset(Target.getSymB(), Layout) - getSymbolOffset(Target.getSymA(), Layout);
}

static auto getRelativeMOSPCCorrection(const bool IsPCRel16) {
  // MOS's PC relative addressing is off by one or two from the standard LLVM
  // PC relative convention.
  return IsPCRel16 ? 2 : 1;
}

static bool fitsIntoFixup(const int64_t SignedValue, const bool IsPCRel16) {
  return SignedValue >= (IsPCRel16 ? INT16_MIN : INT8_MIN)
    && SignedValue <= (IsPCRel16 ? INT16_MAX : INT8_MAX);
}

static auto getFixupValue(const MCAsmLayout &Layout, const MCFixup &Fixup,
			  const MCFragment *DF, const MCValue &Target, const bool IsPCRel16) {
  assert((IsPCRel16 || Fixup.getKind() == (MCFixupKind)MOS::PCRel8) &&
         "unexpected target fixup kind");

  return Target.getConstant()
    - getCumulativeSymbolOffset(Target, Layout)
    - Layout.getFragmentOffset(DF)
    - Fixup.getOffset()
    - getRelativeMOSPCCorrection(IsPCRel16);
}

bool MOSAsmBackend::evaluateTargetFixup(const MCAssembler &Asm,
                                        const MCAsmLayout &Layout,
                                        const MCFixup &Fixup,
                                        const MCFragment *DF,
                                        const MCValue &Target, uint64_t &Value,
                                        bool &WasForced) {
  // ForcePCRelReloc is a CLI option to force relocation emit, primarily for testing
  // R_MOS_PCREL_*.
  WasForced = ForcePCRelReloc;

  const bool IsPCRel16 = Fixup.getKind() == (MCFixupKind)MOS::PCRel16;
  Value = getFixupValue(Layout, Fixup, DF, Target, IsPCRel16);

  return !WasForced && fitsIntoFixup(Value, IsPCRel16);
}

static bool isSymbolChar(const char C) {
  return (C >= 'A' && C <= 'Z') || (C >= 'a' && C <= 'z')
      || (C >= '0' && C <= '9') || (C == '$') || (C == '_');
}

static size_t getFixupLength(const char *FixupNameStart) {
  size_t i;
  for (i = 0; isSymbolChar(FixupNameStart[i]); i++)
    ;
  return i;
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
  if (MME != nullptr) {
    return (Info.TargetSize > (BankRelax ? 16 : 8));
  }
  // Now the fixup kind is not target-specific.  Yet, if it requires more than
  // 8 (or 16) bits, then relaxation is needed.
  if (Info.TargetSize > (BankRelax ? 16 : 8)) {
    return true;
  }
  // In order to resolve an eight to sixteen bit possible relaxation, we need to
  // figure out whether the symbol in question is in zero page or not.  If it is
  // in zero page, then we don't need to do anything.  If not, we need to relax
  // the instruction to 16 bits.
  const char *FixupNameStart = Fixup.getValue()->getLoc().getPointer();
  // If there's no symbol name, and if the fixup does not have a known size,
  // then we can't assume it lives in zero page.
  if (FixupNameStart == nullptr) {
    return true;
  }

  StringRef FixupName(FixupNameStart, getFixupLength(FixupNameStart));

  // The list of symbols is maintained by the assembler, and since that list
  // is not maintained in alpha order, it seems that we need to iterate across
  // it to find the symbol in question... is there a non-O(n) way to do this?
  for (const auto &Symbol : Layout.getAssembler().symbols()) {
    const auto SymbolName = Symbol.getName();
    if (FixupName == SymbolName) {
      // If this symbol has not been assigned to a section, then it can't
      // be in zero page
      if (!Symbol.isInSection()) {
        return true;
      }
      const auto &Section = Symbol.getSection();
      const auto *ELFSection = dyn_cast_or_null<MCSectionELF>(&Section);
      /// If we're not writing to ELF, punt on this whole idea, just do the
      /// relaxation for safety's sake
      if (ELFSection == nullptr) {
        return true;
      }
      /// If the section of the symbol is marked with special zero-page flag
      /// then this is an 8 bit instruction and it doesn't need
      /// relaxation.
      if ((ELFSection->getFlags() & ELF::SHF_MOS_ZEROPAGE) != 0) {
        return false;
      }
      /// If the section of the symbol is one of the prenamed zero page
      /// sections, then this is an 8 bit instruction and it doesn't need
      /// relaxation.
      if (isZeroPageSectionName(ELFSection->getName()))
        return false;
    }
  }
  // we have no convincing reason not to do the relaxation
  return true;
}

bool MOSAsmBackend::isZeroPageSectionName(StringRef Name) {
  return is_contained(MOS::ZeroPageSectionTable, Name);
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

  return RelaxTo && Inst.getNumOperands() == 1 && Visit(Inst.getOperand(0),
							RelaxTo, BankRelax);
}

void MOSAsmBackend::relaxForImmediate(MCInst &Inst,
                                      const MCSubtargetInfo &STI) {
  // Two steps are required for zero-bank relaxation on 65816.
  while (visitRelaxableOperand(
      Inst, STI,
      [&Inst](const MCOperand &Operand, unsigned RelaxTo, bool BankRelax) {
        int64_t Imm;
        if (!Operand.evaluateAsConstantImm(Imm) ||
            (Imm >= 0 && Imm <= (BankRelax ? UINT16_MAX : UCHAR_MAX))) {
          // If the expression evaluates cleanly to an 8-bit value
          // (or 16-bit for bank relaxation), then it doesn't need relaxation.
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
  return visitRelaxableOperand(
      Inst, STI,
      [](const MCOperand &Operand, unsigned RelaxTo, bool BankRelax) {
        return Operand.isExpr();
      });
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
  // todo: fix for virtual targets
  while ((Count--) > 0) {
    OS << 0xEA; // Sports. It's in the game.  Knowing the 6502 hexadecimal
                // representation of a NOP on 6502, used to be an interview
                // question at Electronic Arts.
  }
  return true;
}

std::unique_ptr<llvm::MCObjectTargetWriter>
MOSAsmBackend::createObjectTargetWriter() const {
  return std::make_unique<MOSELFObjectWriter>(OSType);
}

} // namespace llvm
