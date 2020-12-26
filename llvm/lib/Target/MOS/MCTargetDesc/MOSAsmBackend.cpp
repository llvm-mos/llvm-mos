//===-- MOSAsmBackend.cpp - MOS Asm Backend  ------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MOSAsmBackend class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MOSAsmBackend.h"
#include "MCTargetDesc/MOSELFObjectWriter.h"
#include "MCTargetDesc/MOSFixupKinds.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MCTargetDesc/MOSMCExpr.h"

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
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"

#include <climits>
#include <memory>
#include <cstdint>
#include <string>

namespace llvm {
namespace MOS {
struct ZeroPageInstructionRelaxationEntry {
  unsigned From;
  unsigned To;
};
struct MOSZeroPageSectionEntry {
  const char *Name;
};

#define GET_MOSZeroPageInstructionRelaxationTable_DECL
#define GET_MOSZeroPageInstructionRelaxationTable_IMPL
#define GET_MOSZeroPageSectionTable_DECL
#define GET_MOSZeroPageSectionTable_IMPL
#include "MOSGenSearchableTables.inc"
} // namespace MOS

MCAsmBackend *createMOSAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO) {
  return new MOSAsmBackend(STI.getTargetTriple().getOS());
}

void MOSAsmBackend::adjustFixupValue(const MCFixup &Fixup,
                                     const MCValue &Target, uint64_t &Value,
                                     MCContext *Ctx) const {
  unsigned Kind = Fixup.getKind();

  // Parsed LLVM-generated temporary labels are already
  // adjusted for instruction size, but normal labels aren't.
  //
  // To handle both cases, we simply un-adjust the temporary label
  // case so it acts like all other labels.
  if (const MCSymbolRefExpr *A = Target.getSymA()) {
    if (A->getSymbol().isTemporary()) {
      switch (Kind) {
      case FK_Data_1:
      case FK_Data_2:
      case FK_Data_4:
      case FK_Data_8:
        // Don't shift value for absolute addresses.
        break;
      default:
        Value += 2;
      }
    }
  }

  switch (Kind) {
  case MOS::PCRel8:
    /* MOS pc-relative instructions are counted from the end of the instruction,
     * not the middle of it.
     */
    Value = (Value - 1) & 0xff;
    break;
  case MOS::Addr16_Low:
  case MOS::Addr24_Segment_Low:
    Value = Value & 0xff;
    break;
  case MOS::Addr16_High:
  case MOS::Addr24_Segment_High:
    Value = (Value >> 8) & 0xff;
    break;
  case MOS::Addr24_Segment:
    Value = Value & 0xffff;
    break;
  case MOS::Addr24_Bank:
    Value = (Value >> 16) & 0xff;
    break;

  // Fixups which do not require adjustments.
  case FK_Data_1:
  case FK_Data_2:
  case FK_Data_4:
  case FK_Data_8:
    break;

  default:
    llvm_unreachable("don't know how to adjust this fixup");
    break;
  }
}

void MOSAsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                               const MCValue &Target,
                               MutableArrayRef<char> Data, uint64_t Value,
                               bool IsResolved,
                               const MCSubtargetInfo *STI) const {
  unsigned int Bytes = 0;
  unsigned int Kind = Fixup.getKind();
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
  case MOS::Addr16:
  case MOS::Addr24_Segment:
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
  auto Offset = Fixup.getOffset();
  assert(((Bytes + Offset) <= Data.size()) && 
    "Invalid offset within MOS instruction for modifier!");
  for (unsigned int T = Offset; T < (Bytes + Offset); T++) {
    Data[T] = Value & 0xff;
    Value = Value >> 8;
  }
}

bool MOSAsmBackend::fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                                         const MCRelaxableFragment *DF,
                                         const MCAsmLayout &Layout) const {
  return true;
}

bool MOSAsmBackend::evaluateTargetFixup(const MCAssembler &Asm,
                                        const MCAsmLayout &Layout,
                                        const MCFixup &Fixup,
                                        const MCFragment *DF,
                                        const MCValue &Target, uint64_t &Value,
                                        bool &WasForced) {
  assert(Fixup.getKind() == (MCFixupKind)MOS::PCRel8 &&
         "unexpected target fixup kind");
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
  uint32_t Offset = Layout.getFragmentOffset(DF) + Fixup.getOffset();
  Value -= Offset;
  // MOS's PC relative addressing is off by one from the standard LLVM
  // PC relative convention.
  --Value;
  // If this result fits safely into 8 bits, we're done
  int64_t SignedValue = Value;
  return (INT8_MIN <= SignedValue && SignedValue <= INT8_MAX);
}

bool MOSAsmBackend::fixupNeedsRelaxationAdvanced(const MCFixup &Fixup,
                                                 bool Resolved, uint64_t Value,
                                                 const MCRelaxableFragment *DF,
                                                 const MCAsmLayout &Layout,
                                                 const bool WasForced) const {
  auto Info = getFixupKindInfo(Fixup.getKind());
  const auto *MME = dyn_cast<MOSMCExpr>(Fixup.getValue());
  // If this is a target-specific relaxation, e.g. a modifier, then the Info
  // field already knows the exact width of the answer, so decide now.
  if (MME != nullptr)
  {
    return (Info.TargetSize > 8);
  }
  // Now the fixup kind is not target-specific.  Yet, if it requires more than
  // 8 bits, then relaxation is needed.
  if (Info.TargetSize > 8)
  {
    return true;
  }
  // In order to resolve an eight to sixteen bit possible relaxation, we need to
  // figure out whether the symbol in question is in zero page or not.  If it is
  // in zero page, then we don't need to do anything.  If not, we need to relax
  // the instruction to 16 bits.
  const char *FixupNameStart = Fixup.getValue()->getLoc().getPointer();
  // If there's no symbol name, and if the fixup does not have a known size,
  // then  we can't assume it lives in zero page.
  if (FixupNameStart == nullptr)
  {
    return true;
  }
  size_t FixupLength = 0;
  bool Finished = false;
  do {
    const char C = FixupNameStart[FixupLength];
    if ((C >= 'A' && C <= 'Z') || (C >= 'a' && C <= 'z') ||
        (C >= '0' && C <= '9') || (C == '$') || (C == '_')) {
      ++FixupLength;
      continue;
    }
    Finished = true;
  } while (Finished == false);
  StringRef FixupName(FixupNameStart, FixupLength);
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
      const auto &ELFSectionName = ELFSection->getName();
      /// If the section of the symbol is one of the prenamed zero page
      /// sections, then this is an 8 bit instruction and it doesn't need
      /// relaxation.
      for (const auto &ZeroPageSectionEntry : MOS::MOSZeroPageSectionTable) {
        const StringRef ZeroPageName(ZeroPageSectionEntry.Name);
        if (ZeroPageName.equals(ELFSectionName)) {
          /// This symbol goes in zero page.  No relaxation is needed.
          return false;
        }
      }
    }
  }
  // we have no convincing reason not to do the relaxation
  return true;
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

unsigned MOSAsmBackend::relaxInstructionTo(const MCInst &Inst) const {
  const auto *ZPIRE =
      MOS::getZeroPageInstructionRelaxationEntry(Inst.getOpcode());
  if (ZPIRE == nullptr) {
    return 0;
  }
  return ZPIRE->To;
}

bool MOSAsmBackend::mayNeedRelaxation(const MCInst &Inst,
                                      const MCSubtargetInfo &STI) const {
  unsigned relaxTo = relaxInstructionTo(Inst);
  if (relaxTo == 0) {
    // If the instruction can't be relaxed, then it doesn't need relaxation.
    return false;
  }
  if (Inst.getNumOperands() != 1) {
    // If the instruction doesn't have exactly one operand, then it doesn't
    // need relaxation.
    return false;
  }
  const auto &Operand = Inst.getOperand(0);
  if (Operand.isExpr() == false) {
    // If the instruction isn't an expression, then it doesn't need relaxation.
    return false;
  }
  int64_t Imm;
  if (Operand.evaluateAsConstantImm(Imm) && Imm >= 0 && Imm <= UCHAR_MAX) {
    // If the expression evaluates cleanly to an 8-bit value, then it doesn't
    // need relaxation.
    return false;
  }
  // okay you got us, it MAY need relaxation, if the instruction CAN be
  // relaxed.
  return true;
}

void MOSAsmBackend::relaxInstruction(MCInst &Inst,
                                     const MCSubtargetInfo &STI) const {

  unsigned Opcode = relaxInstructionTo(Inst);
  if (Opcode != 0) {
    Inst.setOpcode(Opcode);
  }
}

bool MOSAsmBackend::writeNopData(raw_ostream &OS, uint64_t Count) const {
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