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

#include <memory>
#include <stdint.h>
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
  case MOS::Addr24_Bank_Low:
    Value = Value & 0xff;
    break;
  case MOS::Addr16_High:
  case MOS::Addr24_Bank_High:
    Value = (Value >> 8) & 0xff;
    break;
  case MOS::Addr24_Bank:
    Value = Value & 0xffff;
    break;
  case MOS::Addr24_Segment:
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
  case MOS::Addr24_Segment:
  case MOS::PCRel8:
  case FK_Data_1:
    Bytes = 1;
    break;
  case FK_Data_2:
  case MOS::Addr16:
    Bytes = 2;
    break;
  case MOS::Addr24:
    Bytes = 3;
    break;
  case FK_Data_4:
    Bytes = 4;
    break;
  default:
    llvm_unreachable("unknown fixup kind");
    return;
  }
  auto Offset = Fixup.getOffset();
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
  // In order to resolve an eight to sixteen bit possible relaxation, we need to
  // figure out whether the symbol in question is in zero page or not.  If it is
  // in zero page, then we don't need to do anything.  If not, we need to relax
  // the instruction to 16 bits.
  const char *FixupNameStart = Fixup.getValue()->getLoc().getPointer();
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
      const auto &Section = Symbol.getSection();
      const auto *ELFSection = dyn_cast_or_null<MCSectionELF>(&Section);
      /// If we're not writing to ELF, punt on this whole idea, just do the
      /// relaxation for safety's sake
      if (ELFSection == nullptr) {
        return true;
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
  // NOTE: Many AVR fixups work on sets of non-contignous bits. We work around
  // this by saying that the fixup is the size of the entire instruction.
  const static MCFixupKindInfo Infos[MOS::NumTargetFixupKinds] = {
      // This table *must* be in same the order of fixup_* kinds in
      // MOSFixupKinds.h.
      //
      // name, offset, bits, flags
      {"Imm8", 0, 8, 0},            // An 8 bit immediate value.
      {"Addr8", 0, 8, 0},           // An 8 bit zero page address.
      {"Addr16", 0, 16, 0},         // A 16-bit address.
      {"Addr16_Low", 0, 8, 0},      // The low byte of a 16-bit address.
      {"Addr16_High", 0, 8, 0},     // The high byte of a 16-bit address.
      {"Addr24", 0, 24, 0},         // A 24-bit 65816 address.
      {"Addr24_Segment", 0, 8, 0},  // The segment byte of a 24-bit address.
      {"Addr24_Bank", 0, 16, 0},    // The bank of a 24-byte address.
      {"Addr24_Bank_Low", 0, 8, 0}, // The low byte of the bank of a 24-bit addr
      {"Addr24_Bank_High", 8, 8, 0}, // The hi byte of the bank of a 24-bit addr
      {"PCRel8", 0, 8,
       MCFixupKindInfo::FKF_IsPCRel | MCFixupKindInfo::FKF_IsTarget}};
  if (Kind < FirstTargetFixupKind)
    return MCAsmBackend::getFixupKindInfo(Kind);
  assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
         "Invalid kind!");
  return Infos[Kind - FirstTargetFixupKind];
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
  Operand.evaluateAsConstantImm(Imm);
  if ((Imm >= 0x01) && (Imm <= 0xff)) {
    // If the expression evaluates cleanly to an 8-bit value, then it doesn't
    // need relaxation.
    return false;
  }
  // okay you got us, it MAY need relaxation, if the instruction CAN be
  // relaxed.
  return (relaxInstructionTo(Inst) != 0);
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