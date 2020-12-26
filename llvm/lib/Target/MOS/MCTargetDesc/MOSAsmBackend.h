//===-- MOSAsmBackend.h - MOS Asm Backend  --------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// \file The MOS assembly backend implementation.
//
//===----------------------------------------------------------------------===//
//

#ifndef LLVM_MOS_ASM_BACKEND_H
#define LLVM_MOS_ASM_BACKEND_H

#include "MCTargetDesc/MOSFixupKinds.h"

#include "llvm/ADT/Triple.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCObjectWriter.h"

namespace llvm {

class MCAssembler;
class Target;

struct MCFixupKindInfo;

class MOSObjectTargetWriter : public MCObjectTargetWriter
{
  virtual Triple::ObjectFormatType getFormat() const override
  {
    return Triple::ObjectFormatType::ELF;
  }
};

/// Utilities for manipulating generated MOS machine code.
class MOSAsmBackend : public MCAsmBackend {
public:
  MOSAsmBackend(Triple::OSType OSType)
      : llvm::MCAsmBackend(support::little)
      // , OSType(OSType) 
      {}

  /// Apply the \p Value for given \p Fixup into the provided data fragment, at
  /// the offset specified by the fixup and following the fixup kind as
  /// appropriate. Errors (such as an out of range fixup value) should be
  /// reported via \p Ctx.
  /// The  \p STI is present only for fragments of type MCRelaxableFragment and
  /// MCDataFragment with hasInstructions() == true.
  virtual void applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                          const MCValue &Target, MutableArrayRef<char> Data,
                          uint64_t Value, bool IsResolved,
                          const MCSubtargetInfo *STI) const override;

  virtual std::unique_ptr<MCObjectTargetWriter>
  createObjectTargetWriter() const override;
  /// Simple predicate for targets where !Resolved implies requiring relaxation
  virtual bool fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                                    const MCRelaxableFragment *DF,
                                    const MCAsmLayout &Layout) const override;
  virtual unsigned getNumFixupKinds() const override;
  /// Check whether the given instruction may need relaxation.
  ///
  /// \param Inst - The instruction to test.
  /// \param STI - The MCSubtargetInfo in effect when the instruction was
  /// encoded.
  virtual bool mayNeedRelaxation(const MCInst &Inst,
                                 const MCSubtargetInfo &STI) const override;

    /// Relax the instruction in the given fragment to the next wider instruction.
  ///
  /// \param Inst The instruction to relax, which may be the same as the
  /// output.
  /// \param STI the subtarget information for the associated instruction.
  /// \param [out] Res On return, the relaxed instruction.
  virtual void relaxInstruction(const MCInst &Inst, const MCSubtargetInfo &STI,
                                MCInst &Res) const override;


  /// Write an (optimal) nop sequence of Count bytes to the given output. If the
  /// target cannot generate such a sequence, it should return an error.
  ///
  /// \return - True on success.
  virtual bool writeNopData(raw_ostream &OS, uint64_t Count) const override;

private:
 // Triple::OSType OSType;
};

} // end namespace llvm

#endif // LLVM_MOS_ASM_BACKEND_H
