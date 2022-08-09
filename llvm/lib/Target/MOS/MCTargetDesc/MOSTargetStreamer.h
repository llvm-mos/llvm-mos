//===-- MOSTargetStreamer.h - MOS Target Streamer --------------*- C++ -*--===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_TARGET_STREAMER_H
#define LLVM_MOS_TARGET_STREAMER_H

#include "llvm/MC/MCELFStreamer.h"

namespace llvm {
class MCStreamer;

/// A generic MOS target output stream.
class MOSTargetStreamer : public MCTargetStreamer {
public:
  explicit MOSTargetStreamer(MCStreamer &S);

  void finish() override;

protected:
  virtual bool hasBSS() = 0;
  virtual bool hasZPBSS() = 0;
  virtual bool hasData() = 0;
  virtual bool hasZPData() = 0;
  virtual bool hasInitArray() = 0;
  virtual bool hasFiniArray() = 0;

  void stronglyReference(StringRef Name, StringRef Comment);
  virtual void stronglyReference(MCSymbol *Sym) = 0;
};

/// A target streamer for textual MOS assembly code.
class MOSTargetAsmStreamer final : public MOSTargetStreamer {
public:
  explicit MOSTargetAsmStreamer(MCStreamer &S);

private:
  void changeSection(const MCSection *CurSection, MCSection *Section,
                     const MCExpr *SubSection, raw_ostream &OS) override;

  bool hasBSS() override { return HasBSS; }
  bool hasZPBSS() override { return HasZPBSS; }
  bool hasData() override { return HasData; }
  bool hasZPData() override { return HasZPData; }
  bool hasInitArray() override { return HasInitArray; }
  bool hasFiniArray() override { return HasFiniArray; }

  void stronglyReference(MCSymbol *Sym) override;

  bool HasBSS = false;
  bool HasZPBSS = false;
  bool HasData = false;
  bool HasZPData = false;
  bool HasInitArray = false;
  bool HasFiniArray = false;
};

/// A target streamer for an MOS ELF object file.
class MOSTargetELFStreamer final : public MOSTargetStreamer {
public:
  MOSTargetELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);

private:
  bool hasBSS() override;
  bool hasZPBSS() override;
  bool hasData() override;
  bool hasZPData() override;
  bool hasInitArray() override;
  bool hasFiniArray() override;

  void stronglyReference(MCSymbol *Sym) override;
};

} // end namespace llvm

#endif // LLVM_MOS_TARGET_STREAMER_H
