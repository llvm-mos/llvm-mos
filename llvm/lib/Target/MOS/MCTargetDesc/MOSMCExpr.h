//===-- MOSMCExpr.h - MOS specific MC expression classes --------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_MCEXPR_H
#define LLVM_MOS_MCEXPR_H

#include "MCTargetDesc/MOSFixupKinds.h"
#include "llvm/MC/MCExpr.h"

namespace llvm {

/// A expression in MOS machine code.
class MOSMCExpr : public MCTargetExpr {
public:
  /// Specifies the type of an expression.
  enum VariantKind {
    VK_NONE,

    VK_ADDR16 = MCSymbolRefExpr::FirstTargetSpecifier,
    VK_IMM16,
    VK_ADDR8,
    VK_IMM8,
    VK_ADDR16_HI,
    VK_ADDR16_LO,
    VK_ADDR24,
    VK_ADDR24_BANK,
    VK_ADDR24_SEGMENT,
    VK_ADDR24_SEGMENT_LO,
    VK_ADDR24_SEGMENT_HI,
    VK_ADDR13,
    VK_ADDR_ASCIZ
  };

  /// Creates an MOS machine code expression.
  static const MOSMCExpr *create(VariantKind Kind, const MCExpr *Expr,
                                 bool IsNegated, MCContext &Ctx);

  /// Gets the type of the expression.
  VariantKind getKind() const { return Kind; }
  /// Gets the name of the expression.
  MOS::Fixups getFixupKind() const;
  const char *getName() const;
  const MCExpr *getSubExpr() const { return SubExpr; }
  /// Gets the fixup which corresponds to the expression.
  /// Evaluates the fixup as a constant value.
  bool evaluateAsConstant(int64_t &Result) const;

  bool isNegated() const { return Negated; }
  void setNegated(bool NegatedIn = true) { Negated = NegatedIn; }

  void printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const override;
  bool evaluateAsRelocatableImpl(MCValue &Res,
                                 const MCAssembler *Asm) const override;

  void visitUsedExpr(MCStreamer &Streamer) const override;

  MCFragment *findAssociatedFragment() const override {
    return getSubExpr()->findAssociatedFragment();
  }

  static bool classof(const MCExpr *E) {
    return E->getKind() == MCExpr::Target;
  }

  static VariantKind getKindByName(StringRef Name, bool IsImmediate);

private:
  int64_t evaluateAsInt64(int64_t Value) const;

  const VariantKind Kind;
  const MCExpr *SubExpr;
  bool Negated;

  explicit MOSMCExpr(VariantKind Kind, const MCExpr *Expr, bool Negated)
      : Kind(Kind), SubExpr(Expr), Negated(Negated) {}
};

} // end namespace llvm

#endif // LLVM_MOS_MCEXPR_H
