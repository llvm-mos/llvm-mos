//===-- MOSMCExpr.h - MOS specific MC expression classes --------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_MCEXPR_H
#define LLVM_MOS_MCEXPR_H

#include "llvm/MC/MCExpr.h"

#include "MCTargetDesc/MOSFixupKinds.h"

namespace llvm {

/// A expression in MOS machine code.
class MOSMCExpr : public MCTargetExpr {
public:
  /// Specifies the type of an expression.
  enum VariantKind {
    VK_MOS_None,
  };

  /// Creates an MOS machine code expression.
  static const MOSMCExpr *create(VariantKind Kind, const MCExpr *Expr,
                                 bool isNegated, MCContext &Ctx);

  virtual bool evaluateAsRelocatableImpl(MCValue &Res,
                                         const MCAsmLayout *Layout,
                                         const MCFixup *Fixup) const override {
    // todo
    return true;
  }
  virtual void printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const override {
    // todo
  }
  virtual void visitUsedExpr(MCStreamer &Streamer) const override {
    // todo
  }
  virtual MCFragment *findAssociatedFragment() const override {
    // todo
    return (MCFragment *)nullptr;
  }

  virtual void fixELFSymbolsInTLSFixups(MCAssembler &) const override {
    // todo
  }

  const VariantKind Kind;
  const MCExpr *SubExpr;
  bool Negated;

private:
  explicit MOSMCExpr(VariantKind Kind, const MCExpr *Expr, bool Negated)
      : Kind(Kind), SubExpr(Expr), Negated(Negated) {}
  ~MOSMCExpr() {}
};

} // end namespace llvm

#endif // LLVM_MOS_MCEXPR_H
