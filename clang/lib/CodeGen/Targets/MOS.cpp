//===- MOS.cpp -----------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "ABIInfoImpl.h"
#include "TargetInfo.h"

using namespace clang;
using namespace clang::CodeGen;

//===----------------------------------------------------------------------===//
// MOS ABI Implementation
//===----------------------------------------------------------------------===//

namespace {

class MOSABIInfo : public DefaultABIInfo {
public:
  MOSABIInfo(CodeGen::CodeGenTypes &CGT) : DefaultABIInfo(CGT) {}

  ABIArgInfo classifyArgumentType(QualType Ty) const;
  ABIArgInfo classifyReturnType(QualType RetTy) const;

  void computeInfo(CGFunctionInfo &FI) const override {
    if (!getCXXABI().classifyReturnType(FI))
      FI.getReturnInfo() = classifyReturnType(FI.getReturnType());
    for (auto &I : FI.arguments())
      I.info = classifyArgumentType(I.type);
  }

  Address EmitVAArg(CodeGenFunction &CGF, Address VAListAddr,
                    QualType Ty) const override;
};

class MOSTargetCodeGenInfo : public TargetCodeGenInfo {
public:
  MOSTargetCodeGenInfo(CodeGen::CodeGenTypes &CGT)
      : TargetCodeGenInfo(std::make_unique<MOSABIInfo>(CGT)) {}

  void setTargetAttributes(const Decl *D, llvm::GlobalValue *GV,
                           CodeGen::CodeGenModule &CGM) const override {
    if (GV->isDeclaration())
      return;
    const auto *FD = dyn_cast_or_null<FunctionDecl>(D);
    if (!FD)
      return;
    auto *Fn = cast<llvm::Function>(GV);

    if (FD->getAttr<MOSInterruptAttr>())
      Fn->addFnAttr("interrupt");
    if (FD->getAttr<MOSInterruptNorecurseAttr>())
      Fn->addFnAttr("interrupt-norecurse");
    if (FD->getAttr<MOSNoISRAttr>())
      Fn->addFnAttr("no-isr");
  }
};

} // end anonymous namespace

ABIArgInfo MOSABIInfo::classifyArgumentType(QualType Ty) const {
  Ty = useFirstFieldIfTransparentUnion(Ty);

  if (isAggregateTypeForABI(Ty)) {
    // Large records or those with non-trivial destructors/copy-constructors
    // should not be passed by value.
    if (getRecordArgABI(Ty, getCXXABI()) == CGCXXABI::RAA_DirectInMemory ||
        getContext().getTypeSize(Ty) > 32)
      return getNaturalAlignIndirect(Ty, false);

    if (isEmptyRecord(getContext(), Ty, true))
      return ABIArgInfo::getIgnore();

    return ABIArgInfo::getDirect();
  }
  return DefaultABIInfo::classifyArgumentType(Ty);
}

ABIArgInfo MOSABIInfo::classifyReturnType(QualType RetTy) const {
  // Large records should not be passed by value.
  if (isAggregateTypeForABI(RetTy)) {
    if (isEmptyRecord(getContext(), RetTy, true))
      return ABIArgInfo::getIgnore();

    return getContext().getTypeSize(RetTy) > 32
               ? getNaturalAlignIndirect(RetTy, false)
               : ABIArgInfo::getDirect();
  }
  return DefaultABIInfo::classifyReturnType(RetTy);
}

Address MOSABIInfo::EmitVAArg(CodeGenFunction &CGF, Address VAListAddr,
                              QualType Ty) const {
  ABIArgInfo ArgInfo = classifyArgumentType(Ty);
  return emitVoidPtrVAArg(CGF, VAListAddr, Ty, ArgInfo.isIndirect(),
                          getContext().getTypeInfoInChars(Ty),
                          /*SlotSize=*/CharUnits::One(),
                          /*AllowHigherAlign=*/true);
}

std::unique_ptr<TargetCodeGenInfo>
CodeGen::createMOSTargetCodeGenInfo(CodeGenModule &CGM) {
  return std::make_unique<MOSTargetCodeGenInfo>(CGM.getTypes());
}
