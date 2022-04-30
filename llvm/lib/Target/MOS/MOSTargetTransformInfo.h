//===- MOSTargetTransformInfo.h - MOS specific TTI --------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines a TargetTransformInfo::Concept conforming object specific
// to the MOS target machine. It uses the target's detailed information to
// provide more precise answers to certain TTI queries, while letting the
// target-independent and default TTI implementations handle the rest.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSTARGETTRANSFORMINFO_H
#define LLVM_LIB_TARGET_MOS_MOSTARGETTRANSFORMINFO_H

#include "MOSTargetMachine.h"
#include "llvm/CodeGen/BasicTTIImpl.h"

namespace llvm {

class MOSTTIImpl : public BasicTTIImplBase<MOSTTIImpl> {
  using BaseT = BasicTTIImplBase<MOSTTIImpl>;

  friend BaseT;

  const MOSSubtarget *ST;
  const MOSTargetLowering *TLI;

  const MOSSubtarget *getST() const { return ST; }
  const MOSTargetLowering *getTLI() const { return TLI; }

public:
  explicit MOSTTIImpl(const MOSTargetMachine *TM, const Function &F)
      : BaseT(TM, F.getParent()->getDataLayout()), ST(TM->getSubtargetImpl(F)),
        TLI(ST->getTargetLowering()) {}

  // All div, rem, and divrem ops are libcalls, so any possible combination
  // exists.
  bool hasDivRemOp(Type *DataType, bool IsSigned) { return true; }

  bool allowIllegalIntegerIV() const { return true; }
};

} // end namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSTARGETTRANSFORMINFO_H
