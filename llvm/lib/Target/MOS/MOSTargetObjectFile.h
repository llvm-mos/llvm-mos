//===-- MOSTargetObjectFile.h - MOS Object Info -----------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_TARGET_OBJECT_FILE_H
#define LLVM_MOS_TARGET_OBJECT_FILE_H

#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {

/// Lowering for an MOS ELF32 object file.
class MOSTargetObjectFile : public TargetLoweringObjectFileELF {
  MCSection *getExplicitSectionGlobal(const GlobalObject *GO, SectionKind Kind,
                                      const TargetMachine &TM) const override;
};

} // end namespace llvm

#endif // LLVM_MOS_TARGET_OBJECT_FILE_H
