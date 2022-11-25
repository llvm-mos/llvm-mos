//===-- MOSTargetObjectFile.cpp - MOS Object Files ------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSTargetObjectFile.h"
#include "llvm/IR/GlobalObject.h"
#include "llvm/MC/SectionKind.h"

using namespace llvm;

MCSection *MOSTargetObjectFile::getExplicitSectionGlobal(
    const GlobalObject *GO, SectionKind SK, const TargetMachine &TM) const {
  StringRef SectionName = GO->getSection();
  if (SectionName == ".zp.bss" || SectionName.startswith(".zp.bss."))
    SK = SectionKind::getBSS();
  else if (SectionName == ".zp.data" || SectionName.startswith(".zp.data."))
    SK = SectionKind::getData();
  else if (SectionName == ".zp" || SectionName.startswith(".zp.") ||
           SectionName.endswith(".noinit") || SectionName.contains(".noinit."))
    SK = SectionKind::getNoInit();
  return TargetLoweringObjectFileELF::getExplicitSectionGlobal(GO, SK, TM);
}
