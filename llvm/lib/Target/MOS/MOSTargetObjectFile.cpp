//===-- MOSTargetObjectFile.cpp - MOS Object Files ------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSTargetObjectFile.h"
#include "MOS.h"
#include "MOSTargetMachine.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/GlobalObject.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/SectionKind.h"

using namespace llvm;

void MOSTargetObjectFile::Initialize(MCContext &Ctx, const TargetMachine &TM) {
  TargetLoweringObjectFileELF::Initialize(Ctx, TM);
  ZpDataSection = Ctx.getELFSection(".zp.data", ELF::SHT_PROGBITS,
                                    ELF::SHF_ALLOC | ELF::SHF_WRITE);
  ZpBssSection = Ctx.getELFSection(".zp.bss", ELF::SHT_NOBITS,
                                   ELF::SHF_ALLOC | ELF::SHF_WRITE);
  ZpNoinitSection = Ctx.getELFSection(".zp.noinit", ELF::SHT_NOBITS,
                                      ELF::SHF_ALLOC | ELF::SHF_WRITE);
}

template <typename T> MOS::AddressSpace getAddressSpace(T *V) {
  auto *PT = cast<PointerType>(V->getType());
  assert(PT != nullptr && "unexpected MemSDNode");
  unsigned AS = PT->getAddressSpace();
  if (AS < MOS::NumAddrSpaces)
    return static_cast<MOS::AddressSpace>(AS);
  return MOS::NumAddrSpaces;
}

MCSection *MOSTargetObjectFile::SelectSectionForGlobal(
    const GlobalObject *GO, SectionKind Kind, const TargetMachine &TM) const {
  // Place zero page variables in the .zp sections by default.
  if (getAddressSpace(GO) == MOS::AS_ZeroPage && !GO->hasSection()) {
    if (Kind.isNoInit())
      return ZpNoinitSection;
    if (Kind.isBSS())
      return ZpBssSection;
    return ZpDataSection;
  }

  // Use default ELF handling for all other cases.
  return TargetLoweringObjectFileELF::SelectSectionForGlobal(GO, Kind, TM);
}

MCSection *MOSTargetObjectFile::getExplicitSectionGlobal(
    const GlobalObject *GO, SectionKind SK, const TargetMachine &TM) const {
  StringRef SectionName = GO->getSection();
  if (SectionName == ".zp.bss" || SectionName.starts_with(".zp.bss."))
    SK = SectionKind::getBSS();
  else if (SectionName == ".zp.data" || SectionName.starts_with(".zp.data."))
    SK = SectionKind::getData();
  else if (SectionName == ".zp" || SectionName.starts_with(".zp.") ||
           SectionName.ends_with(".noinit") || SectionName.contains(".noinit."))
    SK = SectionKind::getNoInit();
  return TargetLoweringObjectFileELF::getExplicitSectionGlobal(GO, SK, TM);
}
