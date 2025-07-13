//===-- MOSMCAsmInfo.cpp - MOS asm properties -----------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the MOSMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "MOSMCAsmInfo.h"
#include "MCTargetDesc/MOSMCExpr.h"
#include "MOSMCTargetDesc.h"

#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/TargetParser/Triple.h"

namespace llvm {

static const MCAsmInfo::AtSpecifier AtSpecifiers[] = {
    {MOSMCExpr::VK_IMM8, "mosimm8"},
    {MOSMCExpr::VK_IMM16, "mosimm16"},
    {MOSMCExpr::VK_ADDR8, "mos8"},
    {MOSMCExpr::VK_ADDR16, "mos16"},
    {MOSMCExpr::VK_ADDR16_LO, "mos16lo"},
    {MOSMCExpr::VK_ADDR16_HI, "mos16hi"},
    {MOSMCExpr::VK_ADDR24, "mos24"},
    {MOSMCExpr::VK_ADDR24_BANK, "mos24bank"},
    {MOSMCExpr::VK_ADDR24_SEGMENT, "mos24segment"},
    {MOSMCExpr::VK_ADDR24_SEGMENT_LO, "mos24segmentlo"},
    {MOSMCExpr::VK_ADDR24_SEGMENT_HI, "mos24segmenthi"},
    {MOSMCExpr::VK_ADDR13, "mos13"},
};

MOSMCAsmInfo::MOSMCAsmInfo(const Triple &TT, const MCTargetOptions &Options) {
  // While the platform uses 2-byte pointers, the ELF files use 4-byte pointers
  // to convey banking information; this field is used, among others, by the
  // DWARF debug structures.
  CodePointerSize = 4;
  CalleeSaveStackSlotSize = 0;
  SeparatorString = "\n";
  CommentString = ";";
  UseMotorolaIntegers = true;
  // Required for SPC700 $xx.0 bit addressing.
  DotAsIntSeparator = true;
  // Maximum instruction length across all supported subtargets.
  MaxInstLength = 7;
  SupportsDebugInformation = true;

  initializeAtSpecifiers(AtSpecifiers);
}

unsigned MOSMCAsmInfo::getMaxInstLength(const MCSubtargetInfo *STI) const {
  if (!STI)
    return MaxInstLength;

  if (STI->hasFeature(MOS::FeatureHUC6280))
    return 7;
  if (STI->hasFeature(MOS::Feature45GS02))
    return 5;
  if (STI->hasFeature(MOS::FeatureW65816))
    return 4;
  return 3;
}

} //  namespace llvm
