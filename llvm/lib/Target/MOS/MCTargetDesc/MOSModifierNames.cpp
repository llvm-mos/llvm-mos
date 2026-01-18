//===-- MOSModifierNames.cpp - MOS modifier name table --------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSModifierNames.h"
#include "llvm/ADT/ArrayRef.h"

namespace llvm {
namespace MOS {

static const struct ModifierEntry ModifierNames[] = {
    // Define immediate variants of mos8() and mos16() first.
    {"mos8", MOSMCExpr::VK_IMM8, true},
    {"mos16", MOSMCExpr::VK_IMM16, true},
    {"mos8", MOSMCExpr::VK_ADDR8, false},
    {"mos16", MOSMCExpr::VK_ADDR16, false},
    {"mos16lo", MOSMCExpr::VK_ADDR16_LO, false},
    {"mos16hi", MOSMCExpr::VK_ADDR16_HI, false},
    {"mos24", MOSMCExpr::VK_ADDR24, false},
    {"mos24bank", MOSMCExpr::VK_ADDR24_BANK, false},
    {"mos24segment", MOSMCExpr::VK_ADDR24_SEGMENT, false},
    {"mos24segmentlo", MOSMCExpr::VK_ADDR24_SEGMENT_LO, false},
    {"mos24segmenthi", MOSMCExpr::VK_ADDR24_SEGMENT_HI, false},
    {"mos13", MOSMCExpr::VK_ADDR13, false},
};

llvm::ArrayRef<ModifierEntry> modifierNames() { return ModifierNames; }

} // namespace MOS
} // namespace llvm
