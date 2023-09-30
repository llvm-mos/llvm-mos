//===-- MOSFixupKinds.cpp - MOS fixup kinds  ------------------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the MOSAsmBackend class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MOSFixupKinds.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCFixupKindInfo.h"

namespace llvm {
const MCFixupKindInfo &
MOSFixupKinds::getFixupKindInfo(const MOS::Fixups Kind,
                                const MCAsmBackend *Alternative) {
  const static MCFixupKindInfo Infos[MOS::NumTargetFixupKinds] = {
      // This table *must* be in same the order of fixup_* kinds in
      // MOSFixupKinds.h.
      //
      // name, offset, bits, flags
      {"Imm8", 0, 8, 0},            // An 8-bit immediate value.
      {"Imm16", 0, 16, 0},          // An 16-bit immediate value.
      {"Addr8", 0, 8, 0},           // An 8-bit zero page address.
      {"Addr16", 0, 16, 0},         // A 16-bit address.
      {"Addr16_Low", 0, 8, 0},      // The low byte of a 16-bit address.
      {"Addr16_High", 0, 8, 0},     // The high byte of a 16-bit address.
      {"Addr24", 0, 24, 0},         // A 24-bit 65816 address.
      {"Addr24_Bank", 0, 8, 0},     // The bank byte of a 24-bit address.
      {"Addr24_Segment", 0, 16, 0}, // The segment 16-bits of a 24-byte address.
      {"Addr24_Segment_Low", 0, 8,
       0}, // The low byte of the segment of a 24-bit addr
      {"Addr24_Segment_High", 0, 8,
       0}, // The high byte of the segment of a 24-bit addr
      {"Addr13", 0, 13, 0},         // A 13-bit address.
      // PCRel* is pc-relative and requires target specific handling
      {"PCRel8", 0, 8,
       MCFixupKindInfo::FKF_IsPCRel | MCFixupKindInfo::FKF_IsTarget},
      {"PCRel16", 0, 16,
       MCFixupKindInfo::FKF_IsPCRel | MCFixupKindInfo::FKF_IsTarget}};
  if (Kind < static_cast<MOS::Fixups>(FirstTargetFixupKind)) {
    assert(Alternative &&
           "Alternative MOS backend expected, but none was given!");
    return Alternative->getFixupKindInfo(static_cast<MCFixupKind>(Kind));
  }
  assert(unsigned(Kind - FirstTargetFixupKind) <
             MOS::Fixups::NumTargetFixupKinds &&
         "Invalid kind!");
  return Infos[Kind - FirstTargetFixupKind];
}

} // namespace llvm
