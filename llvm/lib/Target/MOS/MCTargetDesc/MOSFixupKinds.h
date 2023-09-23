//===-- MOSFixupKinds.h - MOS Specific Fixup Entries ------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_FIXUP_KINDS_H
#define LLVM_MOS_FIXUP_KINDS_H

#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCFixupKindInfo.h"

namespace llvm {
namespace MOS {

/// The set of supported fixups.
///
/// Although most of the current fixup types reflect a unique relocation
/// one can have multiple fixup types for a given relocation and thus need
/// to be uniquely named.
///
/// \note This table *must* be in the same order of
///       MCFixupKindInfo Infos[MOS::NumTargetFixupKinds]
///       in `MOSFixupKinds.cpp`.
enum Fixups {
  Imm8 = FirstTargetFixupKind, // An 8-bit immediate value.
  Imm16,                       // A 16-bit immediate value.
  Addr8,                       // An 8-bit zero page address.
  Addr16,                      // A 16-bit address.
  Addr16_Low,                  // The low byte of a 16-bit address.
  Addr16_High,                 // The high byte of a 16-bit address.
  Addr24,                      // A 24-bit 65816 address.
  Addr24_Bank,                 // The bank byte of a 24-bit address.
  Addr24_Segment,              // The segment 16-bits of a 24-bit address.
  Addr24_Segment_Low,  // The low 8 bits of the segment of a 24-bit address.
  Addr24_Segment_High, // The high 8 bits of the segment of a 24-bit address.
  Addr13,              // A 13-bit SPC700 address.
  PCRel8,              // An 8-bit PC relative value.
  PCRel16,             // An 16-bit PC relative value.
  AddrAsciz,           // Address encoded as a decimal ASCII string.
  LastTargetFixupKind,
  NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
};

namespace fixups {} // end of namespace fixups
} // namespace MOS

class MOSFixupKinds {
public:
  const static MCFixupKindInfo &getFixupKindInfo(const MOS::Fixups Kind,
                                                 const MCAsmBackend *Alternative);
};
} // namespace llvm

#endif // LLVM_MOS_FIXUP_KINDS_H
