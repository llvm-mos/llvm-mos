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

#include "llvm/MC/MCFixup.h"

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
///       in `MOSAsmBackend.cpp`.
enum Fixups {
  /// A 7-bit PC-relative fixup for the family of conditional
  /// branches which take 7-bit targets (BRNE,BRGT,etc).
  fixup_7_pcrel = FirstTargetFixupKind,
  /// A 16-bit address.
  fixup_16,
  /// A 16-bit program memory address.
  fixup_16_pm,
  /// Replaces the 8-bit immediate with another value.
  fixup_ldi,
  LastTargetFixupKind,
  NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
};

namespace fixups {} // end of namespace fixups
} // namespace MOS
} // namespace llvm

#endif // LLVM_MOS_FIXUP_KINDS_H
