#pragma once

#include "MCTargetDesc/MOSMCExpr.h"
#include "llvm/ADT/ArrayRef.h"

namespace llvm {
namespace MOS {

struct ModifierEntry {
  const char *const Spelling;
  MOSMCExpr::VariantKind VariantKind;
  bool ImmediateOnly = false;
};

typedef struct ModifierEntry ModifierEntry;

llvm::ArrayRef<ModifierEntry> modifierNames();

} // namespace MOS
} // namespace llvm
