#pragma once

#include "llvm/ADT/ArrayRef.h"
#include "MCTargetDesc/MOSMCExpr.h"

namespace llvm {
namespace MOS {

  struct ModifierEntry {
    const char *const Spelling;
    MOSMCExpr::VariantKind VariantKind;
    bool ImmediateOnly = false;
  };

  typedef struct ModifierEntry ModifierEntry;

  llvm::ArrayRef<ModifierEntry> modifierNames();

}
}
