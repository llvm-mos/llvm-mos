//===-- MOSCallGraphUtils.h - MOS Call Graph Utilities ----------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file declares utilities for temporarily modifying the semantics of the
/// CallGraph to enforce invariants needed for whole-program static stack and
/// zero page analyses.
///
//===----------------------------------------------------------------------===//


#ifndef LLVM_LIB_TARGET_MOS_MOSCALLGRAPHUTILS_H
#define LLVM_LIB_TARGET_MOS_MOSCALLGRAPHUTILS_H

#include "llvm/ADT/StringRef.h"

namespace llvm {

class CallGraph;
class Function;
class MachineModuleInfo;
class Module;

namespace mos {

// Returns the function that an symbol reference will ultimately resolve to,
// looking through aliases and pointer casts.
Function *getSymbolFunction(Module &M, StringRef Name);

// Collect libcalls and added edges for them to the call graph.
//
// The call graph is constructed from the IR alone, so it does not consider
// calls that only exist in generated code. This routine corrects this.
void addLibcallEdges(CallGraph &CG, const MachineModuleInfo &MMI);

// Nodes that call external nodes may transitively call any external funciton
// except interrupt handlers, so add edges to record this.
void addExternalEdges(CallGraph &CG);

} // namespace mos
} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSCALLGRAPHUTILS_H
