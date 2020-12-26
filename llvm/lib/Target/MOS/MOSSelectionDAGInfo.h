//===-- MOSSelectionDAGInfo.h - MOS SelectionDAG Info -----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS subclass for SelectionDAGTargetInfo.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_SELECTION_DAG_INFO_H
#define LLVM_MOS_SELECTION_DAG_INFO_H

#include "llvm/CodeGen/SelectionDAGTargetInfo.h"

namespace llvm {

/// Holds information about the MOS instruction selection DAG.
class MOSSelectionDAGInfo : public SelectionDAGTargetInfo {
public:
};

} // end namespace llvm

#endif // LLVM_MOS_SELECTION_DAG_INFO_H
