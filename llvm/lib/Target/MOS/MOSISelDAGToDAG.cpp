//===-- MOSISelDAGToDAG.cpp - A dag to dag inst selector for MOS ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines an instruction selector for the MOS target.
//
//===----------------------------------------------------------------------===//

#include "MOS.h"
#include "MOSTargetMachine.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "mos-isel"

namespace llvm {

/// Lowers LLVM IR (in DAG form) to MOS MC instructions (in DAG form).
class MOSDAGToDAGISel : public SelectionDAGISel {
public:
  MOSDAGToDAGISel(MOSTargetMachine &TM, CodeGenOpt::Level OptLevel)
      : SelectionDAGISel(TM, OptLevel) {}

  StringRef getPassName() const override {
    return "MOS DAG->DAG Instruction Selection";
  }

  virtual void Select(SDNode *N) override {} 


#include "MOSGenDAGISel.inc"
};

FunctionPass *createMOSISelDag(MOSTargetMachine &TM,
                               CodeGenOpt::Level OptLevel) {
  return new MOSDAGToDAGISel(TM, OptLevel);
}

} // end of namespace llvm

