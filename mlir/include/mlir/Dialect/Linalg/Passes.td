//===-- Passes.td - Linalg pass definition file ------------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef MLIR_DIALECT_LINALG_PASSES
#define MLIR_DIALECT_LINALG_PASSES

include "mlir/Pass/PassBase.td"

def ConvertElementwiseToLinalgPass : Pass<"convert-elementwise-to-linalg", ""> {
  let summary = "Convert ElementwiseMappable ops to linalg";
  let description = [{
    Convert ops with the `ElementwiseMappable` trait to linalg parallel loops.

    This pass only converts ops that operate on ranked tensors. It can be
    run on op which contains linalg ops (most commonly a
    FunctionOpInterface op).
  }];
  let dependentDialects = ["linalg::LinalgDialect", "memref::MemRefDialect"];
}

def ConvertLinalgToAffineLoopsPass : Pass<"convert-linalg-to-affine-loops"> {
  let summary = "Lower the operations from the linalg dialect into affine "
                "loops";
  let dependentDialects = [
    "affine::AffineDialect", "linalg::LinalgDialect", "memref::MemRefDialect"];
}

def ConvertLinalgToLoopsPass : Pass<"convert-linalg-to-loops"> {
  let summary = "Lower the operations from the linalg dialect into loops";
  let description = [{
    Lowers the `linalg` ops to loop nests using `scf.for`.

    Pre-condition: the operands used by the `linalg` ops have buffer semantics,
    i.e., tensor operands and results must be converted to memrefs via
    bufferization.
  }];
  let dependentDialects = [
    "linalg::LinalgDialect",
    "scf::SCFDialect",
    "affine::AffineDialect"
  ];
}

def ConvertLinalgToParallelLoopsPass
    : Pass<"convert-linalg-to-parallel-loops"> {
  let summary = "Lower the operations from the linalg dialect into parallel "
                "loops";
  let dependentDialects = [
    "affine::AffineDialect",
    "linalg::LinalgDialect",
    "memref::MemRefDialect",
    "scf::SCFDialect"
  ];
}

def LinalgFoldUnitExtentDimsPass : Pass<"linalg-fold-unit-extent-dims", ""> {
  let summary = "Remove unit-extent dimension in Linalg ops on tensors";
  let options = [
    Option<"useRankReducingSlices", "use-rank-reducing-slices", "bool",
           /*default=*/"false",
           "Generate rank-reducing slices instead of reassociative reshapes">
  ];
  let dependentDialects = [
    "linalg::LinalgDialect", "affine::AffineDialect", "memref::MemRefDialect"
  ];
}

def LinalgElementwiseOpFusionPass : Pass<"linalg-fuse-elementwise-ops"> {
  let summary = "Fuse elementwise operations on tensors";
  let dependentDialects = [
    "affine::AffineDialect", "linalg::LinalgDialect", "memref::MemRefDialect"
  ];
}

def LinalgNamedOpConversionPass: Pass<"linalg-named-op-conversion"> {
  let summary = "Convert from one named linalg op to another.";
  let dependentDialects = ["linalg::LinalgDialect", "tensor::TensorDialect"];
}

def LinalgInlineScalarOperandsPass : Pass<"linalg-inline-scalar-operands"> {
  let summary = "Inline scalar operands into linalg generic ops";
  let dependentDialects = [
    "linalg::LinalgDialect"
  ];
}

def LinalgBufferizePass : Pass<"linalg-bufferize"> {
  let summary = "Bufferize the linalg dialect";
  let dependentDialects = [
    "affine::AffineDialect",
    "bufferization::BufferizationDialect",
    "linalg::LinalgDialect",
    "memref::MemRefDialect",
  ];
}

def LinalgGeneralizeNamedOpsPass : Pass<"linalg-generalize-named-ops"> {
  let summary = "Convert named ops into generic ops";
  let dependentDialects = ["linalg::LinalgDialect"];
}

def LinalgDetensorizePass : InterfacePass<"linalg-detensorize", "FunctionOpInterface"> {
  let summary = "Detensorize linalg ops";
  let dependentDialects = [];

  let description = [{
    Detensoring is the process through which a tensor value is converted to one
    or potentially more primitive value(s). During this process, operations with
    such detensored operands are also converted to an equivalent form that works
    on primitives.

    The detensoring process is driven by linalg-on-tensor ops. In particular, a
    linalg-on-tensor op is checked to see whether *all* its operands can be
    detensored. If so, those operands are converted to their primitive
    counterparts and the linalg op is replaced by an equivalent op that takes
    those new primitive values as operands. Therefore, detensoring an op can be
    divided into 2 main logical phases:

    1. Detect/match an op that can be detensored.
    2. Detensor the operands of the op and replace it with a primitive
       equivalent.

    In addition to detensoring individual ops, this pass detensors internal
    control flow inside a function. All blocks except for the entry block are
    detensored by converting their arguments whenever possible.

    This can be run on any FunctionOpInterface op and must not be
    run on others. This is because it performs specific legalization of the
    blocks that make up the body, which it assumes has is a FunctionOpInterface.
  }];
  let options = [
    Option<"aggressiveMode", "aggressive-mode", "bool", /*default=*/"false",
           "Detensorize all ops that qualify for detensoring along with branch"
           " operands and basic-block arguments.">

  ];
}

#endif // MLIR_DIALECT_LINALG_PASSES
