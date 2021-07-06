//===-- MOSCallingConv.td - Calling Conventions MOS --------*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the MOS architecture.
//
//===----------------------------------------------------------------------===//
//
// The C calling convention is mostly described inline. There are a few
// additional points worth mentioning:
//
// - Aggregate types (structs, arrays, etc.) are passed by pointer. The pointer
//   is managed entirely by the caller, and may or may not be on the soft stack.
//   The callee is free to write to the memory; the caller must consider the
//   memory overwritten by the call. This is handled directly by Clang; LLVM
//   itself should never see aggregates.
//
// - Aggregate types are returned by a pointer passed as an implicit first
//   argument. The resulting function returns void. This is handled directly by
//   Clang; LLVM itself should never see aggregates.
//
// - RS2 and RS4 (and subregisters) are callee-saved. All other Imag8 locations,
//   registers, and flags are caller-saved. The gap between the callee-saved
//   registers balances beteen caller- and callee-saved registers if very little
//   of the zero page is available. It's not use having more than two, since at
//   most 4 bytes of hard stack is allowed per frame, and spilling zero page
//   registers to the soft stack isn't worthwhile.
//
// - Variable arguments (those within the ellipses of the argument list) are
//   passed through the stack. Named arguments before the variable arguments are
//   passed as usual: first in registers, then stack. Note that the variable
//   argument and regular calling convention differ; thus, variable argument
//   functions must only be called if prototyped. The C standard requires this,
//   but many platforms do not; their variable argument and regular calling
//   conventions are identical. A notable exception is Apple ARM64.
//
// - Values may be returned on the soft stack if insufficiently many registers
//   are available. Callers must reserve sufficient space for this as they do
//   for arguments. The space reserved for arguments may overlap freely with the
//   space used for return values; thus only enough space for the larger of the
//   two need be allocated.
//
// For insight into the design of performant calling conventions, see the
// following work by Davidson and Whalley. By their convention, this plaftorm
// uses the "smarter hybrid" method, since LLVM performs both shrink wrapping
// and caller save-restore placement optimizations, while using both
// callee-saved and caller-saved registers when appropriate.
//
// Methods for Saving and Restoring Register Values across Function Calls
// Software--Practice and Experience Vol 21(2), 149-165 (February 1991)
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.14.4669&rep=rep1&type=pdf
//
//===----------------------------------------------------------------------===//

let Entry = 1 in {
def CC_MOS : CallingConv<[
  // Pointers are preferentially assigned to imaginary registers so indirect
  // addressing modes work without additional copying. If there aren't any
  // remaining, they'll fall through and be split to 8-bit values to be
  // assigned separately.
  //
  // RS0 is skipped since it's the stack pointer. RS2 and RS4 are skipped
  // since they're callee-saved.
  CCIfPtr<CCAssignToReg<[
    RS1, RS3, RS5, RS6, RS7, RS8, RS9, RS10, RS11, RS12, RS13, RS14, RS15,
    RS16, RS17, RS18, RS19, RS20, RS21, RS22, RS23, RS24, RS25, RS26, RS27,
    RS28, RS29, RS30, RS31, RS32, RS33, RS34, RS35, RS36, RS37, RS38, RS39,
    RS40, RS41, RS42, RS43, RS44, RS45, RS46, RS47, RS48, RS49, RS50, RS51,
    RS52, RS53, RS54, RS55, RS56, RS57, RS58, RS59, RS60, RS61, RS62, RS63,
    RS64, RS65, RS66, RS67, RS68, RS69, RS70, RS71, RS72, RS73, RS74, RS75,
    RS76, RS77, RS78, RS79, RS80, RS81, RS82, RS83, RS84, RS85, RS86, RS87,
    RS88, RS89, RS90, RS91, RS92, RS93, RS94, RS95, RS96, RS97, RS98, RS99,
    RS100, RS101, RS102, RS103, RS104, RS105, RS106, RS107, RS108, RS109,
    RS110, RS111, RS112, RS113, RS114, RS115, RS116, RS117, RS118, RS119,
    RS120, RS121, RS122, RS123, RS124, RS125, RS126, RS127
  ]>>,

  // 8-bit values are assigned to A, then X, then the available 8-bit imaginary
  // registers. Y is not used for arguments (but is still caller-save); it's
  // reserved to help the compiler shuffle values into and out of the
  // appropriate registers.
  //
  // RC0 and RC1 are skipped since they're the stack pointer. RC4, RC5, RC8, and
  // RC9 are skipped because they're callee-saved.
  CCIfType<[i8], CCAssignToReg<[
    A, X, RC2, RC3, RC6, RC7, RC10, RC11, RC12, RC13, RC14, RC15, RC16,
    RC17, RC18, RC19, RC20, RC21, RC22, RC23, RC24, RC25, RC26, RC27, RC28,
    RC29, RC30, RC31, RC32, RC33, RC34, RC35, RC36, RC37, RC38, RC39, RC40,
    RC41, RC42, RC43, RC44, RC45, RC46, RC47, RC48, RC49, RC50, RC51, RC52,
    RC53, RC54, RC55, RC56, RC57, RC58, RC59, RC60, RC61, RC62, RC63, RC64,
    RC65, RC66, RC67, RC68, RC69, RC70, RC71, RC72, RC73, RC74, RC75, RC76,
    RC77, RC78, RC79, RC80, RC81, RC82, RC83, RC84, RC85, RC86, RC87, RC88,
    RC89, RC90, RC91, RC92, RC93, RC94, RC95, RC96, RC97, RC98, RC99,
    RC100, RC101, RC102, RC103, RC104, RC105, RC106, RC107, RC108, RC109,
    RC110, RC111, RC112, RC113, RC114, RC115, RC116, RC117, RC118, RC119,
    RC120, RC121, RC122, RC123, RC124, RC125, RC126, RC127, RC128, RC129,
    RC130, RC131, RC132, RC133, RC134, RC135, RC136, RC137, RC138, RC139,
    RC140, RC141, RC142, RC143, RC144, RC145, RC146, RC147, RC148, RC149,
    RC150, RC151, RC152, RC153, RC154, RC155, RC156, RC157, RC158, RC159,
    RC160, RC161, RC162, RC163, RC164, RC165, RC166, RC167, RC168, RC169,
    RC170, RC171, RC172, RC173, RC174, RC175, RC176, RC177, RC178, RC179,
    RC180, RC181, RC182, RC183, RC184, RC185, RC186, RC187, RC188, RC189,
    RC190, RC191, RC192, RC193, RC194, RC195, RC196, RC197, RC198, RC199,
    RC200, RC201, RC202, RC203, RC204, RC205, RC206, RC207, RC208, RC209,
    RC210, RC211, RC212, RC213, RC214, RC215, RC216, RC217, RC218, RC219,
    RC220, RC221, RC222, RC223, RC224, RC225, RC226, RC227, RC228, RC229,
    RC230, RC231, RC232, RC233, RC234, RC235, RC236, RC237, RC238, RC239,
    RC240, RC241, RC242, RC243, RC244, RC245, RC246, RC247, RC248, RC249,
    RC250, RC251, RC252, RC253, RC254, RC255
  ]>>,

  // Any 8-bit values that wouldn't fit anywhere else are passed directly on
  // the stack.
  CCIfType<[i8], CCAssignToStack<1, 1>>,

  // Larger types are here implicitily split to bytes, then they repeat the
  // whole convention.
]>;

// Calling convention for the variable section of a variadic function call.
// Named arguments in such functions still use the above calling convention.
def CC_MOS_VarArgs : CallingConv<[
  // Assign all 8-bit values directly to the stack. This allows variable arguments
  CCIfType<[i8], CCAssignToStack<1, 1>>,

  // Larger types are here implicitily split to bytes, then they repeat the
  // whole convention.
]>;
}

// All registers are caller saved except RS2 and RS4. These are represented
// implicitly by their subregisters, since the subregisters are what must be
// saved and restored.
def MOS_CSR : CalleeSavedRegs<(add RC4, RC5, RC8, RC9)>;
