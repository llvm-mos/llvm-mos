def label : Operand<OtherVT>;

// Pseudoinstructions lower to a sequence of real 6502 instructions. Which
// sequence of instructions they expand to usually depends on the register
// allocation of their operands.
//
// Unfortunately, it's often the case that the registers internally
// used/clobbered by these instruction sequences depends on the register
// allocation as well. Were these sequences emitted naively, they could clobber
// values used by future instructions.
//
// Broadly, there are two different locations where these pseudos can be emitted:
// before and during/after register allocation. Pseudos that can only be emitted
// before register allocation should give a conservative estimate of the real
// register uses of their expansions. Then the register allocator can later use
// this information to avoid unneccessary conflicts. For pseudos that can be
// emitted during/after register allocation, it's too late for the register
// allocator to do this, so pseudos must instead report that they affect
// nothing.
//
// In either case, once the pseudo instructions are to be lowered to a real
// sequence of instructions, register allocation has already occurred. The
// emitted instruction sequence is derived from the register allocation of the
// pseudo's operand. The sequence is then wrapped in appropriate save/reload
// code to make its register use/def profile match the one advertized by the
// pseudo definition in this file. It only need to match in an "as-if" fashion:
// any locations that are not live across the pseudo may be freely used without
// save/restore.

class Pseudo : Instruction {
  let Namespace = "MOS";

  dag OutOperandList = (outs);
  dag InOperandList = (ins);

  let hasSideEffects = false;
  let mayLoad = false;
  let mayStore = false;
  let isPseudo = true;
  let isCodeGenOnly = true;
}

// Frame Pseudoinstructions

def LDstk : Pseudo {
  dag OutOperandList = (outs Anyc:$dst);
  dag InOperandList = (ins Imag16:$base, i16imm:$offset);

  let Defs = [NZ];

  let mayLoad = true;
}

def STstk : Pseudo {
  dag InOperandList = (ins Anyc:$src, Imag16:$base, i16imm:$fi);

  let Defs = [NZ];

  let mayStore = true;
}

// Produces the low byte of the address of the given frame index. Produces a
// carry required by the high byte calculation.
def AddrLostk : Pseudo {
  dag OutOperandList = (outs Anyi8:$dst, Cc:$carry);
  dag InOperandList = (ins Imag16:$base, i16imm:$offset);

  let Defs = [NZ];

  let isReMaterializable = true;
}

// Produces the high byte of the address of the given frame index. Accepts the
// carry produced by the low byte calculation.
def AddrHistk : Pseudo {
  dag OutOperandList = (outs Anyi8:$dst);
  dag InOperandList = (ins Imag16:$base, i16imm:$offset, Cc:$carry);

  let Defs = [C, NZ];

  let isReMaterializable = true;
}

// Soft stack

// These are the call-frame setup and teardown pseudoinstructions for the MOS
// target. They form a signalling mechanism used to pass the size of outgoing
// call frames all the way from calling-convention lowering (in IRTranslator)
// through to the stack frame prolog-epilog insertion. The later examines these
// pseudoinstructions to find the largest size that an outgoing call frame can
// be, then preallocates that much space to the stack frame if possible. This
// allows these pseudoinstructions to be removed wholesale in most cases, since
// we can instead guarantee that the bottom-most bytes of the stack will always
// have enough free space to set up the arguments of any outgoing call. This
// technique is called call frame reservation.

def ADJCALLSTACKDOWN : Pseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);

  let Defs = [RS0];
  let Uses = [RS0];
}

def ADJCALLSTACKUP : Pseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);

  let Defs = [RS0];
  let Uses = [RS0];
}

// Miscellaneous pseudoinstructions

def LDabs_offset : Pseudo {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i16imm:$addr, i16imm:$offset);

  let Defs = [NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

def STabs_offset : Pseudo {
  dag OutOperandList = (outs);
  dag InOperandList = (ins GPR:$src, i16imm:$addr, i16imm:$offset);

  let mayStore = true;
}

// $dst = *(addr + (unsigned char)idx);
// Sets NZ to undefined value, since expansion may clobber.
def LDidx : Pseudo {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i16imm:$addr, XY:$idx);

  let Defs = [NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

// Logical Instructions

// These pseudoinstruction correspond to logical instruction set of the 6502.
// Instructions with identical effects except the placement of their operands
// are joined together into instructions with register classes directly
// representing those constraints.
//
// This may take more than one instruction. For example, the register transfer
// instructions TAX, TXA, TAY, and TYA are split into TA_ and T_A, since
// register classes are too crude of an instrument to specify that either the
// source or target must be A. An XY register class fills in the remaining
// freedom that the _ may be fulfilled with either X or Y.
//
// Doing things this way allows the code generator to pretend that the 6502 is
// much more regular than it really is, right up until AsmPrinter brings the
// instructions down to their real, bitwise representation. This shields most of
// LLVM from the rigors of the real, highly irregular 6502 instruction set. For
// example, it brings the number of implicit physical registers down to the
// level typical of a LLVM backend. Codegen passes can handle implicit physical
// registers every once in a while, but they have a hard time dealing with every
// single instruction using them for nearly every operand. Most of the issues are
// probably just oversights, but there's a lot to be gained by tightly limiting
// "weirdness" as much as possible.

def ADCimm : Pseudo, PseudoInstExpansion<(ADC_Immediate imm8:$r)> {
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  dag InOperandList = (ins Ac:$l, i8imm:$r, Cc:$carryin);
  let Constraints = "$dst = $l, $carryout = $carryin";

  let Defs = [NZ];

  let isAdd = true;
  let isReMaterializable = true;
}

def ADCimag8 : Pseudo, PseudoInstExpansion<(ADC_ZeroPage addr8:$r)> {
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  dag InOperandList = (ins Ac:$l, Imag8:$r, Cc:$carryin);
  // Don't give the carries a tied constraint, since having two tied operands
  // inhibits commutation of $l and $r.
  let Constraints = "$dst = $l";

  let Defs = [NZ];

  let isAdd = true;
  let isCommutable = true;
  let isReMaterializable = true;
}

def ASL : Pseudo {
  dag OutOperandList = (outs AImag8:$dst, Cc:$carry);
  dag InOperandList = (ins AImag8:$src);
  let Constraints = "$dst = $src";

  let Defs = [NZ];

  let isReMaterializable = true;
}

// Branches to $tgt if $flag = $val.
def BR : Pseudo {
  dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$val);

  let isBranch = true;
  let isTerminator = true;
}

def CMPimm : Pseudo {
  dag InOperandList = (ins GPR:$l, i8imm:$r);

  let Defs = [C, NZ];

  let isCompare = true;
  let isReMaterializable = true;

  // Keep CMPimm adjacent to the branch that uses it.
  // This prevents NZ from ever being live outside of the terminators.
  let isTerminator = true;
}

def IN_ : Pseudo {
  dag OutOperandList = (outs XY:$dst);
  dag InOperandList = (ins XY:$src);
  let Constraints = "$src = $dst";

  let Defs = [NZ];

  let Pattern = [(set XY:$dst, (add XY:$src, 1))];


  let isAdd = true;
  let isReMaterializable = true;
}

def JMP : Pseudo, PseudoInstExpansion<(JMP_Absolute addr16:$tgt)> {
  dag InOperandList = (ins label:$tgt);

  let Pattern = [(br bb:$tgt)];

  let isBarrier = true;
  let isBranch = true;
  let isTerminator = true;
}

def JSR : Pseudo, PseudoInstExpansion<(JSR_Absolute addr16:$tgt)> {
  dag InOperandList = (ins label:$tgt);

  let isCall = true;
}

def LDCimm : Pseudo {
  dag OutOperandList = (outs Cc:$dst);
  dag InOperandList = (ins i1imm:$val);

  let isReMaterializable = true;
  let isAsCheapAsAMove = true;
}

def LDimm : Pseudo {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i8imm:$val);

  let Defs = [NZ];

  let Pattern = [(set GPR:$dst, imm:$val)];

  let isAsCheapAsAMove = true;
  let isMoveImm = true;
  let isReMaterializable = true;
}

def LDimag8 : Pseudo {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins Imag8:$src);

  let Defs = [NZ];

  let isReMaterializable = true;
}

def LDabs : Pseudo {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i16imm:$src);

  let Defs = [NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDAidx : Pseudo {
  dag InOperandList = (ins i16imm:$addr, XY:$idx);

  let Defs = [A, NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDXidx : Pseudo, PseudoInstExpansion<(LDX_AbsoluteY addr16:$addr)> {
  dag InOperandList = (ins i16imm:$addr);

  let Defs = [X, NZ];
  let Uses = [Y];

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDYidx : Pseudo, PseudoInstExpansion<(LDY_AbsoluteX addr16:$addr)> {
  dag InOperandList = (ins i16imm:$addr);

  let Defs = [Y, NZ];
  let Uses = [X];

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDyindir : Pseudo, PseudoInstExpansion<(LDA_IndirectIndexed addr8:$addr)> {
  dag OutOperandList = (outs Ac:$dst);
  dag InOperandList = (ins Imag16:$addr, Yc:$offset);

  let Defs = [NZ];

  let mayLoad = true;
}

def PHA : Pseudo, PseudoInstExpansion<(PHA_Implied)> {
  let Uses = [A];

  let mayStore = true;
}

def PLA : Pseudo, PseudoInstExpansion<(PLA_Implied)> {
  let Defs = [A, NZ];

  let mayLoad = true;
}

def PHP : Pseudo, PseudoInstExpansion<(PHP_Implied)> {
  let Uses = [P];

  let mayStore = true;
}

def PLP : Pseudo, PseudoInstExpansion<(PLP_Implied)> {
  let Defs = [P];

  let mayLoad = true;
}

def ROL : Pseudo {
  dag OutOperandList = (outs AImag8:$dst, Cc:$carryout);
  dag InOperandList = (ins AImag8:$src, Cc:$carryin);
  let Constraints = "$dst = $src, $carryout = $carryin";

  let Defs = [NZ];

  let isReMaterializable = true;
}

def RTS : Pseudo, PseudoInstExpansion<(RTS_Implied)> {
  let isBarrier = true;
  let isReturn = true;
  let isTerminator = true;
}

def SBCimm : Pseudo, PseudoInstExpansion<(SBC_Immediate imm8:$r)> {
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  dag InOperandList = (ins Ac:$l, i8imm:$r, Cc:$carryin);
  let Constraints = "$dst = $l, $carryout = $carryin";

  let Defs = [NZ];

  let isReMaterializable = true;
}

def SBCimag8 : Pseudo, PseudoInstExpansion<(SBC_ZeroPage addr8:$r)> {
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  dag InOperandList = (ins Ac:$l, Imag8:$r, Cc:$carryin);
  // Don't give the carries a tied constraint, since having two tied operands
  // inhibits commutation of $l and $r.
  let Constraints = "$dst = $l";

  let Defs = [NZ];

  let isReMaterializable = true;
}

def STabs : Pseudo {
  dag InOperandList = (ins GPR:$src, i16imm:$dst);

  let mayStore = true;
}

def STidx : Pseudo {
  dag InOperandList = (ins Ac:$src, i16imm:$addr, XY:$idx);

  let mayStore = true;
}

def STyindir : Pseudo, PseudoInstExpansion<(STA_IndirectIndexed addr8:$addr)> {
  dag InOperandList = (ins Ac:$src, Imag16:$addr, Yc:$offset);

  let mayStore = true;
}

def STimag8 : Pseudo {
  dag OutOperandList = (outs Imag8:$dst);
  dag InOperandList = (ins GPR:$src);
}

def TA_ : Pseudo {
  dag OutOperandList = (outs XY:$dst);

  let Defs = [NZ];
  let Uses = [A];

  let isAsCheapAsAMove = true;
  let isReMaterializable = true;
}

def T_A : Pseudo {
  dag InOperandList = (ins XY:$src);

  let Defs = [A, NZ];

  let isAsCheapAsAMove = true;
  let isReMaterializable = true;
}

def TSX : Pseudo, PseudoInstExpansion<(TSX_Implied)> {
  let Defs = [X, NZ];

  let isAsCheapAsAMove = true;
  let isReMaterializable = true;
}