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

  let Uses = [];
  let Defs = [NZ];

  let hasSideEffects = 0;
  let mayLoad = 0;
  let mayStore = 0;
  let isPseudo = true;
  let isCodeGenOnly = true;
}

// Frame Pseudoinstructions

def LDstk : Pseudo {
  dag InOperandList = (ins Stack:$base, i16imm:$offset);
  dag OutOperandList = (outs Anyc:$dst);

  let Defs = [];

  let mayLoad = true;
}

def STstk : Pseudo {
  dag InOperandList = (ins Anyc:$src, Stack:$base, i16imm:$fi);
  dag OutOperandList = (outs);

  let Defs = [];

  let mayStore = true;
}

// Produces the low byte of the address of the given frame index. Produces a
// carry required by the high byte calculation.
def AddrLostk : Pseudo {
  dag InOperandList = (ins Stack:$base, i16imm:$offset);
  dag OutOperandList = (outs Anyi8:$dst, Cc:$carry);

  let Defs = [NZ];

  let isReMaterializable = true;
}

// Produces the high byte of the address of the given frame index. Accepts the
// carry produced by the low byte calculation.
def AddrHistk : Pseudo {
  dag InOperandList = (ins Stack:$base, i16imm:$offset, Cc:$carry);
  dag OutOperandList = (outs Anyi8:$dst);

  let Defs = [C, NZ];

  let isReMaterializable = true;
}

// Produces the address of the given frame index. Doesn't allow instructions
// to be inserted between calculation of the low and high byte, but in exchange,
// the instruction can be moved around and rematerialized as a whole.
def Addrstk : Pseudo {
  dag InOperandList = (ins Stack:$base, i16imm:$offset);
  dag OutOperandList = (outs ZP_PTR:$dst);

  let Defs = [NZ];

  let isReMaterializable = true;
}

// Hard Stack

def Pull : Pseudo {
  dag InOperandList = (ins);
  dag OutOperandList = (outs Anyi8:$dst);

  let Defs = [];

  let mayLoad = true;
}

// Same as pull, but preserving the value in the A register if live.
def PullPreserve : Pseudo {
  dag InOperandList = (ins);
  dag OutOperandList = (outs);

  let Defs = [];

  let mayLoad = true;
}

def Push : Pseudo {
  dag InOperandList = (ins Anyi8:$src);
  dag OutOperandList = (outs);

  let Defs = [];

  let mayStore = true;
}

// Soft stack

// Add $bytes bytes to SP, saving the result back in SP. A negative argument
// decreases SP.
def IncSP : Pseudo {
  dag InOperandList = (ins i16imm:$bytes);
  dag OutOperandList = (outs);

  let Uses = [RS0];
  let Defs = [RS0];
}

def ADJCALLSTACKDOWN : Pseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);
  dag OutOperandList = (outs);

  let Uses = [RS0];
  let Defs = [RS0];
}

def ADJCALLSTACKUP : Pseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);
  dag OutOperandList = (outs);

  let Uses = [RS0];
  let Defs = [RS0];
}

// Miscellaneous pseudoinstructions

// $dst = *(addr + (unsigned char)idx);
// Sets NZ to undefined value, since expansion may clobber.
def LDidx : Pseudo {
  dag InOperandList = (ins i16imm:$addr, XY:$idx);
  dag OutOperandList = (outs GPR:$dst);

  let isReMaterializable = true;
  let mayLoad = true;
}

// Version of LDimm that promises not to overwrite NZ. This allows
// rematerializing LDimm with a possible PHP/PLP instead of spilling/loading the
// constant on the stack. The spill is very expensive, and would need to
// PHP/PLP anyways, so rematerialization is strictly better in this case.
def LDimm_preserve : Pseudo {
  dag InOperandList = (ins i8imm:$val);
  dag OutOperandList = (outs GPR:$dst);

  let Defs = [];

  let isMoveImm = true;
  let isReMaterializable = true;
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
  dag InOperandList = (ins Ac:$l, i8imm:$r, Cc:$carryin);
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  let Constraints = "$dst = $l, $carryout = $carryin";

  let Defs = [NZ];

  let isAdd = true;
  let isReMaterializable = true;
}

def ADCzpr : Pseudo, PseudoInstExpansion<(ADC_ZeroPage addr8:$r)> {
  dag InOperandList = (ins Ac:$l, ZP:$r, Cc:$carryin);
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  // Don't give the carries a tied constraint, since having two tied operands
  // inhibits commutation of $l and $r.
  let Constraints = "$dst = $l";

  let Defs = [NZ];

  let isAdd = true;
  let isCommutable = true;
  let isReMaterializable = true;
}

def ASL : Pseudo {
  dag InOperandList = (ins AZP:$src);
  dag OutOperandList = (outs AZP:$dst, Cc:$carry);
  let Constraints = "$dst = $src";

  let Defs = [NZ];

  let isReMaterializable = true;
}

def CMPimm : Pseudo {
  dag InOperandList = (ins GPR:$l, i8imm:$r);
  dag OutOperandList = (outs);

  let isCompare = true;
  let isReMaterializable = true;

  // Keep CMPimm adjacent to the branch that uses it.
  // This prevents NZ from ever being live across an instruction.
  let isTerminator = true;
}

def IN_ : Pseudo {
  dag InOperandList = (ins XY:$src);
  dag OutOperandList = (outs XY:$dst);
  let Constraints = "$src = $dst";

  let Pattern = [(set XY:$dst, (add XY:$src, 1))];

  let isAdd = true;
  let isReMaterializable = true;
}

def LDCimm : Pseudo {
  dag InOperandList = (ins i1imm:$val);
  dag OutOperandList = (outs Cc:$dst);

  let Defs = [];

  let isReMaterializable = true;
  let isAsCheapAsAMove = true;
}

def LDimm : Pseudo {
  dag InOperandList = (ins i8imm:$val);
  dag OutOperandList = (outs GPR:$dst);

  let Pattern = [(set GPR:$dst, imm:$val)];

  let isAsCheapAsAMove = true;
  let isMoveImm = true;
  let isReMaterializable = true;
}

def LDzpr : Pseudo {
  dag InOperandList = (ins ZP:$src);
  dag OutOperandList = (outs GPR:$dst);

  let isReMaterializable = true;
}

def LDabs : Pseudo {
  dag InOperandList = (ins i16imm:$src);
  dag OutOperandList = (outs GPR:$dst);

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDAidx : Pseudo {
  dag InOperandList = (ins i16imm:$addr, XY:$idx);
  dag OutOperandList = (outs);

  let Defs = [A, NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDXidx : Pseudo, PseudoInstExpansion<(LDX_AbsoluteY addr16:$addr)> {
  dag InOperandList = (ins i16imm:$addr);
  dag OutOperandList = (outs);

  let Uses = [Y];
  let Defs = [X, NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDYidx : Pseudo, PseudoInstExpansion<(LDY_AbsoluteX addr16:$addr)> {
  dag InOperandList = (ins i16imm:$addr);
  dag OutOperandList = (outs);

  let Uses = [X];
  let Defs = [Y, NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDyindirr : Pseudo, PseudoInstExpansion<(LDA_IndirectIndexed addr8:$addr)> {
  dag InOperandList = (ins ZP_PTR:$addr, Yc:$offset);
  dag OutOperandList = (outs Ac:$dst);

  let Defs = [NZ];

  let mayLoad = true;
}

def ROL : Pseudo {
  dag InOperandList = (ins AZP:$src, Cc:$carryin);
  dag OutOperandList = (outs AZP:$dst, Cc:$carryout);
  let Constraints = "$dst = $src, $carryout = $carryin";

  let Defs = [NZ];

  let isReMaterializable = true;
}

def SBCimm : Pseudo, PseudoInstExpansion<(SBC_Immediate imm8:$r)> {
  dag InOperandList = (ins Ac:$l, i8imm:$r, Cc:$carryin);
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  let Constraints = "$dst = $l, $carryout = $carryin";

  let Defs = [NZ];

  let isReMaterializable = true;
}

def SBCzpr : Pseudo, PseudoInstExpansion<(SBC_ZeroPage addr8:$r)> {
  dag InOperandList = (ins Ac:$l, ZP:$r, Cc:$carryin);
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  // Don't give the carries a tied constraint, since having two tied operands
  // inhibits commutation of $l and $r.
  let Constraints = "$dst = $l";

  let Defs = [NZ];

  let isReMaterializable = true;
}

def TA_ : Pseudo {
  dag InOperandList = (ins);
  dag OutOperandList = (outs XY:$dst);

  let Uses = [A];

  let isAsCheapAsAMove = true;
  let isReMaterializable = true;
}

def T_A : Pseudo {
  dag InOperandList = (ins XY:$src);
  dag OutOperandList = (outs);

  let Defs = [A, NZ];

  let isAsCheapAsAMove = true;
  let isReMaterializable = true;
}

def TSX : Pseudo, PseudoInstExpansion<(TSX_Implied)> {
  dag InOperandList = (ins);
  dag OutOperandList = (outs);

  let Defs = [X, NZ];

  let isAsCheapAsAMove = true;
  let isReMaterializable = true;
}

def PHA : Pseudo, PseudoInstExpansion<(PHA_Implied)> {
  dag InOperandList = (ins);
  dag OutOperandList = (outs);
  let Uses = [A];
  let Defs = [];

  let mayStore = true;
}

def PLA : Pseudo, PseudoInstExpansion<(PLA_Implied)> {
  dag InOperandList = (ins);
  dag OutOperandList = (outs);
  let Defs = [A, NZ];

  let mayLoad = true;
}

def PHP : Pseudo, PseudoInstExpansion<(PHP_Implied)> {
  dag InOperandList = (ins);
  dag OutOperandList = (outs);
  let Uses = [P];
  let Defs = [];

  let mayStore = true;
}

def PLP : Pseudo, PseudoInstExpansion<(PLP_Implied)> {
  dag InOperandList = (ins);
  dag OutOperandList = (outs);
  let Defs = [P];

  let mayLoad = true;
}

def STzpr : Pseudo {
  dag InOperandList = (ins GPR:$src);
  dag OutOperandList = (outs ZP:$dst);

  let Defs = [];
}

def STabs : Pseudo {
  dag InOperandList = (ins GPR:$src, i16imm:$dst);
  dag OutOperandList = (outs);

  let Defs = [];

  let mayStore = true;
}

def STidx : Pseudo {
  dag InOperandList = (ins Ac:$src, i16imm:$addr, XY:$idx);
  dag OutOperandList = (outs);

  let Defs = [];

  let mayStore = true;
}

def STyindirr : Pseudo, PseudoInstExpansion<(STA_IndirectIndexed addr8:$addr)> {
  dag InOperandList = (ins Ac:$src, ZP_PTR:$addr, Yc:$offset);
  dag OutOperandList = (outs);

  let Defs = [];

  let mayStore = true;
}

// Branches to $tgt if $flag = $val.
def BR : Pseudo {
  dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$val);
  dag OutOperandList = (outs);
  let Defs = [];

  let isBranch = true;
  let isTerminator = true;
}

def JMP : Pseudo, PseudoInstExpansion<(JMP_Absolute addr16:$tgt)> {
  dag InOperandList = (ins label:$tgt);
  dag OutOperandList = (outs);

  let Defs = [];

  let Pattern = [(br bb:$tgt)];

  let isBarrier = true;
  let isBranch = true;
  let isTerminator = true;
}
