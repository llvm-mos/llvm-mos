//===-- MOSInstrInfo.td - MOS Instruction Formats ----------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// MOS Instruction Format Definitions.
//
//===----------------------------------------------------------------------===//

// The base MOS instruction class, from which all instructions descend.
class MOSInst< dag outs, dag ins, string asmstr, list<dag> pattern> : Instruction
{
  let Namespace = "MOS";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = asmstr;
  let Pattern = pattern;

  field bits<32> SoftFail = 0;
  let DecoderNamespace = "MOS";
}

class ImmediateAsmOperand<string name>
  : AsmOperandClass {
  let Name = name;
  let RenderMethod = "addImmOperands";
  let DiagnosticType = !strconcat("Invalid", name);
}

def imm8 : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm8">;
}

class MOSInst8< dag outs, dag ins, string asmstr, list<dag> pattern, bits<8> opcode = 0> :
    MOSInst< outs, ins, asmstr, pattern>
{
  let Size = 1;
  field bits<8> Inst;
  let Inst = opcode;
}

class MOSInst16< dag outs, dag ins, string asmstr, list<dag> pattern> :
    MOSInst< outs, ins, asmstr, pattern>
{
  let Size = 2;
  field bits<16> Inst;
}
// Predicates for subtarget features.


def Has6502 : Predicate<"Subtarget->has6502()">,
                         AssemblerPredicate<"Feature6502">
{
  let PredicateName = "Feature6502";
}


