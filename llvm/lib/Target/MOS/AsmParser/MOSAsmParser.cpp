//===---- MOSAsmParser.cpp - Parse MOS assembly to MCInst instructions ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MOSMCELFStreamer.h"
#include "MCTargetDesc/MOSMCExpr.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSRegisterInfo.h"

#include "llvm/ADT/APInt.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

#include <sstream>

#define DEBUG_TYPE "mos-asm-parser"

using namespace llvm;

namespace llvm {

/// An parsed MOS assembly operand.
class MOSOperand : public MCParsedAsmOperand {
public:
  typedef MCParsedAsmOperand Base;
  enum KindTy { k_Immediate, k_Register, k_Token, k_Memri } Kind;

  struct RegisterImmediate {
    unsigned Reg;
    MCExpr const *Imm;
  };
  union {
    StringRef Tok;
    RegisterImmediate RegImm;
  };

  SMLoc Start, End;

  MOSOperand(StringRef Tok, SMLoc const &S)
      : Base(), Kind(k_Token), Tok(Tok), Start(S), End(S) {}
  MOSOperand(unsigned Reg, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(k_Register), RegImm({Reg, nullptr}), Start(S), End(E) {}
  MOSOperand(MCExpr const *Imm, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(k_Immediate), RegImm({0, Imm}), Start(S), End(E) {}
  MOSOperand(unsigned Reg, MCExpr const *Imm, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(k_Memri), RegImm({Reg, Imm}), Start(S), End(E) {}

  bool tryParseImmediate(OperandVector &operands);
  bool tryParseRegisterOperand(OperandVector &Operands);
  bool tryParseExpression(OperandVector &Operand);

  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    // Add as immediate when possible
    if (!Expr)
      Inst.addOperand(MCOperand::createImm(0));
    else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Immediate && "Unexpected operand kind");
    assert(N == 1 && "Invalid number of operands!");

    const MCExpr *Expr = getImm();
    addExpr(Inst, Expr);
  }

  void addRegOperands(MCInst &Inst, unsigned N) const {
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  static std::unique_ptr<MOSOperand> CreateImm(const MCExpr *Val, SMLoc S,
                                               SMLoc E) {
    return make_unique<MOSOperand>(Val, S, E);
  }

  static std::unique_ptr<MOSOperand> CreateReg(unsigned RegNum, SMLoc S,
                                               SMLoc E) {
    return make_unique<MOSOperand>(RegNum, S, E);
  }

  static std::unique_ptr<MOSOperand> CreateToken(StringRef Str, SMLoc S) {
    return make_unique<MOSOperand>(Str, S);
  }

  SMLoc getEndLoc() const { return End; }

  StringRef getToken() const {
    assert(Kind == k_Token && "Invalid access!");
    return Tok;
  }

  unsigned getReg() const {
    assert((Kind == k_Register || Kind == k_Memri) && "Invalid access!");

    return RegImm.Reg;
  }

  const MCExpr *getImm() const {
    assert((Kind == k_Immediate || Kind == k_Memri) && "Invalid access!");
    return RegImm.Imm;
  }

  SMLoc getStartLoc() const { return Start; }

  template<int64_t N, int64_t M>
  bool isImmediate() const {
    if (!isImm()) return false;
    const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(getImm());
    if (!CE) return false;
    int64_t Value = CE->getValue();
    return Value >= N && Value <= M;
  }

  virtual bool isImm() const { return (Kind == k_Immediate); }
  virtual bool isImm8() const { return isImmediate<0, (1 << 8) >(); }
  virtual bool isImm16() const { return isImmediate<0, (1 << 16) >(); }

  virtual bool isMem() const { return (Kind == k_Memri); }
  virtual bool isReg() const { return (Kind == k_Register); }
  virtual bool isToken() const { return (Kind == k_Token); }

  void makeImm(MCExpr const *Ex) {
    Kind = k_Immediate;
    RegImm = {0, Ex};
  }

  void makeReg(unsigned RegNo) {
    Kind = k_Register;
    RegImm = {RegNo, nullptr};
  }

  void makeToken(StringRef Token) {
    Kind = k_Token;
    Tok = Token;
  }

  virtual void print(raw_ostream &O) const {
    switch (Kind) {
    case k_Token:
      O << "Token: \"" << getToken() << "\"";
      break;
    case k_Register:
      O << "Register: " << getReg();
      break;
    case k_Immediate:
      O << "Immediate: \"" << *getImm() << "\"";
      break;
    case k_Memri: {
      // only manually print the size for non-negative values,
      // as the sign is inserted automatically.
      O << "Memri: \"" << getReg() << '+' << *getImm() << "\"";
      break;
    }
    }
    O << "\n";
  }
};

/// Parses MOS assembly from a stream.
class MOSAsmParser : public MCTargetAsmParser {
  const MCSubtargetInfo &STI;
  MCAsmParser &Parser;
  const MCRegisterInfo *MRI;

#define GET_ASSEMBLER_HEADER
#include "MOSGenAsmMatcher.inc"

public:

 enum MOSMatchResultTy {
    Match_UnknownError = FIRST_TARGET_MATCH_RESULT_TY,
#define GET_OPERAND_DIAGNOSTIC_TYPES
#include "MOSGenAsmMatcher.inc"

  };

  MOSAsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
               const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII), STI(STI), Parser(Parser) {
    MCAsmParserExtension::Initialize(Parser);
    MRI = getContext().getRegisterInfo();

    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }
  MCAsmLexer &getLexer() const { return Parser.getLexer(); }
  MCAsmParser &getParser() const { return Parser; }

  // #define GET_REGISTER_MATCHER
  // #include "MOSGenAsmMatcher.inc"

  bool invalidOperand(SMLoc const &Loc, OperandVector const &Operands,
                      uint64_t const &ErrorInfo) {
    SMLoc ErrorLoc = Loc;
    char const *Diag = 0;

    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size()) {
        Diag = "too few operands for instruction.";
      } else {
        MOSOperand const &Op = (MOSOperand const &)*Operands[ErrorInfo];

        // TODO: See if we can do a better error than just "invalid ...".
        if (Op.getStartLoc() != SMLoc()) {
          ErrorLoc = Op.getStartLoc();
        }
      }
    }

    if (!Diag) {
      Diag = "invalid operand for instruction";
    }

    return Error(ErrorLoc, Diag);
  }

  bool missingFeature(llvm::SMLoc const &Loc, uint64_t const &ErrorInfo) {
    return Error(Loc,
                 "instruction requires a CPU feature not currently enabled");
  }

  bool emit(MCInst &Inst, SMLoc const &Loc, MCStreamer &Out) const {
    Inst.setLoc(Loc);
    Out.EmitInstruction(Inst, STI);

    return false;
  }

  /// MatchAndEmitInstruction - Recognize a series of operands of a parsed
  /// instruction as an actual MCInst and emit it to the specified MCStreamer.
  /// This returns false on success and returns true on failure to match.
  ///
  /// On failure, the target parser is responsible for emitting a diagnostic
  /// explaining the match failure.
  bool MatchAndEmitInstruction(SMLoc Loc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override {
    MCInst Inst;
    unsigned MatchResult = MatchInstructionImpl(Operands, Inst, ErrorInfo,
                                                MatchingInlineAsm);
    switch (MatchResult) {
    case Match_Success:
      return emit(Inst, Loc, Out);
    case Match_MissingFeature:
      return missingFeature(Loc, ErrorInfo);
    case Match_InvalidOperand:
      return invalidOperand(Loc, Operands, ErrorInfo);
    case Match_MnemonicFail:
      return Error(Loc, "invalid instruction");
    case Match_InvalidImm8:
      return Error(Loc, "operand too large; must be an 8-bit value");
    case Match_NearMisses:
      return Error(Loc, "found some near misses");
    default:
      return true;
    }
  }

  /// ParseDirective - Parse a target specific assembler directive
  ///
  /// The parser is positioned following the directive name.  The target
  /// specific directive parser should parse the entire directive doing or
  /// recording any target specific work, or return true and do nothing if the
  /// directive is not target specific. If the directive is specific for
  /// the target, the entire line is parsed up to and including the
  /// end-of-statement token and false is returned.
  ///
  /// \param DirectiveID - the identifier token of the directive.
  virtual bool ParseDirective(AsmToken DirectiveID) override {
    // todo
    return true;
  }

  bool parseOperand(OperandVector &Operands) {
    LLVM_DEBUG(dbgs() << "parseOperand\n");

    switch (getLexer().getKind()) {
    default:
      return Error(Parser.getTok().getLoc(), "unexpected token in operand");

    case AsmToken::Identifier:
      // Try to parse a register, if it fails,
      // fall through to the next case.
      if (!tryParseRegisterOperand(Operands)) {
        return false;
      }
      LLVM_FALLTHROUGH;
    case AsmToken::LParen:
    case AsmToken::Integer:
    case AsmToken::Dot:
      return tryParseExpression(Operands);
    case AsmToken::Plus:
    case AsmToken::Minus: {
      // If the sign preceeds a number, parse the number,
      // otherwise treat the sign a an independent token.
      switch (getLexer().peekTok().getKind()) {
      case AsmToken::Integer:
      case AsmToken::BigNum:
      case AsmToken::Identifier:
      case AsmToken::Real:
        if (!tryParseExpression(Operands))
          return false;
        break;
      default:
        break;
      }
      // Treat the token as an independent token.
      Operands.push_back(MOSOperand::CreateToken(Parser.getTok().getString(),
                                                 Parser.getTok().getLoc()));
      Parser.Lex(); // Eat the token.
      return false;
    } // case AsmToken::minus
    } // switch

    // Could not parse operand
    return true;
  } // class MOSOperand;

  virtual bool ParseInstruction(ParseInstructionInfo &Info, StringRef Mnemonic,
                                SMLoc NameLoc,
                                OperandVector &Operands) override {
    // todo
    Operands.push_back(MOSOperand::CreateToken(Mnemonic, NameLoc));

    while (getLexer().isNot(AsmToken::EndOfStatement)) {

      if (!tryParseImmediate(Operands))
        continue;

      if (!parseOperand(Operands)) {
        continue;
      }

      if (!tryParseRegisterOperand(Operands)) {
        continue;
      }

      if (!tryParseExpression(Operands)) {
        continue;
      }

      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "failed to parse register and immediate pair");
    }
    Parser.Lex(); // Consume the EndOfStatement
    return false;
  }

  int parseRegister() {
    int RegNum = MOS::NoRegister;

    if (Parser.getTok().is(AsmToken::Identifier)) {
      // Check for register pair syntax
      if (Parser.getLexer().peekTok().is(AsmToken::Colon)) {
        Parser.Lex();
        Parser.Lex(); // Eat high (odd) register and colon
        /*

        if (Parser.getTok().is(AsmToken::Identifier)) {
          // Convert lower (even) register to DREG
          RegNum = toDREG(parseRegisterName());
        }
        */
      } else {
        RegNum = parseRegisterName();
      }
    }
    return RegNum;
  }

  virtual bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                             SMLoc &EndLoc) override {
    // todo
    return true;
  }

  int parseRegisterName(unsigned (*matchFn)(StringRef)) {
    StringRef Name = Parser.getTok().getString();

    int RegNum = matchFn(Name);

    if (RegNum == MOS::NoRegister) {
      RegNum = matchFn(Name.lower());
    }
    if (RegNum == MOS::NoRegister) {
      RegNum = matchFn(Name.upper());
    }

    return RegNum;
  }

  int parseRegisterName();

  bool tryParseImmediate(OperandVector &Operands) {
    if (Parser.getTok().is(AsmToken::Hash)) {
      Lex();
      AsmToken const &T = Parser.getTok();
      // it's a hex constant
      MCExpr const *Expression;
      if (getParser().parseExpression(Expression))
        return true;

      if (Parser.getTok().is(AsmToken::Integer)) {
        Operands.push_back(MOSOperand::CreateImm(Expression, T.getLoc(), T.getEndLoc()));
        return false;
      }

      /*
      Operands.push_back( MOSOperand::CreateImm( T, T.getLoc(), T.getEndLoc());
      */
      return true;
    }
    return true;
  }

  bool tryParseRegisterOperand(OperandVector &Operands) {
    int RegNo = parseRegister();

    if (RegNo == MOS::NoRegister)
      return true;

    AsmToken const &T = Parser.getTok();
    Operands.push_back(MOSOperand::CreateReg(RegNo, T.getLoc(), T.getEndLoc()));
    Parser.Lex(); // Eat register token.

    return false;
  }

  bool tryParseExpression(OperandVector &Operands) {
    SMLoc S = Parser.getTok().getLoc();

    if ((Parser.getTok().getKind() == AsmToken::Plus ||
         Parser.getTok().getKind() == AsmToken::Minus) &&
        Parser.getLexer().peekTok().getKind() == AsmToken::Identifier) {
      // Don't handle this case - it should be split into two
      // separate tokens.
      return true;
    }

    // Parse (potentially inner) expression
    MCExpr const *Expression;
    if (getParser().parseExpression(Expression))
      return true;

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer());
    Operands.push_back(MOSOperand::CreateImm(Expression, S, E));
    return false;
  }
}; // class MOSAsmParser
#define GET_MATCHER_IMPLEMENTATION
// #define GET_SUBTARGET_FEATURE_NAME
#define GET_REGISTER_MATCHER
// #define GET_MNEMONIC_SPELL_CHECKER
#include "MOSGenAsmMatcher.inc"

int MOSAsmParser::parseRegisterName() {
  int RegNum = parseRegisterName(&MatchRegisterName);

  if (RegNum == MOS::NoRegister)
    RegNum = parseRegisterName(&MatchRegisterAltName);

  return RegNum;
}

extern "C" void LLVMInitializeMOSAsmParser() {
  RegisterMCAsmParser<MOSAsmParser> X(getTheMOSTarget());
}

} // namespace llvm
