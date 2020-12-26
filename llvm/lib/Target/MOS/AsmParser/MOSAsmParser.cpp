//===---- MOSAsmParser.cpp - Parse MOS assembly to MCInst instructions ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MOS.h"
#include "MOSRegisterInfo.h"
#include "MCTargetDesc/MOSMCELFStreamer.h"
#include "MCTargetDesc/MOSMCExpr.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

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

#define DEBUG_TYPE "avr-asm-parser"

using namespace llvm;

namespace {
/// Parses MOS assembly from a stream.
class MOSAsmParser : public MCTargetAsmParser {
  const MCSubtargetInfo &STI;
  MCAsmParser &Parser;
  const MCRegisterInfo *MRI;
  const std::string GENERATE_STUBS = "gs";

#define GET_ASSEMBLER_HEADER
#include "MOSGenAsmMatcher.inc"

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool ParseDirective(AsmToken DirectiveID) override;

  OperandMatchResultTy parseMemriOperand(OperandVector &Operands);

  bool parseOperand(OperandVector &Operands);
  int parseRegisterName(unsigned (*matchFn)(StringRef));
  int parseRegisterName();
  int parseRegister();
  bool tryParseRegisterOperand(OperandVector &Operands);
  bool tryParseExpression(OperandVector &Operands);
  bool tryParseRelocExpression(OperandVector &Operands);
  void eatComma();

  unsigned validateTargetOperandClass(MCParsedAsmOperand &Op,
                                      unsigned Kind) override;

  unsigned toDREG(unsigned Reg, unsigned From = MOS::sub_lo) {
    MCRegisterClass const *Class = &MOSMCRegisterClasses[MOS::DREGSRegClassID];
    return MRI->getMatchingSuperReg(Reg, From, Class);
  }

  bool emit(MCInst &Instruction, SMLoc const &Loc, MCStreamer &Out) const;
  bool invalidOperand(SMLoc const &Loc, OperandVector const &Operands,
                      uint64_t const &ErrorInfo);
  bool missingFeature(SMLoc const &Loc, uint64_t const &ErrorInfo);

  bool parseLiteralValues(unsigned SizeInBytes, SMLoc L);

public:
  MOSAsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
               const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII), STI(STI), Parser(Parser) {
    MCAsmParserExtension::Initialize(Parser);
    MRI = getContext().getRegisterInfo();

    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }

  MCAsmParser &getParser() const { return Parser; }
  MCAsmLexer &getLexer() const { return Parser.getLexer(); }
};

/// An parsed MOS assembly operand.
class MOSOperand : public MCParsedAsmOperand {
  typedef MCParsedAsmOperand Base;
  enum KindTy { k_Immediate, k_Register, k_Token, k_Memri } Kind;

public:
  MOSOperand(StringRef Tok, SMLoc const &S)
      : Base(), Kind(k_Token), Tok(Tok), Start(S), End(S) {}
  MOSOperand(unsigned Reg, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(k_Register), RegImm({Reg, nullptr}), Start(S), End(E) {}
  MOSOperand(MCExpr const *Imm, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(k_Immediate), RegImm({0, Imm}), Start(S), End(E) {}
  MOSOperand(unsigned Reg, MCExpr const *Imm, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(k_Memri), RegImm({Reg, Imm}), Start(S), End(E) {}

  struct RegisterImmediate {
    unsigned Reg;
    MCExpr const *Imm;
  };
  union {
    StringRef Tok;
    RegisterImmediate RegImm;
  };

  SMLoc Start, End;

public:
  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Register && "Unexpected operand kind");
    assert(N == 1 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(getReg()));
  }

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

  /// Adds the contained reg+imm operand to an instruction.
  void addMemriOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Memri && "Unexpected operand kind");
    assert(N == 2 && "Invalid number of operands");

    Inst.addOperand(MCOperand::createReg(getReg()));
    addExpr(Inst, getImm());
  }

  bool isReg() const { return Kind == k_Register; }
  bool isImm() const { return Kind == k_Immediate; }
  bool isToken() const { return Kind == k_Token; }
  bool isMem() const { return Kind == k_Memri; }
  bool isMemri() const { return Kind == k_Memri; }

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

  static std::unique_ptr<MOSOperand> CreateToken(StringRef Str, SMLoc S) {
    return make_unique<MOSOperand>(Str, S);
  }

  static std::unique_ptr<MOSOperand> CreateReg(unsigned RegNum, SMLoc S,
                                               SMLoc E) {
    return make_unique<MOSOperand>(RegNum, S, E);
  }

  static std::unique_ptr<MOSOperand> CreateImm(const MCExpr *Val, SMLoc S,
                                               SMLoc E) {
    return make_unique<MOSOperand>(Val, S, E);
  }

  static std::unique_ptr<MOSOperand>
  CreateMemri(unsigned RegNum, const MCExpr *Val, SMLoc S, SMLoc E) {
    return make_unique<MOSOperand>(RegNum, Val, S, E);
  }

  void makeToken(StringRef Token) {
    Kind = k_Token;
    Tok = Token;
  }

  void makeReg(unsigned RegNo) {
    Kind = k_Register;
    RegImm = {RegNo, nullptr};
  }

  void makeImm(MCExpr const *Ex) {
    Kind = k_Immediate;
    RegImm = {0, Ex};
  }

  void makeMemri(unsigned RegNo, MCExpr const *Imm) {
    Kind = k_Memri;
    RegImm = {RegNo, Imm};
  }

  SMLoc getStartLoc() const { return Start; }
  SMLoc getEndLoc() const { return End; }

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

} // end anonymous namespace.

// Auto-generated Match Functions

/// Maps from the set of all register names to a register number.
/// \note Generated by TableGen.
static unsigned MatchRegisterName(StringRef Name);

/// Maps from the set of all alternative registernames to a register number.
/// \note Generated by TableGen.
static unsigned MatchRegisterAltName(StringRef Name);

bool MOSAsmParser::invalidOperand(SMLoc const &Loc,
                                  OperandVector const &Operands,
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

bool MOSAsmParser::missingFeature(llvm::SMLoc const &Loc,
                                  uint64_t const &ErrorInfo) {
  return Error(Loc, "instruction requires a CPU feature not currently enabled");
}

bool MOSAsmParser::emit(MCInst &Inst, SMLoc const &Loc, MCStreamer &Out) const {
  Inst.setLoc(Loc);
  Out.EmitInstruction(Inst, STI);

  return false;
}

bool MOSAsmParser::MatchAndEmitInstruction(SMLoc Loc, unsigned &Opcode,
                                           OperandVector &Operands,
                                           MCStreamer &Out, uint64_t &ErrorInfo,
                                           bool MatchingInlineAsm) {
  MCInst Inst;
  unsigned MatchResult =
      MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);

  switch (MatchResult) {
  case Match_Success:        return emit(Inst, Loc, Out);
  case Match_MissingFeature: return missingFeature(Loc, ErrorInfo);
  case Match_InvalidOperand: return invalidOperand(Loc, Operands, ErrorInfo);
  case Match_MnemonicFail:   return Error(Loc, "invalid instruction");
  default:                   return true;
  }
}

/// Parses a register name using a given matching function.
/// Checks for lowercase or uppercase if necessary.
int MOSAsmParser::parseRegisterName(unsigned (*matchFn)(StringRef)) {
  StringRef Name = Parser.getTok().getString();

  int RegNum = matchFn(Name);

  // GCC supports case insensitive register names. Some of the MOS registers
  // are all lower case, some are all upper case but non are mixed. We prefer
  // to use the original names in the register definitions. That is why we
  // have to test both upper and lower case here.
  if (RegNum == MOS::NoRegister) {
    RegNum = matchFn(Name.lower());
  }
  if (RegNum == MOS::NoRegister) {
    RegNum = matchFn(Name.upper());
  }

  return RegNum;
}

int MOSAsmParser::parseRegisterName() {
  int RegNum = parseRegisterName(&MatchRegisterName);

  if (RegNum == MOS::NoRegister)
    RegNum = parseRegisterName(&MatchRegisterAltName);

  return RegNum;
}

int MOSAsmParser::parseRegister() {
  int RegNum = MOS::NoRegister;

  if (Parser.getTok().is(AsmToken::Identifier)) {
    // Check for register pair syntax
    if (Parser.getLexer().peekTok().is(AsmToken::Colon)) {
      Parser.Lex();
      Parser.Lex(); // Eat high (odd) register and colon

      if (Parser.getTok().is(AsmToken::Identifier)) {
        // Convert lower (even) register to DREG
        RegNum = toDREG(parseRegisterName());
      }
    } else {
      RegNum = parseRegisterName();
    }
  }
  return RegNum;
}

bool MOSAsmParser::tryParseRegisterOperand(OperandVector &Operands) {
  int RegNo = parseRegister();

  if (RegNo == MOS::NoRegister)
    return true;

  AsmToken const &T = Parser.getTok();
  Operands.push_back(MOSOperand::CreateReg(RegNo, T.getLoc(), T.getEndLoc()));
  Parser.Lex(); // Eat register token.

  return false;
}

bool MOSAsmParser::tryParseExpression(OperandVector &Operands) {
  SMLoc S = Parser.getTok().getLoc();

  if (!tryParseRelocExpression(Operands))
    return false;

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

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(MOSOperand::CreateImm(Expression, S, E));
  return false;
}

bool MOSAsmParser::tryParseRelocExpression(OperandVector &Operands) {
  bool isNegated = false;
  MOSMCExpr::VariantKind ModifierKind = MOSMCExpr::VK_MOS_None;

  SMLoc S = Parser.getTok().getLoc();

  // Check for sign
  AsmToken tokens[2];
  size_t ReadCount = Parser.getLexer().peekTokens(tokens);

  if (ReadCount == 2) {
    if ((tokens[0].getKind() == AsmToken::Identifier &&
         tokens[1].getKind() == AsmToken::LParen) ||
        (tokens[0].getKind() == AsmToken::LParen &&
         tokens[1].getKind() == AsmToken::Minus)) {

      AsmToken::TokenKind CurTok = Parser.getLexer().getKind();
      if (CurTok == AsmToken::Minus ||
          tokens[1].getKind() == AsmToken::Minus) {
        isNegated = true;
      } else {
        assert(CurTok == AsmToken::Plus);
        isNegated = false;
      }

      // Eat the sign
      if (CurTok == AsmToken::Minus || CurTok == AsmToken::Plus)
        Parser.Lex();
    }
  }

  // Check if we have a target specific modifier (lo8, hi8, &c)
  if (Parser.getTok().getKind() != AsmToken::Identifier ||
      Parser.getLexer().peekTok().getKind() != AsmToken::LParen) {
    // Not a reloc expr
    return true;
  }
  StringRef ModifierName = Parser.getTok().getString();
  ModifierKind = MOSMCExpr::getKindByName(ModifierName.str().c_str());

  if (ModifierKind != MOSMCExpr::VK_MOS_None) {
    Parser.Lex();
    Parser.Lex(); // Eat modifier name and parenthesis
    if (Parser.getTok().getString() == GENERATE_STUBS &&
        Parser.getTok().getKind() == AsmToken::Identifier) {
      std::string GSModName = ModifierName.str() + "_" + GENERATE_STUBS;
      ModifierKind = MOSMCExpr::getKindByName(GSModName.c_str());
      if (ModifierKind != MOSMCExpr::VK_MOS_None)
        Parser.Lex(); // Eat gs modifier name
    }
  } else {
    return Error(Parser.getTok().getLoc(), "unknown modifier");
  }

  if (tokens[1].getKind() == AsmToken::Minus ||
      tokens[1].getKind() == AsmToken::Plus) {
    Parser.Lex();
    assert(Parser.getTok().getKind() == AsmToken::LParen);
    Parser.Lex(); // Eat the sign and parenthesis
  }

  MCExpr const *InnerExpression;
  if (getParser().parseExpression(InnerExpression))
    return true;

  if (tokens[1].getKind() == AsmToken::Minus ||
      tokens[1].getKind() == AsmToken::Plus) {
    assert(Parser.getTok().getKind() == AsmToken::RParen);
    Parser.Lex(); // Eat closing parenthesis
  }

  // If we have a modifier wrap the inner expression
  assert(Parser.getTok().getKind() == AsmToken::RParen);
  Parser.Lex(); // Eat closing parenthesis

  MCExpr const *Expression = MOSMCExpr::create(ModifierKind, InnerExpression,
                                               isNegated, getContext());

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(MOSOperand::CreateImm(Expression, S, E));

  return false;
}

bool MOSAsmParser::parseOperand(OperandVector &Operands) {
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
  }
  }

  // Could not parse operand
  return true;
}

OperandMatchResultTy
MOSAsmParser::parseMemriOperand(OperandVector &Operands) {
  LLVM_DEBUG(dbgs() << "parseMemriOperand()\n");

  SMLoc E, S;
  MCExpr const *Expression;
  int RegNo;

  // Parse register.
  {
    RegNo = parseRegister();

    if (RegNo == MOS::NoRegister)
      return MatchOperand_ParseFail;

    S = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    Parser.Lex(); // Eat register token.
  }

  // Parse immediate;
  {
    if (getParser().parseExpression(Expression))
      return MatchOperand_ParseFail;

    E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  }

  Operands.push_back(MOSOperand::CreateMemri(RegNo, Expression, S, E));

  return MatchOperand_Success;
}

bool MOSAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                 SMLoc &EndLoc) {
  StartLoc = Parser.getTok().getLoc();
  RegNo = parseRegister();
  EndLoc = Parser.getTok().getLoc();

  return (RegNo == MOS::NoRegister);
}

void MOSAsmParser::eatComma() {
  if (getLexer().is(AsmToken::Comma)) {
    Parser.Lex();
  } else {
    // GCC allows commas to be omitted.
  }
}

bool MOSAsmParser::ParseInstruction(ParseInstructionInfo &Info,
                                    StringRef Mnemonic, SMLoc NameLoc,
                                    OperandVector &Operands) {
  Operands.push_back(MOSOperand::CreateToken(Mnemonic, NameLoc));

  bool first = true;
  while (getLexer().isNot(AsmToken::EndOfStatement)) {
    if (!first) eatComma();

    first = false;

    auto MatchResult = MatchOperandParserImpl(Operands, Mnemonic);

    if (MatchResult == MatchOperand_Success) {
      continue;
    }

    if (MatchResult == MatchOperand_ParseFail) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();

      return Error(Loc, "failed to parse register and immediate pair");
    }

    if (parseOperand(Operands)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }
  }
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

bool MOSAsmParser::ParseDirective(llvm::AsmToken DirectiveID) {
  StringRef IDVal = DirectiveID.getIdentifier();
  if (IDVal.lower() == ".long") {
    parseLiteralValues(SIZE_LONG, DirectiveID.getLoc());
  } else if (IDVal.lower() == ".word" || IDVal.lower() == ".short") {
    parseLiteralValues(SIZE_WORD, DirectiveID.getLoc());
  } else if (IDVal.lower() == ".byte") {
    parseLiteralValues(1, DirectiveID.getLoc());
  }
  return true;
}

bool MOSAsmParser::parseLiteralValues(unsigned SizeInBytes, SMLoc L) {
  MCAsmParser &Parser = getParser();
  MOSMCELFStreamer &MOSStreamer =
      static_cast<MOSMCELFStreamer &>(Parser.getStreamer());
  AsmToken Tokens[2];
  size_t ReadCount = Parser.getLexer().peekTokens(Tokens);
  if (ReadCount == 2 && Parser.getTok().getKind() == AsmToken::Identifier &&
      Tokens[0].getKind() == AsmToken::Minus &&
      Tokens[1].getKind() == AsmToken::Identifier) {
    MCSymbol *Symbol = getContext().getOrCreateSymbol(".text");
    MOSStreamer.EmitValueForModiferKind(Symbol, SizeInBytes, L,
            MOSMCExpr::VK_MOS_None);
    return false;
  }

  if (Parser.getTok().getKind() == AsmToken::Identifier &&
      Parser.getLexer().peekTok().getKind() == AsmToken::LParen) {
    StringRef ModifierName = Parser.getTok().getString();
    MOSMCExpr::VariantKind ModifierKind =
        MOSMCExpr::getKindByName(ModifierName.str().c_str());
    if (ModifierKind != MOSMCExpr::VK_MOS_None) {
      Parser.Lex();
      Parser.Lex(); // Eat the modifier and parenthesis
    } else {
      return Error(Parser.getTok().getLoc(), "unknown modifier");
    }
    MCSymbol *Symbol =
        getContext().getOrCreateSymbol(Parser.getTok().getString());
    MOSStreamer.EmitValueForModiferKind(Symbol, SizeInBytes, L, ModifierKind);
    return false;
  }

  auto parseOne = [&]() -> bool {
    const MCExpr *Value;
    if (Parser.parseExpression(Value))
      return true;
    Parser.getStreamer().EmitValue(Value, SizeInBytes, L);
    return false;
  };
  return (parseMany(parseOne));
}

extern "C" void LLVMInitializeMOSAsmParser() {
  RegisterMCAsmParser<MOSAsmParser> X(getTheMOSTarget());
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "MOSGenAsmMatcher.inc"

// Uses enums defined in MOSGenAsmMatcher.inc
unsigned MOSAsmParser::validateTargetOperandClass(MCParsedAsmOperand &AsmOp,
                                                  unsigned ExpectedKind) {
  MOSOperand &Op = static_cast<MOSOperand &>(AsmOp);
  MatchClassKind Expected = static_cast<MatchClassKind>(ExpectedKind);

  // If need be, GCC converts bare numbers to register names
  // It's ugly, but GCC supports it.
  if (Op.isImm()) {
    if (MCConstantExpr const *Const = dyn_cast<MCConstantExpr>(Op.getImm())) {
      int64_t RegNum = Const->getValue();
      std::ostringstream RegName;
      RegName << "r" << RegNum;
      RegNum = MatchRegisterName(RegName.str().c_str());
      if (RegNum != MOS::NoRegister) {
        Op.makeReg(RegNum);
        if (validateOperandClass(Op, Expected) == Match_Success) {
          return Match_Success;
        }
      }
      // Let the other quirks try their magic.
    }
  }

  if (Op.isReg()) {
    // If the instruction uses a register pair but we got a single, lower
    // register we perform a "class cast".
    if (isSubclass(Expected, MCK_DREGS)) {
      unsigned correspondingDREG = toDREG(Op.getReg());

      if (correspondingDREG != MOS::NoRegister) {
        Op.makeReg(correspondingDREG);
        return validateOperandClass(Op, Expected);
      }
    }
  }
  return Match_InvalidOperand;
}
