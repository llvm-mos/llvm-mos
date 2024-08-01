//===---- MOSAsmParser.cpp - Parse MOS assembly to MCInst instructions ----===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MOSFixupKinds.h"
#include "MCTargetDesc/MOSMCELFStreamer.h"
#include "MCTargetDesc/MOSMCExpr.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MCTargetDesc/MOSTargetStreamer.h"
#include "MOS.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCAsmMacro.h"
#include "llvm/MC/MCAssembler.h"
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
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"

#include <sstream>

#define DEBUG_TYPE "mos-asm-parser"

namespace llvm {

#define GET_REGISTER_MATCHER
#include "MOSGenAsmMatcher.inc"

class MOSOperand : public MCParsedAsmOperand {
public:
  MOSOperand() = delete;
  /// Create an immediate MOSOperand.
  MOSOperand(const MOSSubtarget &STI, const MCExpr *Val, SMLoc S, SMLoc E)
      : Kind(k_Immediate), Imm(Val), Start(S), End(E), STI(STI){};
  /// Create a register MOSOperand.
  MOSOperand(const MOSSubtarget &STI, unsigned RegNum, SMLoc S, SMLoc E)
      : Kind(k_Register), Reg(RegNum), Start(S), End(E), STI(STI){};
  /// Create a token MOSOperand.
  MOSOperand(const MOSSubtarget &STI, StringRef Str, SMLoc S)
      : Kind(k_Token), Tok(Str), Start(S), STI(STI){};

private:
  enum KindTy {
    k_None,
    k_Immediate,
    k_Register,
    k_Token,
  } Kind{k_None};

  MCExpr const *Imm{};
  unsigned int Reg{};
  StringRef Tok;
  SMLoc Start, End;
  const MOSSubtarget &STI;

public:
  template <int64_t Low, int64_t High> bool isImmediate() const {
    if (!isImm()) {
      return false;
    }

    // if it's a MOS-specific modifier, we need to figure out the size based
    // on the type of expression.  If the largest value that the modifier
    // can produce is larger than the expression that this immediate can hold,
    // we have to refuse to match it, so that we don't inadvertently match
    // zero-page addresses against 16-bit modifiers.
    const auto *MME = dyn_cast<MOSMCExpr>(getImm());
    if (MME) {
      const MOS::Fixups Kind = MME->getFixupKind();
      // Imm16 modifier enforces a lower bound which rejects Imm8.
      if (Kind == MOS::Imm16 && High < 0x10000 - 1)
        return false;
      const MCFixupKindInfo &Info =
          MOSFixupKinds::getFixupKindInfo(Kind, nullptr);
      int64_t MaxValue = (1 << Info.TargetSize) - 1;
      bool Evaluated = false;
      int64_t Constant = 0;
      Evaluated = MME->evaluateAsConstant(Constant);
      // if the constant is non-zero, evaluate for size now
      if (Evaluated && Constant > 0) {
        return (Constant <= MaxValue);
      }
      return (MaxValue <= High);
    }

    // if it's a symbol ref, we'll replace it later
    const auto *SRE = dyn_cast<MCSymbolRefExpr>(getImm());
    if (SRE) {
      return true;
    }
    // if it's an immediate but not castable to one, it must be a label
    const auto *CE = dyn_cast<MCConstantExpr>(getImm());
    if (!CE) {
      return true;
    }
    int64_t Value = CE->getValue();
    return Value >= Low && Value <= High;
  }

  bool is(KindTy K) const { return (Kind == K); }
  bool isToken() const override { return is(k_Token); }
  bool isImm() const override { return is(k_Immediate); }
  bool isReg() const override { return is(k_Register); }
  bool isMem() const override {
    assert(false);
    return false;
  }
  SMLoc getStartLoc() const override { return Start; }
  SMLoc getEndLoc() const override { return End; }
  const StringRef &getToken() const {
    assert(isToken());
    return Tok;
  }
  const MCExpr *getImm() const {
    assert(isImm());
    return Imm;
  };
  MCRegister getReg() const override {
    assert(isReg());
    return Reg;
  }

  bool isImm3() const { return isImmediate<0, 0x8 - 1>(); }
  bool isImm4() const { return isImmediate<0, 0x10 - 1>(); }
  bool isImm8() const { return isImmediate<0, 0x100 - 1>(); }
  bool isImm16() const { return isImmediate<0, 0x10000 - 1>(); }
  bool isImm24() const { return isImmediate<0, 0x1000000 - 1>(); }
  bool isPCRel8() const { return isImm8(); }
  bool isPCRel16() const { return isImm16(); }
  bool isAddr8() const {
    // For constants, use the offseted zero page.
    auto ZeroPageOffset = STI.hasFeature(MOS::FeatureHUC6280) ? 0x2000 : 0;
    if (ZeroPageOffset != 0 && isImm()) {
      const auto *CE = dyn_cast<MCConstantExpr>(getImm());
      if (CE) {
        int64_t Value = CE->getValue();
        return Value >= ZeroPageOffset && Value <= ZeroPageOffset + 0xFF;
      }
    }
    return isImm8();
  }
  bool isAddr13() const { return isImmediate<0, 0x2000 - 1>(); }
  bool isAddr16() const { return isImm16(); }
  bool isAddr24() const { return isImm24(); }

  static void addExpr(MCInst &Inst, const MCExpr *Expr) {
    if (const auto *CE = dyn_cast<MCConstantExpr>(Expr)) {
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    } else {
      Inst.addOperand(MCOperand::createExpr(Expr));
    }
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Immediate && "Unexpected operand kind");
    assert(N == 1 && "Invalid number of operands!");

    const MCExpr *Expr = getImm();
    addExpr(Inst, Expr);
  }

  void addRegOperands(MCInst &Inst, unsigned /*N*/) const {
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addPCRel8Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addPCRel16Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addAddr8Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addAddr13Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addAddr16Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addAddr24Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  static std::unique_ptr<MOSOperand>
  createImm(const MOSSubtarget &STI, const MCExpr *Val, SMLoc S, SMLoc E) {
    return std::make_unique<MOSOperand>(STI, Val, S, E);
  }

  static std::unique_ptr<MOSOperand>
  createReg(const MOSSubtarget &STI, unsigned RegNum, SMLoc S, SMLoc E) {
    return std::make_unique<MOSOperand>(STI, RegNum, S, E);
  }

  static std::unique_ptr<MOSOperand> createToken(const MOSSubtarget &STI,
                                                 StringRef Str, SMLoc S) {
    return std::make_unique<MOSOperand>(STI, Str, S);
  }

  void print(raw_ostream &O) const override {
    switch (Kind) {
    case k_None:
      O << "None";
      break;
    case k_Token:
      O << "Token: \"" << getToken() << "\"";
      break;
    case k_Register:
      O << "Register: " << getReg();
      break;
    case k_Immediate:
      O << "Immediate: \"" << *getImm() << "\"";
      break;
    }
    O << "\n";
  };
};

/// Parses MOS assembly from a stream.
class MOSAsmParser : public MCTargetAsmParser {
  const MOSSubtarget &STI;
  MCAsmParser &Parser;
  const MCRegisterInfo *MRI;
  const std::string GenerateStubs = "gs";

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
      : MCTargetAsmParser(Options, STI, MII),
        STI(static_cast<const MOSSubtarget &>(STI)), Parser(Parser) {
    MCAsmParserExtension::Initialize(Parser);
    MRI = getContext().getRegisterInfo();

    Parser.addAliasForDirective(".hword", ".byte");
    Parser.addAliasForDirective(".word", ".2byte");
    Parser.addAliasForDirective(".dword", ".4byte");
    Parser.addAliasForDirective(".xword", ".8byte");

    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }
  MCAsmLexer &getLexer() const { return Parser.getLexer(); }
  MCAsmParser &getParser() const { return Parser; }

  bool parsePrimaryExpr(const MCExpr *&Res, SMLoc &EndLoc) override {
    return MCTargetAsmParser::parsePrimaryExpr(Res, EndLoc);
  }

  bool invalidOperand(SMLoc const &Loc, OperandVector const &Operands,
                      uint64_t const &ErrorInfo) {
    SMLoc ErrorLoc = Loc;
    char const *Diag = nullptr;

    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size()) {
        Diag = "too few operands for instruction";
      } else {
        ErrorLoc = Operands[ErrorInfo]->getStartLoc();
      }
    }

    if (Diag == nullptr) {
      Diag = "invalid operand for instruction";
    }

    return Error(ErrorLoc, Diag);
  }

  bool missingFeature(llvm::SMLoc const &Loc, uint64_t const & /*ErrorInfo*/) {
    return Error(Loc,
                 "instruction requires a CPU feature not currently enabled");
  }

  bool emit(MCInst &Inst, SMLoc const &Loc, MCStreamer &Out) const {
    Inst.setLoc(Loc);
    Out.emitInstruction(Inst, STI);

    return false;
  }

  /// MatchAndEmitInstruction - Recognize a series of operands of a parsed
  /// instruction as an actual MCInst and emit it to the specified MCStreamer.
  /// This returns false on success and returns true on failure to match.
  ///
  /// On failure, the target parser is responsible for emitting a diagnostic
  /// explaining the match failure.
  bool MatchAndEmitInstruction(SMLoc Loc, unsigned & /*Opcode*/,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override {
    MCInst Inst;
    unsigned MatchResult =
        // we always want ConvertToMapAndConstraints to be called
        MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);
    switch (MatchResult) {
    case Match_Success:
      return emit(Inst, Loc, Out);
    case Match_MissingFeature:
      return missingFeature(Loc, ErrorInfo);
    case Match_InvalidOperand:
      return invalidOperand(Loc, Operands, ErrorInfo);
    case Match_MnemonicFail:
      return Error(Loc, "invalid instruction");
    case Match_InvalidAddr8:
      return Error(Loc, "operand must be an 8-bit address");
    case Match_InvalidAddr16:
      return Error(Loc, "operand must be a 16-bit address");
    case Match_InvalidPCRel8:
      return Error(Loc, "operand must be an 8-bit PC relative address");
    case Match_immediate:
      return Error(Loc, "operand must be an immediate value");
    case Match_NearMisses:
      return Error(Loc, "found some near misses");
    default:
      return true;
    }
  }

  MOSTargetStreamer &getTargetStreamer() {
    MCTargetStreamer &TS = *getParser().getStreamer().getTargetStreamer();
    return static_cast<MOSTargetStreamer &>(TS);
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
  bool ParseDirective(AsmToken DirectiveID) override {
    StringRef IDVal = DirectiveID.getIdentifier();
    if (IDVal.starts_with(".mos_addr_asciz"))
      return parseAddrAsciz(DirectiveID.getLoc());
    if (IDVal.starts_with(".zeropage"))
      return parseZeropage(DirectiveID.getLoc());
    return true;
  }

  bool parseAddrAsciz(SMLoc DirectiveLoc) {
    const MCExpr *AddrValue;
    SMLoc AddrLoc = getLexer().getLoc();
    if (Parser.checkForValidSection() || Parser.parseExpression(AddrValue))
      return true;

    if (Parser.parseToken(AsmToken::Comma, "expected `, <char-count>`"))
      return true;

    SMLoc CharCountLoc = getLexer().getLoc();
    int64_t CharCountValue;
    if (Parser.parseAbsoluteExpression(CharCountValue))
      return true;

    if (CharCountValue < 1 || CharCountValue > 8) {
      return Error(CharCountLoc, "char count out of range [1,8]");
    }

    // Special case constant expressions to match code generator.
    if (const MCConstantExpr *MCE = dyn_cast<MCConstantExpr>(AddrValue)) {
      std::string ValueStr = itostr(MCE->getValue());
      if (ValueStr.size() > static_cast<size_t>(CharCountValue))
        return Error(AddrLoc, "out of range literal value");
      getStreamer().emitBytes(ValueStr);
      getStreamer().emitBytes(StringRef("\0\0\0\0\0\0\0\0\0",
                                        CharCountValue - ValueStr.size() + 1));
    } else {
      const MOSMCExpr *Expr =
          MOSMCExpr::create(MOSMCExpr::VK_MOS_ADDR_ASCIZ, AddrValue,
                            /*isNegated=*/false, getContext());
      getStreamer().emitValue(Expr, CharCountValue + 1, DirectiveLoc);
    }
    return false;
  }

  bool parseZeropage(SMLoc DirectiveLoc) {
    auto parseOp = [&]() -> bool {
      StringRef Name;
      SMLoc Loc = getTok().getLoc();
      if (Parser.parseIdentifier(Name))
        return Error(Loc, "expected identifier");

      if (Parser.discardLTOSymbol(Name))
        return false;

      MCSymbol *Sym = getContext().getOrCreateSymbol(Name);

      if (!getTargetStreamer().emitDirectiveZeroPage(Sym))
        return Error(Loc, "unable to mark symbol as zero page");
      return false;
    };

    return parseMany(parseOp);
  }

  signed char hexToChar(const signed char Letter) {
    if (Letter >= '0' && Letter <= '9') {
      return static_cast<signed char>(Letter - '0');
    }
    if (Letter >= 'a' && Letter <= 'f') {
      return static_cast<signed char>(Letter - 'a' + 10);
    }
    return -1;
  }

  // Converts what could be a hex string to an integer value.
  // Result must fit into 32 bits.  More than that is an error.
  // Like everything else in this particular API, it returns false on success.
  bool tokenToHex(uint64_t &Res, const AsmToken &Tok) {
    Res = 0;
    std::string Text = Tok.getString().str();
    if (Text.size() > 8) {
      return true;
    }
    std::transform(Text.begin(), Text.end(), Text.begin(),
                   [](unsigned char C) { return std::tolower(C); });
    for (char Letter : Text) {
      signed char Converted = hexToChar(Letter);
      if (Converted == -1) {
        return true;
      }
      Res = (Res * 16) + Converted;
    }
    return false;
  }

  void eatThatToken(OperandVector &Operands) {
    Operands.push_back(MOSOperand::createToken(
        STI, getLexer().getTok().getString(), getLexer().getLoc()));
    Lex();
  }

  bool tryPushSPC700AdditionExpr(OperandVector &Operands, const MCExpr *LHS,
                                const MCBinaryExpr *BE, SMLoc S, SMLoc E) {
    // On SPC700, the expression can end in +X or +Y, which should be parsed
    // as an addressing mode, not as part of the expression.
    if (const auto *SE = dyn_cast<MCSymbolRefExpr>(BE->getRHS())) {
      if ((SE->getSymbol().getName().equals_insensitive("x") ||
          SE->getSymbol().getName().equals_insensitive("y")) &&
          BE->getOpcode() == MCBinaryExpr::Add) {
        Operands.push_back(MOSOperand::createImm(STI, LHS, S, E));
        Operands.push_back(MOSOperand::createToken(
            STI, "+", BE->getLoc()));
        Operands.push_back(MOSOperand::createToken(
            STI, SE->getSymbol().getName(), SE->getLoc()));
        return false;
      }
    }
    return true;
  }

  void pushExpr(OperandVector &Operands, const MCExpr *Val, SMLoc S, SMLoc E) {
    if (STI.hasFeature(MOS::FeatureSPC700)) {
      // Detect mos...(expr+x)
      if (const auto *ME = dyn_cast<MOSMCExpr>(Val)) {
        if (const auto *BE = dyn_cast<MCBinaryExpr>(ME->getSubExpr())) {
          MCExpr const *LHS = MOSMCExpr::create(ME->getKind(), BE->getLHS(),
                                                ME->isNegated(), getContext());
          if (!tryPushSPC700AdditionExpr(Operands, LHS, BE, S, E)) {
            return;
          }
        }
      }
      // Detect expr+x
      if (const auto *BE = dyn_cast<MCBinaryExpr>(Val)) {
        if (!tryPushSPC700AdditionExpr(Operands, BE->getLHS(), BE, S, E)) {
          return;
        }
      }
    }
    Operands.push_back(MOSOperand::createImm(STI, Val, S, E));
  }

  enum ExpressionType {
    ExprTypeOther,
    ExprTypeImmediate,
    ExprTypeAddress
  };

  bool tryParseRelocExpression(OperandVector &Operands, ExpressionType EType) {
    bool IsNegated = false;
    MOSMCExpr::VariantKind ModifierKind = MOSMCExpr::VK_MOS_NONE;

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
          IsNegated = true;
        } else {
          assert(CurTok == AsmToken::Plus);
          IsNegated = false;
        }

        // Eat the sign
        if (CurTok == AsmToken::Minus || CurTok == AsmToken::Plus)
          Parser.Lex();
      }
    }

    /*
    Shorthands for addressing modes conform to WDC's 65816 standard:

    #<addr == mos16lo(addr)
    #>addr == mos16hi(addr)
    #^addr == mos24bank(addr)

     <addr == mos8(addr)
     |addr == mos16(addr)
     !addr == mos16(addr)
     >addr == mos24(addr)
    */
    MCExpr const *InnerExpression;
    if (EType == ExprTypeImmediate &&
        (Parser.getTok().getKind() == AsmToken::Less ||
         Parser.getTok().getKind() == AsmToken::Greater ||
         Parser.getTok().getKind() == AsmToken::Caret)) {

      bool IsImm16 = false;
      switch (Parser.getTok().getKind()) {
      case AsmToken::Less:
        ModifierKind = IsImm16 ?
                       MOSMCExpr::VK_MOS_ADDR16 :
                       MOSMCExpr::VK_MOS_ADDR16_LO;
        break;
      case AsmToken::Greater:
        ModifierKind = IsImm16 ?
                       MOSMCExpr::VK_MOS_ADDR24_SEGMENT :
                       MOSMCExpr::VK_MOS_ADDR16_HI;
        break;
      case AsmToken::Caret:
        ModifierKind = MOSMCExpr::VK_MOS_ADDR24_BANK;
        break;
      default:
        assert(false);
      }

      Parser.Lex();

      if (getParser().parseExpression(InnerExpression))
        return true;
    } else if (EType == ExprTypeAddress &&
               (Parser.getTok().getKind() == AsmToken::Less ||
                Parser.getTok().getKind() == AsmToken::Greater ||
                Parser.getTok().getKind() == AsmToken::Pipe ||
                Parser.getTok().getKind() == AsmToken::Exclaim)) {

      switch (Parser.getTok().getKind()) {
      case AsmToken::Less:
        ModifierKind = MOSMCExpr::VK_MOS_ADDR8;
        break;
      case AsmToken::Exclaim:
      case AsmToken::Pipe:
        ModifierKind = MOSMCExpr::VK_MOS_ADDR16;
        break;
      case AsmToken::Greater:
        ModifierKind = MOSMCExpr::VK_MOS_ADDR24;
        break;
      default:
        assert(false);
      }

      Parser.Lex();

      if (getParser().parseExpression(InnerExpression))
        return true;

    } else {
      // Check if we have a target specific modifier (lo8, hi8, &c)
      if (Parser.getTok().getKind() != AsmToken::Identifier ||
          Parser.getLexer().peekTok().getKind() != AsmToken::LParen) {
        // Not a reloc expr
        return true;
      }
      StringRef ModifierName = Parser.getTok().getString();
      ModifierKind = MOSMCExpr::getKindByName(ModifierName.str(),
                                              EType != ExprTypeAddress);

      if (ModifierKind != MOSMCExpr::VK_MOS_NONE) {
        Parser.Lex();
        Parser.Lex(); // Eat modifier name and parenthesis
        if (Parser.getTok().getString() == GenerateStubs &&
            Parser.getTok().getKind() == AsmToken::Identifier) {
          std::string GSModName = ModifierName.str() + "_" + GenerateStubs;
          ModifierKind = MOSMCExpr::getKindByName(GSModName,
                                                  EType != ExprTypeAddress);
          if (ModifierKind != MOSMCExpr::VK_MOS_NONE) {
            Parser.Lex(); // Eat gs modifier name
          }
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
    }

    MCExpr const *Expression = MOSMCExpr::create(ModifierKind, InnerExpression,
                                                 IsNegated, getContext());

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    pushExpr(Operands, Expression, S, E);

    return false;
  }

  bool tryParseExpr(OperandVector &Operands, ExpressionType EType,
                    StringRef ErrorMsg) {
    if (!tryParseRelocExpression(Operands, EType)) {
      return false;
    }

    MCExpr const *Expression;
    SMLoc S = getLexer().getLoc();
    SMLoc E = getLexer().getTok().getEndLoc();
    if (getParser().parseExpression(Expression)) {
      Parser.eatToEndOfStatement();
      return Error(getLexer().getLoc(), ErrorMsg);
    }
    pushExpr(Operands, Expression, S, E);
    return false;
  }

  ParseStatus tryParseRegister(MCRegister &Reg, SMLoc &StartLoc,
                               SMLoc &EndLoc) override {
    std::string AnyCase(StartLoc.getPointer(),
                        EndLoc.getPointer() - StartLoc.getPointer());
    std::transform(AnyCase.begin(), AnyCase.end(), AnyCase.begin(),
                   [](unsigned char C) { return std::tolower(C); });
    StringRef RegisterName(AnyCase.c_str(), AnyCase.size());
    Reg = MatchRegisterName(RegisterName);
    if (Reg == 0) {
      // If the user has requested to ignore short register names, then ignore
      // them
      if (getSTI().getFeatureBits()[MOS::FeatureAltRegisterNamesOnly] ==
          false) {
        Reg = MatchRegisterAltName(RegisterName);
      }
    }
    return (Reg != 0) ? ParseStatus::Success : ParseStatus::NoMatch;
  }

  ParseStatus tryParseRegister(OperandVector &Operands) {
    MCRegister Reg = 0;
    SMLoc S = getLexer().getLoc();
    SMLoc E = getLexer().getTok().getEndLoc();
    if (tryParseRegister(Reg, S, E).isSuccess()) {
      Operands.push_back(MOSOperand::createReg(STI, Reg, S, E));
      return ParseStatus::Success;
    }
    return ParseStatus::NoMatch;
  }

  // Parse only registers that can be considered parameters to real MOS
  // instructions.  The instruction parser considers a, x, y, z, and s to be
  // strings, not registers, so make a point of filtering those cases out
  // of what's acceptable.
  ParseStatus tryParseAsmParamRegClass(OperandVector &Operands) {
    SMLoc S = getLexer().getLoc();

    const char *LowerStr =
        StringSwitch<const char *>(getLexer().getTok().getString())
            .CaseLower("a", "a")
            .CaseLower("x", "x")
            .CaseLower("y", "y")
            .CaseLower("z", "z")
            .CaseLower("s", "s")
            .CaseLower("sp", "s")
            .CaseLower("r", "r")     // 65EL02
            .CaseLower("rp", "r")    // 65EL02
            .CaseLower("ya", "ya")   // SPC700
            .CaseLower("c", "c")     // SPC700
            .CaseLower("psw", "psw") // SPC700
            .Default(nullptr);
    if (LowerStr != nullptr) {
      Operands.push_back(MOSOperand::createToken(STI, LowerStr, S));
      return ParseStatus::Success;
    }

    MCRegister Reg = 0;
    SMLoc E = getLexer().getTok().getEndLoc();
    if (tryParseRegister(Reg, S, E).isSuccess()) {
      Operands.push_back(MOSOperand::createReg(STI, Reg, S, E));
      return ParseStatus::Success;
    }
    return ParseStatus::NoMatch;
  }

  bool ParseInstruction(ParseInstructionInfo & /*Info*/, StringRef Mnemonic,
                        SMLoc NameLoc, OperandVector &Operands) override {
    /*
    On 65xx family instructions, mnemonics and addressing modes take the form:

    mnemonic (#)expr
    mnemonic [(]expr[),xy]*
    mnemonic a

    65816 and 45GS02 only:
    mnemonic [(]expr[),sxyz]*
    mnemonic \[ expr \]

    SPC700:
    mnemonic expr.bit
    mnemonic [(]expr[)]+[xy]*
    mnemonic \[ expr \]

    Any constant may be prefixed by a $, indicating that it is a hex constant.
    Such onstants can appear anywhere an integer appears in an expr, so expr
    parsing needs to take that into account.

    Handle all these cases, fairly loosely, and let tablegen sort out what's
    what.

    */
    // First, the mnemonic goes on the stack.
    Operands.push_back(MOSOperand::createToken(STI, Mnemonic, NameLoc));
    AsmToken::TokenKind RightHandSide = AsmToken::Eof;
    while (getLexer().isNot(AsmToken::EndOfStatement) &&
           getLexer().isNot(AsmToken::Eof)) {
      // Handle SPC700-specific syntax quirks.
      if (STI.hasFeature(MOS::FeatureSPC700)) {
        // Handle bit indexes ($xx.n).
        if (getLexer().is(AsmToken::Dot)) {
          eatThatToken(Operands);
          if (!tryParseExpr(Operands, ExprTypeOther,
                            "bit index must be an expression evaluating "
                            "to a value between 0 and 7 inclusive")) {
            continue;
          }
        }
        // Handle /.
        if (getLexer().is(AsmToken::Slash)) {
          eatThatToken(Operands);
          continue;
        }
        // Handle +, +x, +y.
        if (getLexer().is(AsmToken::Plus)) {
          eatThatToken(Operands);
          if (tryParseAsmParamRegClass(Operands).isSuccess()) {
            Parser.Lex();
          }
          continue;
        }
      }
      // Handle special characters.
      if (getLexer().is(AsmToken::Hash)) {
        eatThatToken(Operands);
        if (!tryParseExpr(Operands, ExprTypeImmediate,
                          STI.hasW65816Or65EL02() ?
                          "immediate operand must be an expression evaluating "
                          "to a value between 0 and 65535 inclusive" :
                          "immediate operand must be an expression evaluating "
                          "to a value between 0 and 255 inclusive")) {
          continue;
        }
      }
      // Handle parentheses and brackets.
      if (getLexer().is(AsmToken::LParen)) {
        eatThatToken(Operands);
        // Handle SPC700 (x), (y)
        if (STI.hasFeature(MOS::FeatureSPC700)) {
          if (tryParseAsmParamRegClass(Operands).isSuccess()) {
            Parser.Lex();
            RightHandSide = AsmToken::RParen;
            continue;
          }
        }
        if (!tryParseExpr(Operands, ExprTypeAddress,
                          "expression expected after left parenthesis")) {
          RightHandSide = AsmToken::RParen;
          continue;
        }
      }
      if ((STI.hasFeature(MOS::FeatureW65816) ||
           STI.hasFeature(MOS::FeatureSPC700) ||
           STI.hasFeature(MOS::Feature45GS02)) &&
          getLexer().is(AsmToken::LBrac)) {
        eatThatToken(Operands);
        if (!tryParseExpr(Operands, ExprTypeAddress,
                          "expression expected after left bracket")) {
          RightHandSide = AsmToken::RBrac;
          continue;
        }
      }
      if (RightHandSide != AsmToken::Eof && getLexer().is(RightHandSide)) {
        eatThatToken(Operands);
        RightHandSide = AsmToken::Eof;
        continue;
      }
      // I don't know what LLVM has against commas, but for some reason
      // TableGen makes an effort to ignore them during parsing.  So,
      // strangely enough, we have to throw out commas too, even though
      // they have semantic meaning on MOS platforms.
      if (getLexer().is(AsmToken::Comma)) {
        Lex();
        continue;
      }

      // We'll only ever see a register name on the third or later parameter.
      if (tryParseAsmParamRegClass(Operands).isSuccess()) {
        Parser.Lex();
        continue;
      }

      if (!tryParseExpr(Operands, ExprTypeAddress, "expression expected")) {
        continue;
      }

      // Okay then, you're a token.  Hope you're happy.
      eatThatToken(Operands);
    }
    Parser.Lex(); // Consume the EndOfStatement
    return false;
  }

  bool parseRegister(MCRegister &Reg, SMLoc &StartLoc, SMLoc &EndLoc) override {
    return !tryParseRegister(Reg, StartLoc, EndLoc).isSuccess();
  }

}; // class MOSAsmParser

#define GET_MATCHER_IMPLEMENTATION
#include "MOSGenAsmMatcher.inc"

extern "C" LLVM_EXTERNAL_VISIBILITY void
LLVMInitializeMOSAsmParser() { // NOLINT
  RegisterMCAsmParser<MOSAsmParser> X(getTheMOSTarget());
}

} // namespace llvm
