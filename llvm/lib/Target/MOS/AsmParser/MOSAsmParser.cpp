//===---- MOSAsmParser.cpp - Parse MOS assembly to MCInst instructions ----===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the MOSAsmParser class, which parses MOS assembly
// language into MCInst instructions. It supports the full 65xx family:
//   - 6502, 65C02, 65816 (WDC)
//   - SPC700 (Sony/Nintendo)
//   - HUC6280 (NEC/Hudson)
//   - 45GS02, 65CE02, 65EL02 (CSG/CMD variants)
//
// Addressing mode syntax follows WDC conventions with extensions for
// variant-specific features like SPC700's bit addressing and 65816's
// 24-bit long addressing.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MOSFixupKinds.h"
#include "MCTargetDesc/MOSMCELFStreamer.h"
#include "MCTargetDesc/MOSMCExpr.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MCTargetDesc/MOSTargetStreamer.h"
#include "MOS.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCParser/AsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mos-asm-parser"

namespace llvm {

#define GET_REGISTER_MATCHER
#include "MOSGenAsmMatcher.inc"

//===----------------------------------------------------------------------===//
// MOSOperand - Parsed assembly operand
//===----------------------------------------------------------------------===//

/// Represents a parsed MOS assembly operand: token, register, or immediate.
class MOSOperand : public MCParsedAsmOperand {
public:
  enum KindTy { k_Token, k_Register, k_Immediate };

private:
  KindTy Kind;
  SMLoc StartLoc, EndLoc;
  const MCSubtargetInfo &STI;

  union {
    StringRef Tok;
    unsigned Reg;
    const MCExpr *Imm;
  };

public:
  // Constructors
  MOSOperand(const MCSubtargetInfo &STI, StringRef Tok, SMLoc Loc)
      : Kind(k_Token), StartLoc(Loc), EndLoc(Loc), STI(STI), Tok(Tok) {}

  MOSOperand(const MCSubtargetInfo &STI, unsigned Reg, SMLoc S, SMLoc E)
      : Kind(k_Register), StartLoc(S), EndLoc(E), STI(STI), Reg(Reg) {}

  MOSOperand(const MCSubtargetInfo &STI, const MCExpr *Imm, SMLoc S, SMLoc E)
      : Kind(k_Immediate), StartLoc(S), EndLoc(E), STI(STI), Imm(Imm) {}

  // Factory methods
  static std::unique_ptr<MOSOperand> createToken(const MCSubtargetInfo &STI,
                                                 StringRef Tok, SMLoc Loc) {
    return std::make_unique<MOSOperand>(STI, Tok, Loc);
  }

  static std::unique_ptr<MOSOperand> createReg(const MCSubtargetInfo &STI,
                                               unsigned Reg, SMLoc S, SMLoc E) {
    return std::make_unique<MOSOperand>(STI, Reg, S, E);
  }

  static std::unique_ptr<MOSOperand>
  createImm(const MCSubtargetInfo &STI, const MCExpr *Imm, SMLoc S, SMLoc E) {
    return std::make_unique<MOSOperand>(STI, Imm, S, E);
  }

  // Kind queries
  bool isToken() const override { return Kind == k_Token; }
  bool isReg() const override { return Kind == k_Register; }
  bool isImm() const override { return Kind == k_Immediate; }
  bool isMem() const override { return false; }

  // Accessors
  SMLoc getStartLoc() const override { return StartLoc; }
  SMLoc getEndLoc() const override { return EndLoc; }

  StringRef getToken() const {
    assert(isToken());
    return Tok;
  }

  MCRegister getReg() const override {
    assert(isReg());
    return Reg;
  }

  const MCExpr *getImm() const {
    assert(isImm());
    return Imm;
  }

  // Size predicates for instruction matching
  template <int64_t Low, int64_t High> bool isImmInRange() const {
    if (!isImm())
      return false;

    // MOS-specific modifier: check if result fits in range
    if (const auto *ME = dyn_cast<MOSMCExpr>(Imm)) {
      const MOS::Fixups FixupKind = ME->getFixupKind();

      // Imm16 modifier requires 16-bit range
      if (FixupKind == MOS::Imm16 && High < 0xFFFF)
        return false;

      const MCFixupKindInfo &Info =
          MOSFixupKinds::getFixupKindInfo(FixupKind, nullptr);
      int64_t MaxValue = (1LL << Info.TargetSize) - 1;

      // Try to evaluate constant now
      int64_t Value;
      if (ME->evaluateAsConstant(Value) && Value > 0)
        return Value <= MaxValue;

      return MaxValue <= High;
    }

    // Symbol reference: defer to linker
    if (isa<MCSymbolRefExpr>(Imm))
      return true;

    // Constant: check range directly
    if (const auto *CE = dyn_cast<MCConstantExpr>(Imm))
      return CE->getValue() >= Low && CE->getValue() <= High;

    // Other expression types: accept optimistically
    return true;
  }

  bool isImm3() const { return isImmInRange<0, 7>(); }
  bool isImm4() const { return isImmInRange<0, 15>(); }
  bool isImm8() const { return isImmInRange<0, 255>(); }
  bool isImm16() const { return isImmInRange<0, 65535>(); }
  bool isImm24() const { return isImmInRange<0, 0xFFFFFF>(); }

  bool isPCRel8() const { return isImm8(); }
  bool isPCRel16() const { return isImm16(); }

  bool isAddr8() const {
    // HUC6280 has zero page at 0x2000-0x20FF
    if (STI.hasFeature(MOS::FeatureHUC6280) && isImm()) {
      if (const auto *CE = dyn_cast<MCConstantExpr>(Imm)) {
        int64_t Value = CE->getValue();
        return Value >= 0x2000 && Value <= 0x20FF;
      }
    }
    return isImm8();
  }

  bool isAddr13() const { return isImmInRange<0, 0x1FFF>(); }
  bool isAddr16() const { return isImm16(); }
  bool isAddr24() const { return isImm24(); }

  // Operand rendering for MCInst
  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid operand count");
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid operand count");
    if (const auto *CE = dyn_cast<MCConstantExpr>(Imm))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Imm));
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

  void print(raw_ostream &O, const MCAsmInfo &MAI) const override {
    switch (Kind) {
    case k_Token:
      O << "Token: \"" << Tok << "\"";
      break;
    case k_Register:
      O << "Register: " << Reg;
      break;
    case k_Immediate:
      O << "Immediate: ";
      MAI.printExpr(O, *Imm);
      break;
    }
    O << "\n";
  }
};

//===----------------------------------------------------------------------===//
// MOSAsmParser - Main parser class
//===----------------------------------------------------------------------===//

// Forward declaration - defined in generated MOSGenAsmMatcher.inc
static const char *getSubtargetFeatureName(uint64_t Val);

class MOSAsmParser : public MCTargetAsmParser {
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
      : MCTargetAsmParser(Options, STI, MII), Parser(Parser) {
    MCAsmParserExtension::Initialize(Parser);
    MRI = getContext().getRegisterInfo();

    // Set up directive aliases for common assembler conventions
    Parser.addAliasForDirective(".hword", ".byte");
    Parser.addAliasForDirective(".word", ".2byte");
    Parser.addAliasForDirective(".dword", ".4byte");
    Parser.addAliasForDirective(".xword", ".8byte");

    setAvailableFeatures(ComputeAvailableFeatures(getSTI().getFeatureBits()));
  }

  // Convenience accessors
  MCAsmParser &getParser() const { return Parser; }
  AsmLexer &getLexer() const { return Parser.getLexer(); }

  // Feature queries for cleaner conditionals
  bool isSPC700() const { return getSTI().hasFeature(MOS::FeatureSPC700); }
  bool is65816() const { return getSTI().hasFeature(MOS::FeatureW65816); }
  bool is45GS02() const { return getSTI().hasFeature(MOS::Feature45GS02); }
  bool is65EL02() const { return getSTI().hasFeature(MOS::Feature65EL02); }
  bool hasLongIndirect() const { return is65816() || isSPC700() || is45GS02(); }
  bool has16BitImmediate() const { return is65816() || is65EL02(); }

  //===--------------------------------------------------------------------===//
  // Token/operand helpers
  //===--------------------------------------------------------------------===//

  void addToken(OperandVector &Operands, StringRef Str) {
    Operands.push_back(
        MOSOperand::createToken(getSTI(), Str, Parser.getTok().getLoc()));
  }

  void consumeToken(OperandVector &Operands) {
    addToken(Operands, Parser.getTok().getString());
    Parser.Lex();
  }

  //===--------------------------------------------------------------------===//
  // Register parsing
  //===--------------------------------------------------------------------===//

  /// Try to parse a register from the current token.
  ParseStatus tryParseRegister(MCRegister &Reg, SMLoc &StartLoc,
                               SMLoc &EndLoc) override {
    StringRef Name = Parser.getTok().getString();
    std::string LowerName = Name.lower();

    Reg = MatchRegisterName(LowerName);
    if (Reg == 0 && !getSTI().hasFeature(MOS::FeatureAltRegisterNamesOnly)) {
      Reg = MatchRegisterAltName(LowerName);
    }

    if (Reg != 0) {
      StartLoc = Parser.getTok().getLoc();
      EndLoc = Parser.getTok().getEndLoc();
      return ParseStatus::Success;
    }
    return ParseStatus::NoMatch;
  }

  bool parseRegister(MCRegister &Reg, SMLoc &StartLoc, SMLoc &EndLoc) override {
    return !tryParseRegister(Reg, StartLoc, EndLoc).isSuccess();
  }

  /// Parse registers that appear as instruction operands (a, x, y, etc.)
  /// These are represented as tokens, not register operands, because
  /// TableGen's instruction matching expects them that way.
  ParseStatus tryParseRegisterToken(OperandVector &Operands) {
    StringRef Name = Parser.getTok().getString();

    // Map register names to canonical lowercase form
    const char *Canonical = StringSwitch<const char *>(Name)
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

    if (Canonical) {
      addToken(Operands, Canonical);
      return ParseStatus::Success;
    }

    // Also try imaginary registers (rc0, rs0, etc.)
    MCRegister Reg;
    SMLoc S, E;
    if (tryParseRegister(Reg, S, E).isSuccess()) {
      Operands.push_back(MOSOperand::createReg(getSTI(), Reg, S, E));
      return ParseStatus::Success;
    }

    return ParseStatus::NoMatch;
  }

  //===--------------------------------------------------------------------===//
  // Expression and modifier parsing
  //===--------------------------------------------------------------------===//

  /// Context for modifier interpretation - immediate vs address have different
  /// meanings for the same symbols.
  enum ExprContext {
    Ctx_Immediate, // After #: < means low byte, > means high byte
    Ctx_Address,   // Memory operand: < means ZP, > means 24-bit (65816)
    Ctx_Other      // Bit index, etc.
  };

  /// Parse an address modifier prefix (< > ^ ! |) and return the variant kind.
  /// Returns VK_NONE if no modifier present.
  MOSMCExpr::VariantKind parseModifierPrefix(ExprContext Ctx) {
    AsmToken::TokenKind Tok = Parser.getTok().getKind();

    if (Ctx == Ctx_Immediate) {
      // Immediate context: #<expr, #>expr, #^expr
      switch (Tok) {
      case AsmToken::Less:
        Parser.Lex();
        return MOSMCExpr::VK_ADDR16_LO;
      case AsmToken::Greater:
        Parser.Lex();
        return MOSMCExpr::VK_ADDR16_HI;
      case AsmToken::Caret:
        Parser.Lex();
        return MOSMCExpr::VK_ADDR24_BANK;
      default:
        return MOSMCExpr::VK_NONE;
      }
    }

    if (Ctx == Ctx_Address) {
      // Address context: <addr (ZP), !addr or |addr (16-bit), >addr (24-bit)
      switch (Tok) {
      case AsmToken::Less:
        Parser.Lex();
        return MOSMCExpr::VK_ADDR8;
      case AsmToken::Exclaim:
      case AsmToken::Pipe:
        Parser.Lex();
        return MOSMCExpr::VK_ADDR16;
      case AsmToken::Greater:
        Parser.Lex();
        return MOSMCExpr::VK_ADDR24;
      default:
        return MOSMCExpr::VK_NONE;
      }
    }

    return MOSMCExpr::VK_NONE;
  }

  /// Parse a function-style modifier like mos16lo(expr).
  /// Returns true on error, false on success or if no modifier found.
  bool tryParseFunctionModifier(OperandVector &Operands, ExprContext Ctx) {
    // Must be: identifier ( expression )
    if (Parser.getTok().getKind() != AsmToken::Identifier)
      return true;
    if (Parser.getLexer().peekTok().getKind() != AsmToken::LParen)
      return true;

    SMLoc ModLoc = Parser.getTok().getLoc();
    StringRef ModName = Parser.getTok().getString();

    MOSMCExpr::VariantKind VK =
        MOSMCExpr::getKindByName(ModName.str(), Ctx != Ctx_Address);
    if (VK == MOSMCExpr::VK_NONE)
      return Error(ModLoc, "unknown modifier '" + ModName + "'");

    Parser.Lex(); // Eat modifier name
    Parser.Lex(); // Eat '('

    // Check for optional 'gs' suffix (generate stubs)
    if (Parser.getTok().getString().equals_insensitive("gs") &&
        Parser.getTok().getKind() == AsmToken::Identifier) {
      std::string GSName = (ModName + "_gs").str();
      MOSMCExpr::VariantKind GSVK =
          MOSMCExpr::getKindByName(GSName, Ctx != Ctx_Address);
      if (GSVK != MOSMCExpr::VK_NONE) {
        VK = GSVK;
        Parser.Lex(); // Eat 'gs'
      }
    }

    const MCExpr *Inner;
    if (Parser.parseExpression(Inner))
      return true;

    if (Parser.getTok().getKind() != AsmToken::RParen)
      return Error(Parser.getTok().getLoc(), "expected ')'");
    Parser.Lex(); // Eat ')'

    SMLoc EndLoc =
        SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    const MCExpr *Expr =
        MOSMCExpr::create(VK, Inner, /*Negated=*/false, getContext());
    Operands.push_back(MOSOperand::createImm(getSTI(), Expr, ModLoc, EndLoc));
    return false;
  }

  /// Handle SPC700's expr+X and expr+Y addressing modes.
  /// On SPC700, an expression like "addr+x" should be parsed as the address
  /// "addr" followed by the token "+x", not as the sum addr+x.
  void pushExprWithSPC700Check(OperandVector &Operands, const MCExpr *Expr,
                               SMLoc S, SMLoc E) {
    if (!isSPC700()) {
      Operands.push_back(MOSOperand::createImm(getSTI(), Expr, S, E));
      return;
    }

    // Check for binary expression with +X or +Y on the RHS
    auto SplitIfIndexed = [&](const MCExpr *LHS,
                              const MCBinaryExpr *BE) -> bool {
      if (BE->getOpcode() != MCBinaryExpr::Add)
        return false;

      const auto *RHS = dyn_cast<MCSymbolRefExpr>(BE->getRHS());
      if (!RHS)
        return false;

      StringRef Name = RHS->getSymbol().getName();
      StringRef CanonicalName;
      if (Name.equals_insensitive("x"))
        CanonicalName = "x";
      else if (Name.equals_insensitive("y"))
        CanonicalName = "y";
      else
        return false;

      // Split: push LHS as immediate, then "+" and register as tokens
      Operands.push_back(MOSOperand::createImm(getSTI(), LHS, S, E));
      Operands.push_back(MOSOperand::createToken(getSTI(), "+", BE->getLoc()));
      Operands.push_back(
          MOSOperand::createToken(getSTI(), CanonicalName, RHS->getLoc()));
      return true;
    };

    // Check for mos...(expr+x) pattern
    if (const auto *ME = dyn_cast<MOSMCExpr>(Expr)) {
      if (const auto *BE = dyn_cast<MCBinaryExpr>(ME->getSubExpr())) {
        const MCExpr *WrappedLHS = MOSMCExpr::create(
            ME->getKind(), BE->getLHS(), ME->isNegated(), getContext());
        if (SplitIfIndexed(WrappedLHS, BE))
          return;
      }
    }

    // Check for plain expr+x pattern
    if (const auto *BE = dyn_cast<MCBinaryExpr>(Expr)) {
      if (SplitIfIndexed(BE->getLHS(), BE))
        return;
    }

    // No SPC700 special case applies
    Operands.push_back(MOSOperand::createImm(getSTI(), Expr, S, E));
  }

  /// Parse an expression with optional modifier prefix or function-style
  /// modifier.
  bool parseExpr(OperandVector &Operands, ExprContext Ctx, StringRef ErrMsg) {
    SMLoc S = Parser.getTok().getLoc();

    // Try function-style modifier first: mosXXX(expr)
    if (!tryParseFunctionModifier(Operands, Ctx))
      return false;

    // Check for prefix modifier: < > ^ ! |
    MOSMCExpr::VariantKind VK = parseModifierPrefix(Ctx);

    const MCExpr *Expr;
    if (Parser.parseExpression(Expr)) {
      Parser.eatToEndOfStatement();
      return Error(S, ErrMsg);
    }

    // Wrap in modifier if present
    if (VK != MOSMCExpr::VK_NONE)
      Expr = MOSMCExpr::create(VK, Expr, /*Negated=*/false, getContext());

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    pushExprWithSPC700Check(Operands, Expr, S, E);
    return false;
  }

  //===--------------------------------------------------------------------===//
  // Directive parsing
  //===--------------------------------------------------------------------===//

  bool ParseDirective(AsmToken DirectiveID) override {
    StringRef ID = DirectiveID.getIdentifier();

    if (ID.starts_with(".mos_addr_asciz"))
      return parseDirectiveAddrAsciz(DirectiveID.getLoc());
    if (ID.starts_with(".zeropage"))
      return parseDirectiveZeroPage();

    return true; // Not handled
  }

  bool parseDirectiveAddrAsciz(SMLoc Loc) {
    const MCExpr *AddrExpr;
    SMLoc AddrLoc = Parser.getTok().getLoc();
    if (Parser.checkForValidSection() || Parser.parseExpression(AddrExpr))
      return true;

    if (Parser.parseToken(AsmToken::Comma, "expected `, <char-count>`"))
      return true;

    SMLoc CountLoc = Parser.getTok().getLoc();
    int64_t CharCount;
    if (Parser.parseAbsoluteExpression(CharCount))
      return true;

    if (CharCount < 1 || CharCount > 8)
      return Error(CountLoc, "char count out of range [1,8]");

    // Constant: emit decimal string representation
    if (const auto *CE = dyn_cast<MCConstantExpr>(AddrExpr)) {
      std::string Str = itostr(CE->getValue());
      if (Str.size() > static_cast<size_t>(CharCount))
        return Error(AddrLoc, "out of range literal value");
      getStreamer().emitBytes(Str);
      getStreamer().emitBytes(
          StringRef("\0\0\0\0\0\0\0\0\0", CharCount - Str.size() + 1));
    } else {
      // Symbol: emit as relocatable expression
      const MCExpr *Expr = MOSMCExpr::create(MOSMCExpr::VK_ADDR_ASCIZ, AddrExpr,
                                             /*Negated=*/false, getContext());
      getStreamer().emitValue(Expr, CharCount + 1, Loc);
    }
    return false;
  }

  bool parseDirectiveZeroPage() {
    auto ParseOne = [&]() -> bool {
      SMLoc Loc = Parser.getTok().getLoc();
      StringRef Name;
      if (Parser.parseIdentifier(Name))
        return Error(Loc, "expected identifier");

      if (Parser.discardLTOSymbol(Name))
        return false;

      MCSymbol *Sym = getContext().getOrCreateSymbol(Name);
      MOSTargetStreamer &TS = static_cast<MOSTargetStreamer &>(
          *getParser().getStreamer().getTargetStreamer());
      if (!TS.emitDirectiveZeroPage(Sym))
        return Error(Loc, "unable to mark symbol as zero page");
      return false;
    };

    return parseMany(ParseOne);
  }

  //===--------------------------------------------------------------------===//
  // Primary expression hook for directives
  //===--------------------------------------------------------------------===//

  /// Override to handle WDC-style modifiers in .byte/.2byte/etc directives.
  bool parsePrimaryExpr(const MCExpr *&Res, SMLoc &EndLoc) override {
    // Check for < > ^ prefix modifiers
    AsmToken::TokenKind Tok = Parser.getTok().getKind();
    if (Tok == AsmToken::Less || Tok == AsmToken::Greater ||
        Tok == AsmToken::Caret) {

      MOSMCExpr::VariantKind VK;
      switch (Tok) {
      case AsmToken::Less:
        VK = MOSMCExpr::VK_ADDR16_LO;
        break;
      case AsmToken::Greater:
        VK = MOSMCExpr::VK_ADDR16_HI;
        break;
      case AsmToken::Caret:
        VK = MOSMCExpr::VK_ADDR24_BANK;
        break;
      default:
        llvm_unreachable("Already checked token kind");
      }

      Parser.Lex(); // Eat modifier

      const MCExpr *Inner;
      if (Parser.parseExpression(Inner))
        return true;

      Res = MOSMCExpr::create(VK, Inner, /*Negated=*/false, getContext());
      EndLoc = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
      return false;
    }

    // Handle function-style modifiers: mos16lo(expr), mos24bank(expr), etc.
    if (Parser.getTok().is(AsmToken::Identifier)) {
      StringRef Name = Parser.getTok().getString();
      MOSMCExpr::VariantKind VK =
          MOSMCExpr::getKindByName(Name, /*IsImmediate=*/false);

      if (VK != MOSMCExpr::VK_NONE &&
          Parser.getLexer().peekTok().is(AsmToken::LParen)) {
        Parser.Lex(); // Eat identifier
        Parser.Lex(); // Eat '('

        const MCExpr *Inner;
        if (Parser.parseExpression(Inner))
          return true;

        if (Parser.getTok().isNot(AsmToken::RParen))
          return Error(Parser.getTok().getLoc(),
                       "expected ')' after expression");
        Parser.Lex(); // Eat ')'

        Res = MOSMCExpr::create(VK, Inner, /*Negated=*/false, getContext());
        EndLoc =
            SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
        return false;
      }
    }

    // Parse normally
    if (MCTargetAsmParser::parsePrimaryExpr(Res, EndLoc))
      return true;

    // Handle constant@modifier syntax for directives
    if (Parser.getTok().getKind() == AsmToken::At && isa<MCConstantExpr>(Res)) {
      Parser.Lex(); // Eat '@'

      if (Parser.getTok().getKind() != AsmToken::Identifier)
        return Error(Parser.getTok().getLoc(),
                     "expected modifier name after '@'");

      StringRef ModName = Parser.getTok().getString();
      MOSMCExpr::VariantKind VK =
          MOSMCExpr::getKindByName(ModName.str(), /*IsImmediate=*/true);
      if (VK == MOSMCExpr::VK_NONE)
        return Error(Parser.getTok().getLoc(),
                     "unknown modifier '" + ModName + "'");

      Parser.Lex(); // Eat modifier name
      Res = MOSMCExpr::create(VK, Res, /*Negated=*/false, getContext());
      EndLoc = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    }

    return false;
  }

  //===--------------------------------------------------------------------===//
  // Instruction parsing
  //===--------------------------------------------------------------------===//

  bool parseInstruction(ParseInstructionInfo &Info, StringRef Mnemonic,
                        SMLoc NameLoc, OperandVector &Operands) override {
    // Add mnemonic as first operand
    Operands.push_back(MOSOperand::createToken(getSTI(), Mnemonic, NameLoc));

    // Track expected closing bracket/paren
    AsmToken::TokenKind ExpectedClose = AsmToken::Eof;

    while (!Parser.getTok().is(AsmToken::EndOfStatement) &&
           !Parser.getTok().is(AsmToken::Eof)) {

      // Skip commas (TableGen ignores them)
      if (Parser.getTok().is(AsmToken::Comma)) {
        Parser.Lex();
        continue;
      }

      // SPC700-specific syntax
      if (isSPC700()) {
        // Bit index: .n
        if (Parser.getTok().is(AsmToken::Dot)) {
          consumeToken(Operands);
          if (parseExpr(Operands, Ctx_Other, "bit index must be 0-7"))
            return true;
          continue;
        }

        // Bit complement: /
        if (Parser.getTok().is(AsmToken::Slash)) {
          consumeToken(Operands);
          continue;
        }

        // Indexed: +x, +y
        if (Parser.getTok().is(AsmToken::Plus)) {
          consumeToken(Operands);
          if (tryParseRegisterToken(Operands).isSuccess())
            Parser.Lex();
          continue;
        }
      }

      // Immediate: #expr
      if (Parser.getTok().is(AsmToken::Hash)) {
        consumeToken(Operands);
        if (parseExpr(Operands, Ctx_Immediate,
                      has16BitImmediate() ? "immediate must be 0-65535"
                                          : "immediate must be 0-255"))
          return true;
        continue;
      }

      // Indirect: (expr) or (expr,x) or (expr),y
      if (Parser.getTok().is(AsmToken::LParen)) {
        consumeToken(Operands);

        // SPC700: (x) and (y) are valid
        if (isSPC700() && tryParseRegisterToken(Operands).isSuccess()) {
          Parser.Lex();
          ExpectedClose = AsmToken::RParen;
          continue;
        }

        if (parseExpr(Operands, Ctx_Address, "expression expected after '('"))
          return true;
        ExpectedClose = AsmToken::RParen;
        continue;
      }

      // Long indirect: [expr] (65816, SPC700, 45GS02)
      if (hasLongIndirect() && Parser.getTok().is(AsmToken::LBrac)) {
        consumeToken(Operands);
        if (parseExpr(Operands, Ctx_Address, "expression expected after '['"))
          return true;
        ExpectedClose = AsmToken::RBrac;
        continue;
      }

      // Closing bracket/paren
      if (ExpectedClose != AsmToken::Eof && Parser.getTok().is(ExpectedClose)) {
        consumeToken(Operands);
        ExpectedClose = AsmToken::Eof;
        continue;
      }

      // Register operand (a, x, y, etc.)
      if (tryParseRegisterToken(Operands).isSuccess()) {
        Parser.Lex();
        continue;
      }

      // Address expression
      if (!parseExpr(Operands, Ctx_Address, "expression expected"))
        continue;

      // Unknown token - consume it and let TableGen figure it out
      consumeToken(Operands);
    }

    Parser.Lex(); // Consume EndOfStatement
    return false;
  }

  //===--------------------------------------------------------------------===//
  // Instruction matching and emission
  //===--------------------------------------------------------------------===//

  /// Report helpful diagnostics when instruction matching produces near-misses.
  /// This iterates through the NearMissInfo vector and generates specific
  /// error messages about what went wrong with each potential match.
  void reportNearMisses(SmallVectorImpl<NearMissInfo> &NearMisses, SMLoc IDLoc,
                        OperandVector &Operands) {
    // Collect unique error messages to avoid duplicates
    SmallVector<std::pair<SMLoc, std::string>, 4> Messages;
    SmallSet<std::string, 4> SeenMessages;
    bool ReportedTooFewOperands = false;

    for (const NearMissInfo &NM : NearMisses) {
      std::string Message;
      SMLoc Loc = IDLoc;

      switch (NM.getKind()) {
      case NearMissInfo::NearMissFeature: {
        // Missing CPU features
        const FeatureBitset &Missing = NM.getFeatures();
        raw_string_ostream OS(Message);
        OS << "instruction requires:";
        for (unsigned I = 0, E = Missing.size(); I != E; ++I)
          if (Missing.test(I))
            OS << ' ' << getSubtargetFeatureName(I);
        break;
      }

      case NearMissInfo::NearMissOperand: {
        // Wrong operand type
        unsigned OpIdx = NM.getOperandIndex();
        if (OpIdx < Operands.size())
          Loc = Operands[OpIdx]->getStartLoc();

        // Check for custom diagnostic messages based on operand error
        unsigned OperandError = NM.getOperandError();
        switch (OperandError) {
        case Match_InvalidAddr8:
          Message = "operand must be an 8-bit address";
          break;
        case Match_InvalidAddr16:
          Message = "operand must be a 16-bit address";
          break;
        case Match_InvalidPCRel8:
          Message = "operand must be an 8-bit PC-relative address";
          break;
        default:
          Message = "invalid operand for instruction";
          break;
        }
        break;
      }

      case NearMissInfo::NearMissTooFewOperands:
        if (!ReportedTooFewOperands) {
          Loc = Operands.back()->getEndLoc();
          Message = "too few operands for instruction";
          ReportedTooFewOperands = true;
        }
        break;

      case NearMissInfo::NearMissPredicate:
        // MOS doesn't use target predicates currently
        Message = "instruction not valid in current context";
        break;

      case NearMissInfo::NoNearMiss:
        continue;
      }

      // Avoid duplicate messages
      if (!Message.empty() && SeenMessages.insert(Message).second)
        Messages.push_back({Loc, std::move(Message)});
    }

    if (Messages.empty()) {
      // No specific near-misses found
      Error(IDLoc, "invalid instruction");
    } else if (Messages.size() == 1) {
      // Single near-miss - report it directly
      Error(Messages[0].first, Messages[0].second);
    } else {
      // Multiple near-misses - report generic error with notes
      Error(IDLoc,
            "invalid instruction, any one of the following would fix this:");
      for (const auto &[Loc, Msg] : Messages)
        Note(Loc, Msg);
    }
  }

  bool matchAndEmitInstruction(SMLoc Loc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override {
    MCInst Inst;
    SmallVector<NearMissInfo, 4> NearMisses;
    unsigned Result =
        MatchInstructionImpl(Operands, Inst, &NearMisses, MatchingInlineAsm);

    switch (Result) {
    case Match_Success:
      Inst.setLoc(Loc);
      Out.emitInstruction(Inst, getSTI());
      return false;

    case Match_MnemonicFail:
      return Error(Loc, "invalid instruction");

    case Match_NearMisses:
      reportNearMisses(NearMisses, Loc, Operands);
      return true;

    default:
      return Error(Loc, "unknown error matching instruction");
    }
  }
};

#define GET_SUBTARGET_FEATURE_NAME
#define GET_MATCHER_IMPLEMENTATION
#include "MOSGenAsmMatcher.inc"

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMOSAsmParser() {
  RegisterMCAsmParser<MOSAsmParser> X(getTheMOSTarget());
}

} // namespace llvm
