package ca.uwaterloo.bmuscede.ta;

import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.misc.*;
import org.antlr.v4.runtime.tree.*;
import java.util.List;
import java.util.Iterator;
import java.util.ArrayList;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast"})
public class TupleAttributeParser extends Parser {
	static { RuntimeMetaData.checkVersion("4.7", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		T__0=1, T__1=2, T__2=3, T__3=4, T__4=5, T__5=6, T__6=7, T__7=8, T__8=9, 
		T__9=10, T__10=11, T__11=12, T__12=13, NAME=14, WHITESPACE=15, NEWLINE=16, 
		COMMENT=17;
	public static final int
		RULE_root = 0, RULE_schema = 1, RULE_tupleSchema = 2, RULE_inheritLine = 3, 
		RULE_relLine = 4, RULE_inherit = 5, RULE_attributeSchema = 6, RULE_specialAttr = 7, 
		RULE_schemeRelAttr = 8, RULE_factList = 9, RULE_factEntry = 10, RULE_nodeEntry = 11, 
		RULE_relEntry = 12, RULE_attributeList = 13, RULE_attributeEntry = 14, 
		RULE_nodeAttr = 15, RULE_relAttr = 16, RULE_attributes = 17, RULE_attribute = 18, 
		RULE_val = 19, RULE_entry = 20, RULE_list = 21;
	public static final String[] ruleNames = {
		"root", "schema", "tupleSchema", "inheritLine", "relLine", "inherit", 
		"attributeSchema", "specialAttr", "schemeRelAttr", "factList", "factEntry", 
		"nodeEntry", "relEntry", "attributeList", "attributeEntry", "nodeAttr", 
		"relAttr", "attributes", "attribute", "val", "entry", "list"
	};

	private static final String[] _LITERAL_NAMES = {
		null, "'SCHEME TUPLE :'", "'$INHERIT'", "'SCHEME ATTRIBUTE :'", "'$ENTITY'", 
		"'('", "')'", "'FACT TUPLE :'", "'$INSTANCE'", "'FACT ATTRIBUTE :'", "'{'", 
		"'}'", "'='", "'\"'"
	};
	private static final String[] _SYMBOLIC_NAMES = {
		null, null, null, null, null, null, null, null, null, null, null, null, 
		null, null, "NAME", "WHITESPACE", "NEWLINE", "COMMENT"
	};
	public static final Vocabulary VOCABULARY = new VocabularyImpl(_LITERAL_NAMES, _SYMBOLIC_NAMES);

	/**
	 * @deprecated Use {@link #VOCABULARY} instead.
	 */
	@Deprecated
	public static final String[] tokenNames;
	static {
		tokenNames = new String[_SYMBOLIC_NAMES.length];
		for (int i = 0; i < tokenNames.length; i++) {
			tokenNames[i] = VOCABULARY.getLiteralName(i);
			if (tokenNames[i] == null) {
				tokenNames[i] = VOCABULARY.getSymbolicName(i);
			}

			if (tokenNames[i] == null) {
				tokenNames[i] = "<INVALID>";
			}
		}
	}

	@Override
	@Deprecated
	public String[] getTokenNames() {
		return tokenNames;
	}

	@Override

	public Vocabulary getVocabulary() {
		return VOCABULARY;
	}

	@Override
	public String getGrammarFileName() { return "TupleAttribute.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public ATN getATN() { return _ATN; }

	public TupleAttributeParser(TokenStream input) {
		super(input);
		_interp = new ParserATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}
	public static class RootContext extends ParserRuleContext {
		public SchemaContext schema() {
			return getRuleContext(SchemaContext.class,0);
		}
		public TerminalNode EOF() { return getToken(TupleAttributeParser.EOF, 0); }
		public List<FactListContext> factList() {
			return getRuleContexts(FactListContext.class);
		}
		public FactListContext factList(int i) {
			return getRuleContext(FactListContext.class,i);
		}
		public List<AttributeListContext> attributeList() {
			return getRuleContexts(AttributeListContext.class);
		}
		public AttributeListContext attributeList(int i) {
			return getRuleContext(AttributeListContext.class,i);
		}
		public RootContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_root; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterRoot(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitRoot(this);
		}
	}

	public final RootContext root() throws RecognitionException {
		RootContext _localctx = new RootContext(_ctx, getState());
		enterRule(_localctx, 0, RULE_root);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(44);
			schema();
			setState(49);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__6 || _la==T__8) {
				{
				setState(47);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case T__6:
					{
					setState(45);
					factList();
					}
					break;
				case T__8:
					{
					setState(46);
					attributeList();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(51);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(52);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class SchemaContext extends ParserRuleContext {
		public TupleSchemaContext tupleSchema() {
			return getRuleContext(TupleSchemaContext.class,0);
		}
		public AttributeSchemaContext attributeSchema() {
			return getRuleContext(AttributeSchemaContext.class,0);
		}
		public SchemaContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_schema; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterSchema(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitSchema(this);
		}
	}

	public final SchemaContext schema() throws RecognitionException {
		SchemaContext _localctx = new SchemaContext(_ctx, getState());
		enterRule(_localctx, 2, RULE_schema);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(55);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==T__0) {
				{
				setState(54);
				tupleSchema();
				}
			}

			setState(58);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==T__2) {
				{
				setState(57);
				attributeSchema();
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class TupleSchemaContext extends ParserRuleContext {
		public List<InheritLineContext> inheritLine() {
			return getRuleContexts(InheritLineContext.class);
		}
		public InheritLineContext inheritLine(int i) {
			return getRuleContext(InheritLineContext.class,i);
		}
		public List<RelLineContext> relLine() {
			return getRuleContexts(RelLineContext.class);
		}
		public RelLineContext relLine(int i) {
			return getRuleContext(RelLineContext.class,i);
		}
		public TupleSchemaContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_tupleSchema; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterTupleSchema(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitTupleSchema(this);
		}
	}

	public final TupleSchemaContext tupleSchema() throws RecognitionException {
		TupleSchemaContext _localctx = new TupleSchemaContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_tupleSchema);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(60);
			match(T__0);
			setState(65);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__1 || _la==NAME) {
				{
				setState(63);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case T__1:
					{
					setState(61);
					inheritLine();
					}
					break;
				case NAME:
					{
					setState(62);
					relLine();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(67);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class InheritLineContext extends ParserRuleContext {
		public List<InheritContext> inherit() {
			return getRuleContexts(InheritContext.class);
		}
		public InheritContext inherit(int i) {
			return getRuleContext(InheritContext.class,i);
		}
		public InheritLineContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_inheritLine; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterInheritLine(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitInheritLine(this);
		}
	}

	public final InheritLineContext inheritLine() throws RecognitionException {
		InheritLineContext _localctx = new InheritLineContext(_ctx, getState());
		enterRule(_localctx, 6, RULE_inheritLine);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(69); 
			_errHandler.sync(this);
			_alt = 1;
			do {
				switch (_alt) {
				case 1:
					{
					{
					setState(68);
					inherit();
					}
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				setState(71); 
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,6,_ctx);
			} while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER );
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class RelLineContext extends ParserRuleContext {
		public List<RelEntryContext> relEntry() {
			return getRuleContexts(RelEntryContext.class);
		}
		public RelEntryContext relEntry(int i) {
			return getRuleContext(RelEntryContext.class,i);
		}
		public RelLineContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_relLine; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterRelLine(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitRelLine(this);
		}
	}

	public final RelLineContext relLine() throws RecognitionException {
		RelLineContext _localctx = new RelLineContext(_ctx, getState());
		enterRule(_localctx, 8, RULE_relLine);
		try {
			int _alt;
			enterOuterAlt(_localctx, 1);
			{
			setState(74); 
			_errHandler.sync(this);
			_alt = 1;
			do {
				switch (_alt) {
				case 1:
					{
					{
					setState(73);
					relEntry();
					}
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				setState(76); 
				_errHandler.sync(this);
				_alt = getInterpreter().adaptivePredict(_input,7,_ctx);
			} while ( _alt!=2 && _alt!=org.antlr.v4.runtime.atn.ATN.INVALID_ALT_NUMBER );
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class InheritContext extends ParserRuleContext {
		public List<TerminalNode> NAME() { return getTokens(TupleAttributeParser.NAME); }
		public TerminalNode NAME(int i) {
			return getToken(TupleAttributeParser.NAME, i);
		}
		public InheritContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_inherit; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterInherit(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitInherit(this);
		}
	}

	public final InheritContext inherit() throws RecognitionException {
		InheritContext _localctx = new InheritContext(_ctx, getState());
		enterRule(_localctx, 10, RULE_inherit);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(78);
			match(T__1);
			setState(79);
			match(NAME);
			setState(80);
			match(NAME);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class AttributeSchemaContext extends ParserRuleContext {
		public List<NodeAttrContext> nodeAttr() {
			return getRuleContexts(NodeAttrContext.class);
		}
		public NodeAttrContext nodeAttr(int i) {
			return getRuleContext(NodeAttrContext.class,i);
		}
		public List<SpecialAttrContext> specialAttr() {
			return getRuleContexts(SpecialAttrContext.class);
		}
		public SpecialAttrContext specialAttr(int i) {
			return getRuleContext(SpecialAttrContext.class,i);
		}
		public List<SchemeRelAttrContext> schemeRelAttr() {
			return getRuleContexts(SchemeRelAttrContext.class);
		}
		public SchemeRelAttrContext schemeRelAttr(int i) {
			return getRuleContext(SchemeRelAttrContext.class,i);
		}
		public AttributeSchemaContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attributeSchema; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterAttributeSchema(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitAttributeSchema(this);
		}
	}

	public final AttributeSchemaContext attributeSchema() throws RecognitionException {
		AttributeSchemaContext _localctx = new AttributeSchemaContext(_ctx, getState());
		enterRule(_localctx, 12, RULE_attributeSchema);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(82);
			match(T__2);
			setState(88);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while ((((_la) & ~0x3f) == 0 && ((1L << _la) & ((1L << T__3) | (1L << T__4) | (1L << NAME))) != 0)) {
				{
				setState(86);
				_errHandler.sync(this);
				switch (_input.LA(1)) {
				case NAME:
					{
					setState(83);
					nodeAttr();
					}
					break;
				case T__3:
					{
					setState(84);
					specialAttr();
					}
					break;
				case T__4:
					{
					setState(85);
					schemeRelAttr();
					}
					break;
				default:
					throw new NoViableAltException(this);
				}
				}
				setState(90);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class SpecialAttrContext extends ParserRuleContext {
		public AttributesContext attributes() {
			return getRuleContext(AttributesContext.class,0);
		}
		public SpecialAttrContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_specialAttr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterSpecialAttr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitSpecialAttr(this);
		}
	}

	public final SpecialAttrContext specialAttr() throws RecognitionException {
		SpecialAttrContext _localctx = new SpecialAttrContext(_ctx, getState());
		enterRule(_localctx, 14, RULE_specialAttr);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(91);
			match(T__3);
			setState(92);
			attributes();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class SchemeRelAttrContext extends ParserRuleContext {
		public TerminalNode NAME() { return getToken(TupleAttributeParser.NAME, 0); }
		public AttributesContext attributes() {
			return getRuleContext(AttributesContext.class,0);
		}
		public SchemeRelAttrContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_schemeRelAttr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterSchemeRelAttr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitSchemeRelAttr(this);
		}
	}

	public final SchemeRelAttrContext schemeRelAttr() throws RecognitionException {
		SchemeRelAttrContext _localctx = new SchemeRelAttrContext(_ctx, getState());
		enterRule(_localctx, 16, RULE_schemeRelAttr);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(94);
			match(T__4);
			setState(95);
			match(NAME);
			setState(96);
			match(T__5);
			setState(97);
			attributes();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class FactListContext extends ParserRuleContext {
		public List<FactEntryContext> factEntry() {
			return getRuleContexts(FactEntryContext.class);
		}
		public FactEntryContext factEntry(int i) {
			return getRuleContext(FactEntryContext.class,i);
		}
		public FactListContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_factList; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterFactList(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitFactList(this);
		}
	}

	public final FactListContext factList() throws RecognitionException {
		FactListContext _localctx = new FactListContext(_ctx, getState());
		enterRule(_localctx, 18, RULE_factList);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(99);
			match(T__6);
			setState(103);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__7 || _la==NAME) {
				{
				{
				setState(100);
				factEntry();
				}
				}
				setState(105);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class FactEntryContext extends ParserRuleContext {
		public NodeEntryContext nodeEntry() {
			return getRuleContext(NodeEntryContext.class,0);
		}
		public RelEntryContext relEntry() {
			return getRuleContext(RelEntryContext.class,0);
		}
		public FactEntryContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_factEntry; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterFactEntry(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitFactEntry(this);
		}
	}

	public final FactEntryContext factEntry() throws RecognitionException {
		FactEntryContext _localctx = new FactEntryContext(_ctx, getState());
		enterRule(_localctx, 20, RULE_factEntry);
		try {
			setState(108);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case T__7:
				enterOuterAlt(_localctx, 1);
				{
				setState(106);
				nodeEntry();
				}
				break;
			case NAME:
				enterOuterAlt(_localctx, 2);
				{
				setState(107);
				relEntry();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class NodeEntryContext extends ParserRuleContext {
		public List<TerminalNode> NAME() { return getTokens(TupleAttributeParser.NAME); }
		public TerminalNode NAME(int i) {
			return getToken(TupleAttributeParser.NAME, i);
		}
		public NodeEntryContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_nodeEntry; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterNodeEntry(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitNodeEntry(this);
		}
	}

	public final NodeEntryContext nodeEntry() throws RecognitionException {
		NodeEntryContext _localctx = new NodeEntryContext(_ctx, getState());
		enterRule(_localctx, 22, RULE_nodeEntry);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(110);
			match(T__7);
			setState(111);
			match(NAME);
			setState(112);
			match(NAME);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class RelEntryContext extends ParserRuleContext {
		public List<TerminalNode> NAME() { return getTokens(TupleAttributeParser.NAME); }
		public TerminalNode NAME(int i) {
			return getToken(TupleAttributeParser.NAME, i);
		}
		public RelEntryContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_relEntry; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterRelEntry(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitRelEntry(this);
		}
	}

	public final RelEntryContext relEntry() throws RecognitionException {
		RelEntryContext _localctx = new RelEntryContext(_ctx, getState());
		enterRule(_localctx, 24, RULE_relEntry);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(114);
			match(NAME);
			setState(115);
			match(NAME);
			setState(116);
			match(NAME);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class AttributeListContext extends ParserRuleContext {
		public List<AttributeEntryContext> attributeEntry() {
			return getRuleContexts(AttributeEntryContext.class);
		}
		public AttributeEntryContext attributeEntry(int i) {
			return getRuleContext(AttributeEntryContext.class,i);
		}
		public AttributeListContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attributeList; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterAttributeList(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitAttributeList(this);
		}
	}

	public final AttributeListContext attributeList() throws RecognitionException {
		AttributeListContext _localctx = new AttributeListContext(_ctx, getState());
		enterRule(_localctx, 26, RULE_attributeList);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(118);
			match(T__8);
			setState(122);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==T__4 || _la==NAME) {
				{
				{
				setState(119);
				attributeEntry();
				}
				}
				setState(124);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class AttributeEntryContext extends ParserRuleContext {
		public NodeAttrContext nodeAttr() {
			return getRuleContext(NodeAttrContext.class,0);
		}
		public RelAttrContext relAttr() {
			return getRuleContext(RelAttrContext.class,0);
		}
		public AttributeEntryContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attributeEntry; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterAttributeEntry(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitAttributeEntry(this);
		}
	}

	public final AttributeEntryContext attributeEntry() throws RecognitionException {
		AttributeEntryContext _localctx = new AttributeEntryContext(_ctx, getState());
		enterRule(_localctx, 28, RULE_attributeEntry);
		try {
			setState(127);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case NAME:
				enterOuterAlt(_localctx, 1);
				{
				setState(125);
				nodeAttr();
				}
				break;
			case T__4:
				enterOuterAlt(_localctx, 2);
				{
				setState(126);
				relAttr();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class NodeAttrContext extends ParserRuleContext {
		public TerminalNode NAME() { return getToken(TupleAttributeParser.NAME, 0); }
		public AttributesContext attributes() {
			return getRuleContext(AttributesContext.class,0);
		}
		public NodeAttrContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_nodeAttr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterNodeAttr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitNodeAttr(this);
		}
	}

	public final NodeAttrContext nodeAttr() throws RecognitionException {
		NodeAttrContext _localctx = new NodeAttrContext(_ctx, getState());
		enterRule(_localctx, 30, RULE_nodeAttr);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(129);
			match(NAME);
			setState(130);
			attributes();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class RelAttrContext extends ParserRuleContext {
		public List<TerminalNode> NAME() { return getTokens(TupleAttributeParser.NAME); }
		public TerminalNode NAME(int i) {
			return getToken(TupleAttributeParser.NAME, i);
		}
		public AttributesContext attributes() {
			return getRuleContext(AttributesContext.class,0);
		}
		public RelAttrContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_relAttr; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterRelAttr(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitRelAttr(this);
		}
	}

	public final RelAttrContext relAttr() throws RecognitionException {
		RelAttrContext _localctx = new RelAttrContext(_ctx, getState());
		enterRule(_localctx, 32, RULE_relAttr);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(132);
			match(T__4);
			setState(133);
			match(NAME);
			setState(134);
			match(NAME);
			setState(135);
			match(NAME);
			setState(136);
			match(T__5);
			setState(137);
			attributes();
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class AttributesContext extends ParserRuleContext {
		public List<AttributeContext> attribute() {
			return getRuleContexts(AttributeContext.class);
		}
		public AttributeContext attribute(int i) {
			return getRuleContext(AttributeContext.class,i);
		}
		public AttributesContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attributes; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterAttributes(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitAttributes(this);
		}
	}

	public final AttributesContext attributes() throws RecognitionException {
		AttributesContext _localctx = new AttributesContext(_ctx, getState());
		enterRule(_localctx, 34, RULE_attributes);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(139);
			match(T__9);
			setState(143);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==NAME) {
				{
				{
				setState(140);
				attribute();
				}
				}
				setState(145);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(146);
			match(T__10);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class AttributeContext extends ParserRuleContext {
		public TerminalNode NAME() { return getToken(TupleAttributeParser.NAME, 0); }
		public ValContext val() {
			return getRuleContext(ValContext.class,0);
		}
		public AttributeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_attribute; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterAttribute(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitAttribute(this);
		}
	}

	public final AttributeContext attribute() throws RecognitionException {
		AttributeContext _localctx = new AttributeContext(_ctx, getState());
		enterRule(_localctx, 36, RULE_attribute);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(148);
			match(NAME);
			setState(151);
			_errHandler.sync(this);
			_la = _input.LA(1);
			if (_la==T__11) {
				{
				setState(149);
				match(T__11);
				setState(150);
				val();
				}
			}

			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ValContext extends ParserRuleContext {
		public TerminalNode NAME() { return getToken(TupleAttributeParser.NAME, 0); }
		public EntryContext entry() {
			return getRuleContext(EntryContext.class,0);
		}
		public ListContext list() {
			return getRuleContext(ListContext.class,0);
		}
		public ValContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_val; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterVal(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitVal(this);
		}
	}

	public final ValContext val() throws RecognitionException {
		ValContext _localctx = new ValContext(_ctx, getState());
		enterRule(_localctx, 38, RULE_val);
		try {
			setState(156);
			_errHandler.sync(this);
			switch (_input.LA(1)) {
			case NAME:
				enterOuterAlt(_localctx, 1);
				{
				setState(153);
				match(NAME);
				}
				break;
			case T__12:
				enterOuterAlt(_localctx, 2);
				{
				setState(154);
				entry();
				}
				break;
			case T__4:
				enterOuterAlt(_localctx, 3);
				{
				setState(155);
				list();
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class EntryContext extends ParserRuleContext {
		public List<TerminalNode> NAME() { return getTokens(TupleAttributeParser.NAME); }
		public TerminalNode NAME(int i) {
			return getToken(TupleAttributeParser.NAME, i);
		}
		public EntryContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_entry; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterEntry(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitEntry(this);
		}
	}

	public final EntryContext entry() throws RecognitionException {
		EntryContext _localctx = new EntryContext(_ctx, getState());
		enterRule(_localctx, 40, RULE_entry);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(158);
			match(T__12);
			setState(162);
			_errHandler.sync(this);
			_la = _input.LA(1);
			while (_la==NAME) {
				{
				{
				setState(159);
				match(NAME);
				}
				}
				setState(164);
				_errHandler.sync(this);
				_la = _input.LA(1);
			}
			setState(165);
			match(T__12);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ListContext extends ParserRuleContext {
		public List<TerminalNode> NAME() { return getTokens(TupleAttributeParser.NAME); }
		public TerminalNode NAME(int i) {
			return getToken(TupleAttributeParser.NAME, i);
		}
		public ListContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_list; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).enterList(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof TupleAttributeListener ) ((TupleAttributeListener)listener).exitList(this);
		}
	}

	public final ListContext list() throws RecognitionException {
		ListContext _localctx = new ListContext(_ctx, getState());
		enterRule(_localctx, 42, RULE_list);
		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(167);
			match(T__4);
			setState(168);
			match(NAME);
			setState(170); 
			_errHandler.sync(this);
			_la = _input.LA(1);
			do {
				{
				{
				setState(169);
				match(NAME);
				}
				}
				setState(172); 
				_errHandler.sync(this);
				_la = _input.LA(1);
			} while ( _la==NAME );
			setState(174);
			match(T__5);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static final String _serializedATN =
		"\3\u608b\ua72a\u8133\ub9ed\u417c\u3be7\u7786\u5964\3\23\u00b3\4\2\t\2"+
		"\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13"+
		"\t\13\4\f\t\f\4\r\t\r\4\16\t\16\4\17\t\17\4\20\t\20\4\21\t\21\4\22\t\22"+
		"\4\23\t\23\4\24\t\24\4\25\t\25\4\26\t\26\4\27\t\27\3\2\3\2\3\2\7\2\62"+
		"\n\2\f\2\16\2\65\13\2\3\2\3\2\3\3\5\3:\n\3\3\3\5\3=\n\3\3\4\3\4\3\4\7"+
		"\4B\n\4\f\4\16\4E\13\4\3\5\6\5H\n\5\r\5\16\5I\3\6\6\6M\n\6\r\6\16\6N\3"+
		"\7\3\7\3\7\3\7\3\b\3\b\3\b\3\b\7\bY\n\b\f\b\16\b\\\13\b\3\t\3\t\3\t\3"+
		"\n\3\n\3\n\3\n\3\n\3\13\3\13\7\13h\n\13\f\13\16\13k\13\13\3\f\3\f\5\f"+
		"o\n\f\3\r\3\r\3\r\3\r\3\16\3\16\3\16\3\16\3\17\3\17\7\17{\n\17\f\17\16"+
		"\17~\13\17\3\20\3\20\5\20\u0082\n\20\3\21\3\21\3\21\3\22\3\22\3\22\3\22"+
		"\3\22\3\22\3\22\3\23\3\23\7\23\u0090\n\23\f\23\16\23\u0093\13\23\3\23"+
		"\3\23\3\24\3\24\3\24\5\24\u009a\n\24\3\25\3\25\3\25\5\25\u009f\n\25\3"+
		"\26\3\26\7\26\u00a3\n\26\f\26\16\26\u00a6\13\26\3\26\3\26\3\27\3\27\3"+
		"\27\6\27\u00ad\n\27\r\27\16\27\u00ae\3\27\3\27\3\27\2\2\30\2\4\6\b\n\f"+
		"\16\20\22\24\26\30\32\34\36 \"$&(*,\2\2\2\u00b1\2.\3\2\2\2\49\3\2\2\2"+
		"\6>\3\2\2\2\bG\3\2\2\2\nL\3\2\2\2\fP\3\2\2\2\16T\3\2\2\2\20]\3\2\2\2\22"+
		"`\3\2\2\2\24e\3\2\2\2\26n\3\2\2\2\30p\3\2\2\2\32t\3\2\2\2\34x\3\2\2\2"+
		"\36\u0081\3\2\2\2 \u0083\3\2\2\2\"\u0086\3\2\2\2$\u008d\3\2\2\2&\u0096"+
		"\3\2\2\2(\u009e\3\2\2\2*\u00a0\3\2\2\2,\u00a9\3\2\2\2.\63\5\4\3\2/\62"+
		"\5\24\13\2\60\62\5\34\17\2\61/\3\2\2\2\61\60\3\2\2\2\62\65\3\2\2\2\63"+
		"\61\3\2\2\2\63\64\3\2\2\2\64\66\3\2\2\2\65\63\3\2\2\2\66\67\7\2\2\3\67"+
		"\3\3\2\2\28:\5\6\4\298\3\2\2\29:\3\2\2\2:<\3\2\2\2;=\5\16\b\2<;\3\2\2"+
		"\2<=\3\2\2\2=\5\3\2\2\2>C\7\3\2\2?B\5\b\5\2@B\5\n\6\2A?\3\2\2\2A@\3\2"+
		"\2\2BE\3\2\2\2CA\3\2\2\2CD\3\2\2\2D\7\3\2\2\2EC\3\2\2\2FH\5\f\7\2GF\3"+
		"\2\2\2HI\3\2\2\2IG\3\2\2\2IJ\3\2\2\2J\t\3\2\2\2KM\5\32\16\2LK\3\2\2\2"+
		"MN\3\2\2\2NL\3\2\2\2NO\3\2\2\2O\13\3\2\2\2PQ\7\4\2\2QR\7\20\2\2RS\7\20"+
		"\2\2S\r\3\2\2\2TZ\7\5\2\2UY\5 \21\2VY\5\20\t\2WY\5\22\n\2XU\3\2\2\2XV"+
		"\3\2\2\2XW\3\2\2\2Y\\\3\2\2\2ZX\3\2\2\2Z[\3\2\2\2[\17\3\2\2\2\\Z\3\2\2"+
		"\2]^\7\6\2\2^_\5$\23\2_\21\3\2\2\2`a\7\7\2\2ab\7\20\2\2bc\7\b\2\2cd\5"+
		"$\23\2d\23\3\2\2\2ei\7\t\2\2fh\5\26\f\2gf\3\2\2\2hk\3\2\2\2ig\3\2\2\2"+
		"ij\3\2\2\2j\25\3\2\2\2ki\3\2\2\2lo\5\30\r\2mo\5\32\16\2nl\3\2\2\2nm\3"+
		"\2\2\2o\27\3\2\2\2pq\7\n\2\2qr\7\20\2\2rs\7\20\2\2s\31\3\2\2\2tu\7\20"+
		"\2\2uv\7\20\2\2vw\7\20\2\2w\33\3\2\2\2x|\7\13\2\2y{\5\36\20\2zy\3\2\2"+
		"\2{~\3\2\2\2|z\3\2\2\2|}\3\2\2\2}\35\3\2\2\2~|\3\2\2\2\177\u0082\5 \21"+
		"\2\u0080\u0082\5\"\22\2\u0081\177\3\2\2\2\u0081\u0080\3\2\2\2\u0082\37"+
		"\3\2\2\2\u0083\u0084\7\20\2\2\u0084\u0085\5$\23\2\u0085!\3\2\2\2\u0086"+
		"\u0087\7\7\2\2\u0087\u0088\7\20\2\2\u0088\u0089\7\20\2\2\u0089\u008a\7"+
		"\20\2\2\u008a\u008b\7\b\2\2\u008b\u008c\5$\23\2\u008c#\3\2\2\2\u008d\u0091"+
		"\7\f\2\2\u008e\u0090\5&\24\2\u008f\u008e\3\2\2\2\u0090\u0093\3\2\2\2\u0091"+
		"\u008f\3\2\2\2\u0091\u0092\3\2\2\2\u0092\u0094\3\2\2\2\u0093\u0091\3\2"+
		"\2\2\u0094\u0095\7\r\2\2\u0095%\3\2\2\2\u0096\u0099\7\20\2\2\u0097\u0098"+
		"\7\16\2\2\u0098\u009a\5(\25\2\u0099\u0097\3\2\2\2\u0099\u009a\3\2\2\2"+
		"\u009a\'\3\2\2\2\u009b\u009f\7\20\2\2\u009c\u009f\5*\26\2\u009d\u009f"+
		"\5,\27\2\u009e\u009b\3\2\2\2\u009e\u009c\3\2\2\2\u009e\u009d\3\2\2\2\u009f"+
		")\3\2\2\2\u00a0\u00a4\7\17\2\2\u00a1\u00a3\7\20\2\2\u00a2\u00a1\3\2\2"+
		"\2\u00a3\u00a6\3\2\2\2\u00a4\u00a2\3\2\2\2\u00a4\u00a5\3\2\2\2\u00a5\u00a7"+
		"\3\2\2\2\u00a6\u00a4\3\2\2\2\u00a7\u00a8\7\17\2\2\u00a8+\3\2\2\2\u00a9"+
		"\u00aa\7\7\2\2\u00aa\u00ac\7\20\2\2\u00ab\u00ad\7\20\2\2\u00ac\u00ab\3"+
		"\2\2\2\u00ad\u00ae\3\2\2\2\u00ae\u00ac\3\2\2\2\u00ae\u00af\3\2\2\2\u00af"+
		"\u00b0\3\2\2\2\u00b0\u00b1\7\b\2\2\u00b1-\3\2\2\2\25\61\639<ACINXZin|"+
		"\u0081\u0091\u0099\u009e\u00a4\u00ae";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}