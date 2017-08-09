package ca.uwaterloo.bmuscede.ta;

import org.antlr.v4.runtime.tree.ParseTreeListener;

/**
 * This interface defines a complete listener for a parse tree produced by
 * {@link TupleAttributeParser}.
 */
public interface TupleAttributeListener extends ParseTreeListener {
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#root}.
	 * @param ctx the parse tree
	 */
	void enterRoot(TupleAttributeParser.RootContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#root}.
	 * @param ctx the parse tree
	 */
	void exitRoot(TupleAttributeParser.RootContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#schema}.
	 * @param ctx the parse tree
	 */
	void enterSchema(TupleAttributeParser.SchemaContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#schema}.
	 * @param ctx the parse tree
	 */
	void exitSchema(TupleAttributeParser.SchemaContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#tupleSchema}.
	 * @param ctx the parse tree
	 */
	void enterTupleSchema(TupleAttributeParser.TupleSchemaContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#tupleSchema}.
	 * @param ctx the parse tree
	 */
	void exitTupleSchema(TupleAttributeParser.TupleSchemaContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#inheritLine}.
	 * @param ctx the parse tree
	 */
	void enterInheritLine(TupleAttributeParser.InheritLineContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#inheritLine}.
	 * @param ctx the parse tree
	 */
	void exitInheritLine(TupleAttributeParser.InheritLineContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#relLine}.
	 * @param ctx the parse tree
	 */
	void enterRelLine(TupleAttributeParser.RelLineContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#relLine}.
	 * @param ctx the parse tree
	 */
	void exitRelLine(TupleAttributeParser.RelLineContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#inherit}.
	 * @param ctx the parse tree
	 */
	void enterInherit(TupleAttributeParser.InheritContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#inherit}.
	 * @param ctx the parse tree
	 */
	void exitInherit(TupleAttributeParser.InheritContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#attributeSchema}.
	 * @param ctx the parse tree
	 */
	void enterAttributeSchema(TupleAttributeParser.AttributeSchemaContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#attributeSchema}.
	 * @param ctx the parse tree
	 */
	void exitAttributeSchema(TupleAttributeParser.AttributeSchemaContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#specialAttr}.
	 * @param ctx the parse tree
	 */
	void enterSpecialAttr(TupleAttributeParser.SpecialAttrContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#specialAttr}.
	 * @param ctx the parse tree
	 */
	void exitSpecialAttr(TupleAttributeParser.SpecialAttrContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#schemeRelAttr}.
	 * @param ctx the parse tree
	 */
	void enterSchemeRelAttr(TupleAttributeParser.SchemeRelAttrContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#schemeRelAttr}.
	 * @param ctx the parse tree
	 */
	void exitSchemeRelAttr(TupleAttributeParser.SchemeRelAttrContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#factList}.
	 * @param ctx the parse tree
	 */
	void enterFactList(TupleAttributeParser.FactListContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#factList}.
	 * @param ctx the parse tree
	 */
	void exitFactList(TupleAttributeParser.FactListContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#factEntry}.
	 * @param ctx the parse tree
	 */
	void enterFactEntry(TupleAttributeParser.FactEntryContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#factEntry}.
	 * @param ctx the parse tree
	 */
	void exitFactEntry(TupleAttributeParser.FactEntryContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#nodeEntry}.
	 * @param ctx the parse tree
	 */
	void enterNodeEntry(TupleAttributeParser.NodeEntryContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#nodeEntry}.
	 * @param ctx the parse tree
	 */
	void exitNodeEntry(TupleAttributeParser.NodeEntryContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#relEntry}.
	 * @param ctx the parse tree
	 */
	void enterRelEntry(TupleAttributeParser.RelEntryContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#relEntry}.
	 * @param ctx the parse tree
	 */
	void exitRelEntry(TupleAttributeParser.RelEntryContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#attributeList}.
	 * @param ctx the parse tree
	 */
	void enterAttributeList(TupleAttributeParser.AttributeListContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#attributeList}.
	 * @param ctx the parse tree
	 */
	void exitAttributeList(TupleAttributeParser.AttributeListContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#attributeEntry}.
	 * @param ctx the parse tree
	 */
	void enterAttributeEntry(TupleAttributeParser.AttributeEntryContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#attributeEntry}.
	 * @param ctx the parse tree
	 */
	void exitAttributeEntry(TupleAttributeParser.AttributeEntryContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#nodeAttr}.
	 * @param ctx the parse tree
	 */
	void enterNodeAttr(TupleAttributeParser.NodeAttrContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#nodeAttr}.
	 * @param ctx the parse tree
	 */
	void exitNodeAttr(TupleAttributeParser.NodeAttrContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#relAttr}.
	 * @param ctx the parse tree
	 */
	void enterRelAttr(TupleAttributeParser.RelAttrContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#relAttr}.
	 * @param ctx the parse tree
	 */
	void exitRelAttr(TupleAttributeParser.RelAttrContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#attributes}.
	 * @param ctx the parse tree
	 */
	void enterAttributes(TupleAttributeParser.AttributesContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#attributes}.
	 * @param ctx the parse tree
	 */
	void exitAttributes(TupleAttributeParser.AttributesContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#attribute}.
	 * @param ctx the parse tree
	 */
	void enterAttribute(TupleAttributeParser.AttributeContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#attribute}.
	 * @param ctx the parse tree
	 */
	void exitAttribute(TupleAttributeParser.AttributeContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#val}.
	 * @param ctx the parse tree
	 */
	void enterVal(TupleAttributeParser.ValContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#val}.
	 * @param ctx the parse tree
	 */
	void exitVal(TupleAttributeParser.ValContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#entry}.
	 * @param ctx the parse tree
	 */
	void enterEntry(TupleAttributeParser.EntryContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#entry}.
	 * @param ctx the parse tree
	 */
	void exitEntry(TupleAttributeParser.EntryContext ctx);
	/**
	 * Enter a parse tree produced by {@link TupleAttributeParser#list}.
	 * @param ctx the parse tree
	 */
	void enterList(TupleAttributeParser.ListContext ctx);
	/**
	 * Exit a parse tree produced by {@link TupleAttributeParser#list}.
	 * @param ctx the parse tree
	 */
	void exitList(TupleAttributeParser.ListContext ctx);
}