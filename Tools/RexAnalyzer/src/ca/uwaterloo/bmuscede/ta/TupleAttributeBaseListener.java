package ca.uwaterloo.bmuscede.ta;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;

import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.tree.ErrorNode;
import org.antlr.v4.runtime.tree.TerminalNode;

import ca.uwaterloo.bmuscede.graph.Edge;
import ca.uwaterloo.bmuscede.graph.Node;
import edu.uci.ics.jung.graph.Graph;
import edu.uci.ics.jung.graph.SparseMultigraph;
import edu.uci.ics.jung.graph.util.EdgeType;

/**
 * This class provides an empty implementation of {@link TupleAttributeListener},
 * which can be extended to create a listener which only needs to handle a subset
 * of the available methods.
 */
public class TupleAttributeBaseListener implements TupleAttributeListener {
	private Graph<Node, Edge> graph;
	private Queue<String> nodeQueue;
	
	private Map<String, String> attrMap;
	private boolean schema;
	private boolean relAttr;
	
	public TupleAttributeBaseListener(){
		schema = false;
		relAttr = false;
		
		graph = new SparseMultigraph<Node, Edge>();
		nodeQueue = new LinkedList<String>();
		
		attrMap = new HashMap<String, String>();
	}
	
	public Graph<Node, Edge> getGraph(){
		return graph;
	}
	
	private Node findNode(String nodeID){
		//Selects the node based on the name.
		for (Node curNode : graph.getVertices()){
			if (curNode.getNodeID().equals(nodeID)) return curNode;
		}
		
		return null;
	}
	
	private Edge findEdge(String src, String dst, String edgeType) {
		//Finds the edge based on several names.
		for (Edge current : graph.getEdges()){
			if (current.getEdgeType().toString().equals(edgeType) &&
					current.getSrc().getNodeID().equals(src) &&
					current.getDst().getNodeID().equals(dst)) return current;
		}
		return null;
	}

	
	@Override 
	public void enterSchema(TupleAttributeParser.SchemaContext ctx) {
		schema = true;
	}
	
	@Override 
	public void exitSchema(TupleAttributeParser.SchemaContext ctx) {
		schema = false;
	}

	@Override 
	public void enterNodeEntry(TupleAttributeParser.NodeEntryContext ctx) {
		nodeQueue = new LinkedList<String>();
	}
	
	@Override 
	public void exitNodeEntry(TupleAttributeParser.NodeEntryContext ctx) {
		//Adds the features or parameters to the list.
		String name = nodeQueue.poll();
		String nodeType = nodeQueue.poll();
		
		//Adds the node to the graph.
		Node current = new Node(name, nodeType);
		graph.addVertex(current);
	}

	@Override 
	public void enterRelEntry(TupleAttributeParser.RelEntryContext ctx) { 
		nodeQueue = new LinkedList<String>();
	}
	
	@Override 
	public void exitRelEntry(TupleAttributeParser.RelEntryContext ctx) {
		if (schema) return;
		
		//Polls off the queue.
		String edgeType = nodeQueue.poll();
		String srcName = nodeQueue.poll();
		String dstName = nodeQueue.poll();
		
		//Gets the source and destination.
		Node src = findNode(srcName);
		Node dst = findNode(dstName);
		
		//Creates the edge.
		Edge current = new Edge(src, dst, edgeType);
		graph.addEdge(current, src, dst, EdgeType.DIRECTED);
	}

	@Override 
	public void exitNodeAttr(TupleAttributeParser.NodeAttrContext ctx) {
		if (schema) return;
		
		//Get the node and add the attributes.
		String name = nodeQueue.poll();
		Node current = findNode(name);

		//Next, adds all the attributes.
		for (Map.Entry<String, String> cur : attrMap.entrySet()){
			current.addAttribute(cur.getKey(), cur.getValue());
		}
	}
	
	@Override 
	public void enterRelAttr(TupleAttributeParser.RelAttrContext ctx) {
		relAttr = true;
	}
	
	@Override 
	public void exitRelAttr(TupleAttributeParser.RelAttrContext ctx) {
		//Get the node and add the attributes.
		String edgeType = nodeQueue.poll();
		String src = nodeQueue.poll();
		String dst = nodeQueue.poll();
		
		//Gets the edge.
		Edge current = findEdge(src, dst, edgeType);

		//Next, adds all the attributes.
		for (Map.Entry<String, String> cur : attrMap.entrySet()){
			current.addAttribute(cur.getKey(), cur.getValue());
		}

		relAttr = false;
	}
	
	@Override 
	public void enterAttributes(TupleAttributeParser.AttributesContext ctx) {
		attrMap = new HashMap<String, String>();
	}
	
	@Override 
	public void exitAttribute(TupleAttributeParser.AttributeContext ctx) {
		if (schema) return;
		
		//Gets the name.
		String name = nodeQueue.poll();
		String nameOne = "";
		String nameTwo = "";
		if (relAttr) nameOne = nodeQueue.poll();
		if (relAttr) nameTwo = nodeQueue.poll();
		
		//Iterates through the nodes.
		String key = nodeQueue.poll();
		String value = "";
		while (nodeQueue.size() > 0){
			value += nodeQueue.poll();
			if (nodeQueue.size() != 0) value += " ";
		}
		attrMap.put(key, value);
		
		//Adds the name back to the queue.
		nodeQueue.add(name);
		if (relAttr) nodeQueue.add(nameOne);
		if (relAttr) nodeQueue.add(nameTwo);
	}
	
	@Override 
	public void visitTerminal(TerminalNode node) {
		if (node.getSymbol().getType() == 14) nodeQueue.add(node.getText());
	}

	@Override public void enterVal(TupleAttributeParser.ValContext ctx) { }
	@Override public void exitVal(TupleAttributeParser.ValContext ctx) { }
	@Override public void enterEntry(TupleAttributeParser.EntryContext ctx) { }
	@Override public void exitEntry(TupleAttributeParser.EntryContext ctx) { }
	@Override public void enterList(TupleAttributeParser.ListContext ctx) { }
	@Override public void exitList(TupleAttributeParser.ListContext ctx) { }

	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterEveryRule(ParserRuleContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitEveryRule(ParserRuleContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitAttributes(TupleAttributeParser.AttributesContext ctx) { }
	
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterAttribute(TupleAttributeParser.AttributeContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterAttributeList(TupleAttributeParser.AttributeListContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitAttributeList(TupleAttributeParser.AttributeListContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterAttributeEntry(TupleAttributeParser.AttributeEntryContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitAttributeEntry(TupleAttributeParser.AttributeEntryContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterNodeAttr(TupleAttributeParser.NodeAttrContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterRoot(TupleAttributeParser.RootContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitRoot(TupleAttributeParser.RootContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void visitErrorNode(ErrorNode node) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterTupleSchema(TupleAttributeParser.TupleSchemaContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitTupleSchema(TupleAttributeParser.TupleSchemaContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterInheritLine(TupleAttributeParser.InheritLineContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitInheritLine(TupleAttributeParser.InheritLineContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterRelLine(TupleAttributeParser.RelLineContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitRelLine(TupleAttributeParser.RelLineContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterInherit(TupleAttributeParser.InheritContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitInherit(TupleAttributeParser.InheritContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterAttributeSchema(TupleAttributeParser.AttributeSchemaContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitAttributeSchema(TupleAttributeParser.AttributeSchemaContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterSpecialAttr(TupleAttributeParser.SpecialAttrContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitSpecialAttr(TupleAttributeParser.SpecialAttrContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterSchemeRelAttr(TupleAttributeParser.SchemeRelAttrContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitSchemeRelAttr(TupleAttributeParser.SchemeRelAttrContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterFactList(TupleAttributeParser.FactListContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitFactList(TupleAttributeParser.FactListContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void enterFactEntry(TupleAttributeParser.FactEntryContext ctx) { }
	/**
	 * {@inheritDoc}
	 *
	 * <p>The default implementation does nothing.</p>
	 */
	@Override public void exitFactEntry(TupleAttributeParser.FactEntryContext ctx) { }
}