package ca.uwaterloo.bmuscede.rex;

import java.io.IOException;
import java.util.Collection;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.Vector;

import org.antlr.v4.runtime.CharStream;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.TokenStream;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.ParseTreeWalker;

import ca.uwaterloo.bmuscede.graph.Edge;
import ca.uwaterloo.bmuscede.graph.Node;
import ca.uwaterloo.bmuscede.rex.PathData.PathType;
import ca.uwaterloo.bmuscede.ta.StringErrorListener;
import ca.uwaterloo.bmuscede.ta.TupleAttributeBaseListener;
import ca.uwaterloo.bmuscede.ta.TupleAttributeLexer;
import ca.uwaterloo.bmuscede.ta.TupleAttributeParser;
import edu.uci.ics.jung.graph.Graph;

public class Analyzer {
	public final static String HELP_MESSAGE = "";
	
	private static Vector<PathData> masterPaths = null;
	
	public static void main(String[] args){
		//Load in program options.
		if (args.length != 2){
			if (args.length == 1 && args[0].equals("--help")){
				System.out.println(HELP_MESSAGE);
			} else {
				System.err.println("Usage: RexAnalyzer <TAIn> <ExcelOut>");
				System.out.println(HELP_MESSAGE);
			}
			
			return;
		}
		
		//Next, gets the input and output filenames.
		String taIn = args[0];
		String excelOut = args[1];
		
		//Loads in the TA file.
		Graph<Node, Edge> graph = loadTAFile(taIn);
		
		//Generates matrix data.
		SortedMap<String, SortedMap<String, String>> data = generateData(graph);
		Vector<PathData> pathData = masterPaths;
		
		//Outputs the excel sheet.
		InteractionCreator creator = new InteractionCreator();
		creator.createFeatureMatrixSheet(data, pathData);
		boolean result = creator.outputWorkbook(excelOut);
		
		//Check the result.
		if (result == true){
			System.out.println("Excel workbook successfully outputted to " + excelOut + "!");
		} else {
			System.err.println("There was a problem outputting the excel workbook!");
		}
	}

	private static Graph<Node, Edge> loadTAFile(String taIn){
		//Generates a character stream.
		CharStream stream = null;
		try {
			stream = CharStreams.fromFileName(taIn);
		} catch (IOException e) {
			System.err.println("Error loading: " + e.getMessage() + ". Please try a different file.");
			return null;
		}
		
		//Error listener.
		StringErrorListener elistener = new StringErrorListener();
		
		//Adds lexers and parsers for use.
		TupleAttributeLexer lexer = new TupleAttributeLexer(stream);
		lexer.removeErrorListeners();
		lexer.addErrorListener(elistener);
		TokenStream tokens = new CommonTokenStream(lexer);
		TupleAttributeParser parser = new TupleAttributeParser(tokens);
		parser.removeErrorListeners();
		parser.addErrorListener(elistener);
		ParseTree tree = parser.root();
		
		//Checks the errors generated.
		if (parser.getNumberOfSyntaxErrors() > 0){
			System.err.println("Parse error occurred!\n" + elistener.getMessages());
			return null;
		}

		//Generates a walker to walk through the tree.
		TupleAttributeBaseListener listener = new TupleAttributeBaseListener();
		ParseTreeWalker.DEFAULT.walk(listener, tree);
				
		//Sets the resultant graph to be the current one.
		return listener.getGraph();
	}
	
	private static SortedMap<String, SortedMap<String, String>> generateData(Graph<Node, Edge> graph) {
		masterPaths = new Vector<PathData>();
		
		SortedMap<String, SortedMap<String, String>> map = new TreeMap<String, SortedMap<String, String>>();
		Vector<Node> classes = getClasses(graph);
		
		//Now, we loop through and generate a path to each of our nodes.
		for (Node fClass : classes){
			Vector<PathData> paths = new Vector<PathData>();
			
			//Adds the entry.
			map.put(fClass.getAttribute("label").elementAt(0), new TreeMap<String, String>());
			
			//Next, gets the child publishers.
			Vector<Node> pubs = new Vector<Node>();
			for (Edge contEdges : graph.getOutEdges(fClass)){
				if (contEdges.getEdgeType() == Edge.Type.CONTAIN && contEdges.getDst().getNodeType() == Node.Type.ROSPUBLISHER){
					pubs.add(contEdges.getDst());
				}
			}
			
			//For each publisher, follows the path.
			for (Node pub : pubs){
				paths.addAll(getReachability(graph, fClass, pub, 0, new Vector<Node>()));
			}
			
			//Now, we traverse through the classes.
			for (Node sClass : classes){
				if (fClass.equals(sClass)) continue;
				
				//Check if it is in the path list.
				boolean found = false;
				for (PathData curPath : paths){
					if (curPath.getSrc().equals(fClass) && curPath.getDst().equals(sClass)){
						found = true;
						map.get(fClass.getAttribute("label").elementAt(0)).put(
								sClass.getAttribute("label").elementAt(0), curPath.getType().toString());
						masterPaths.add(curPath);
						break;
					}
				}
				
				if (!found){
					map.get(fClass.getAttribute("label").elementAt(0)).put(
							sClass.getAttribute("label").elementAt(0), PathType.NO_PATH.toString());
					masterPaths.add(new PathData(fClass, sClass, PathType.NO_PATH));
				}
			}
		}
		
		return map;
	}

	//TODO: This doesn't deal with loops.
	private static Vector<PathData> getReachability(Graph<Node, Edge> graph, Node original, Node cur, int hops, 
			Vector<Node> path) {
		Vector<PathData> data = new  Vector<PathData>();
		path.add(cur);
		
		//Checks the node type.
		if (cur.getNodeType() == Node.Type.ROSPUBLISHER) {
			//Get the topic it points to.
			Collection<Edge> edges = graph.getOutEdges(cur);
			if (edges.size() > 0) data.addAll(getReachability(graph, original, edges.iterator().next().getDst(), hops, path));
		} else if (cur.getNodeType() == Node.Type.ROSTOPIC) {
			//Get the topic it points to.
			for (Edge topEdge : graph.getOutEdges(cur)){
				if (topEdge.getEdgeType() == Edge.Type.SUBSCRIBE){
					data.addAll(getReachability(graph, original, topEdge.getDst(), hops, path));
				}
			}
		} else if (cur.getNodeType() == Node.Type.ROSSUBSCRIBER){
			Node parent = null;

			//Get the parent class.
			for (Edge curEdge : graph.getInEdges(cur)){
				if (curEdge.getEdgeType() == Edge.Type.CONTAIN){
					parent = curEdge.getSrc();
					break;
				}
			}
			path.add(parent);
			
			//Record the data.
			PathData pathItem = new PathData(original, parent, (hops == 0) ? PathType.DIRECT : PathType.INDIRECT);
			pathItem.addPath(path);
			data.add(pathItem);
			hops++;
			
			//Next, continue.
			for (Edge contEdges : graph.getOutEdges(parent)){
				if (contEdges.getEdgeType() == Edge.Type.CONTAIN && contEdges.getDst().getNodeType() == Node.Type.ROSPUBLISHER){
					if (path.contains(contEdges.getDst())) continue;
					
					data.addAll(getReachability(graph, original, contEdges.getDst(), hops, path));
				}
			}
		}
		
		return data;
	}	
	
	private static Vector<Node> getClasses(Graph<Node, Edge> graph) {
		Vector<Node> classes = new Vector<Node>();
		
		//Go through all the nodes.
		for (Node curNode : graph.getVertices()){
			if (curNode.getNodeType() == Node.Type.CCLASS){
				classes.add(curNode);
			}
		}
		
		return classes;
	}
}
