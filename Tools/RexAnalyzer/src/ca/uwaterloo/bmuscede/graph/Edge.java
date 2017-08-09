package ca.uwaterloo.bmuscede.graph;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

public class Edge {
	public enum Type {
		CONTAIN("contain"), REFERENCE("references"), CALL("call"), ADVERTISE("advertise"), SUBSCRIBE("subscribe"), 
		PUBLISH("publish");
		
		private String name;
		
		private Type(String name){
			this.name = name;
		}
		
		@Override
		public String toString(){
			return name;
		}
	};
	
	private Node src;
	private Node dst;
	private Type edgeType;
	
	private Map<String, Vector<String>> attributes;
	
	public Edge(Node src, Node dst, Type edgeType){
		this.setSrc(src);
		this.setDst(dst);
		this.setEdgeType(edgeType);
		
		attributes = new HashMap<String, Vector<String>>();
	}

	public Edge(Node src, Node dst, String edgeType){
		this.setSrc(src);
		this.setDst(dst);
		this.setEdgeType(Type.valueOf(edgeType.toUpperCase()));
		
		attributes = new HashMap<String, Vector<String>>();
	}
	
	public Type getEdgeType() {
		return edgeType;
	}

	public void setEdgeType(Type edgeType) {
		this.edgeType = edgeType;
	}

	public Node getSrc() {
		return src;
	}

	public void setSrc(Node src) {
		this.src = src;
	}

	public Node getDst() {
		return dst;
	}

	public void setDst(Node dst) {
		this.dst = dst;
	}
	
	public int getNumAttributes() {
		return attributes.size();
	}
	
	public void addAttribute(String key, String value){
		//Checks if the entry exists.
		if (!attributes.containsKey(key)) attributes.put(key, new Vector<String>());
		
		//Inserts the key.
		attributes.get(key).add(value);
	}
	
	public void addAttribute(String key, Vector<String> values){
		//Checks if the entry exists.
		if (!attributes.containsKey(key)) {
			attributes.put(key, values);
			return;
		}
		
		for (String value : values){
			attributes.get(key).add(value);
		}
	}
	
	public Vector<String> getAttributes(String key){
		return attributes.get(key); 
	}
	
	@Override
	public boolean equals(Object obj) {
		if (obj == null) return false;
		if (!Edge.class.isAssignableFrom(obj.getClass())) return false;
		
		final Edge otherEdge = (Edge) obj;
		if (otherEdge.getSrc().equals(src) && otherEdge.getDst().equals(dst) &&
				otherEdge.getEdgeType().equals(edgeType))
			return true;
		
		return false;
	}
	
	@Override
	public int hashCode() {
	    int hash = 3;
	    hash = 53 * hash + (this.src != null ? this.src.hashCode() : 0);
	    hash += 53 * hash + (this.dst != null ? this.dst.hashCode() : 0);
	    hash += 53 * hash + this.edgeType.hashCode();
	    return hash;
	}
	
	@Override
	public String toString() {
		return edgeType.toString();
	}
	
	public String toFactLine() {
		return edgeType.toString() + " " + src.getNodeID() + " " + dst.getNodeID();
	}
	
	public String toAttributeLine() {
		String attr = "(" + toFactLine() + ")" + " { ";
		for (Map.Entry<String, Vector<String>> entry : attributes.entrySet()){
			//Prints the key.
			attr += entry.getKey() + " = ";
			
			//Needs to print the values.
			Vector<String> values = entry.getValue();
			if (values.size() == 1){
				attr += "\"" + values.get(0) + "\" ";
			} else {
				//Iterates through the value list.
				attr += " ( ";
				for (String value : values){
					attr += "\"" + value + "\" ";
				}
				attr += ") ";
			}
		}
		
		//Finishes off the attribute.
		attr += "}";
		return attr;
	}
}