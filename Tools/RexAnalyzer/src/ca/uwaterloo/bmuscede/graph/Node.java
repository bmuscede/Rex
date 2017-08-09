package ca.uwaterloo.bmuscede.graph;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

public class Node {
	public enum Type {
		CCLASS("cClass"), ROSTOPIC("rosTopic"), ROSPUBLISHER("rosPublisher"), 
		ROSSUBSCRIBER("rosSubscriber"), ROSNODEHANDLE("rosNodeHandle"), CFILE("cFile"),
		CFUNCTION("cFunction"), CVARIABLE("cVariable");
		
		private String name;
		
		private Type(String name){
			this.name = name;
		}
		
		@Override
		public String toString(){
			return name;
		}
	};
	
	private String nodeID;
	private Type nodeType;
	protected Map<String, Vector<String>> attributes;
	
	private final String INSTANCE_FLAG = "$INSTANCE";
	
	public Node(String nodeID, Type nodeType){
		this.setNodeID(nodeID);
		this.setNodeType(nodeType);
		
		attributes = new HashMap<String, Vector<String>>();
	}

	public Node(String name, String nodeVal) {
		Type nodeType = Type.valueOf(nodeVal.toUpperCase());
		this.setNodeID(name);
		this.setNodeType(nodeType);
		
		attributes = new HashMap<String, Vector<String>>();
	}

	public String getNodeID() {
		return nodeID;
	}

	public void setNodeID(String nodeID) {
		this.nodeID = nodeID;
	}

	public Type getNodeType() {
		return nodeType;
	}

	public void setNodeType(Type nodeType) {
		this.nodeType = nodeType;
	}
	
	public int getNumAttributes(){
		return attributes.size();
	}
	
	public Vector<String> getAttribute(String key){
		return attributes.get(key);
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
	
	public Map<String, Vector<String>> getAttributes() {
		return attributes;
	}
	
	@Override
	public boolean equals(Object obj) {
		if (obj == null) return false;
		if (!Node.class.isAssignableFrom(obj.getClass())) return false;
		
		final Node otherNode = (Node) obj;
		if (otherNode.getNodeID().equals(nodeID) && otherNode.getNodeType().equals(nodeType))
			return true;
		
		return false;
	}
	
	@Override
	public int hashCode() {
	    int hash = 3;
	    hash = 53 * hash + (this.nodeID != null ? this.nodeID.hashCode() : 0);
	    hash += 53 * hash + this.nodeType.hashCode();
	    return hash;
	}
	
	@Override
	public String toString() {
		return nodeID;
	}
	
	public String toFactLine() {
		return INSTANCE_FLAG + " " + nodeID + " " + nodeType.toString();
	}
	
	public String toAttributeLine() {
		//Checks if there are no attributes.
		if (getNumAttributes() == 0) return "";
		
		String attr = nodeID + " { ";
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
