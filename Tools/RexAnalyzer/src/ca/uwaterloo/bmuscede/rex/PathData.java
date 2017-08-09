package ca.uwaterloo.bmuscede.rex;

import java.util.Vector;

import ca.uwaterloo.bmuscede.graph.Node;

public class PathData {
	private Node srcPath;
	private Node dstPath;
	
	private Vector<Node> path = null;
	private PathType type;
	
	enum PathType {
		DIRECT("Direct"), INDIRECT("Indirect"), NO_PATH("No Communication");
		
		private String name;
		
		private PathType(String name){
			this.name = name;
		}
	
		@Override
		public String toString(){
			return name;
		}
	};
	
	public PathData(Node src, Node dst, PathType type){
		setSrc(src);
		setDst(dst);
		this.type = type;
	}
	
	public void addPath(Vector<Node> path){
		if (type != PathType.INDIRECT) return;
		if (path.contains(srcPath)) path.remove(srcPath);
		if (path.contains(dstPath)) path.remove(dstPath);
		
		Vector<Node> finalPath = new Vector<Node>();
		for (Node curNode : path) {
			if (curNode.getNodeType() == Node.Type.CCLASS) finalPath.add(curNode);
		}
		
		this.path = finalPath;
	}

	public Vector<Node> getPath(){
		return path;
	}
	
	public PathType getType(){
		return type;
	}
	
	public Node getSrc() {
		return srcPath;
	}

	public void setSrc(Node srcPath) {
		this.srcPath = srcPath;
	}

	public Node getDst() {
		return dstPath;
	}

	public void setDst(Node dstPath) {
		this.dstPath = dstPath;
	}
}
