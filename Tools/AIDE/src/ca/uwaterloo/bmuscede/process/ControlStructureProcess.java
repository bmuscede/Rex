package ca.uwaterloo.bmuscede.process;

import java.util.SortedMap;
import java.util.TreeMap;
import java.util.Vector;

public class ControlStructureProcess extends MasterProcess {
	private String vars;
	private String allPairs;
	private String direct;
	private String indirect;
	private SortedMap<String, SortedMap<String, String>> results;
	private SortedMap<String, Vector<String>> varWrites;
	
	private final String SHEET_NAME = "Control Structure Matrix";
	
	public ControlStructureProcess(){
		vars = "";
		direct = "";
		indirect = "";
		results = new TreeMap<String, SortedMap<String, String>>();
		varWrites = new TreeMap<String, Vector<String>>();
	}
	
	@Override
	public boolean performAnalysis(String content) {
		//Takes the content and splits by equals.
		String[] components = content.split(SEPARATOR);
		
		//Divides into sections.
		vars = components[0];
		direct = components[1];
		indirect = components[2];
		allPairs = components[3];
		
		//Next, generates a result matrix.
		results = generateTableEntry(results, allPairs, NONE);
		results = generateTableEntry(results, direct, DIRECT);
		results = generateTableEntry(results, indirect, INDIRECT);
		
		//Now, generates the variable writes SortedMap.
		String[] varSplit = vars.split("\n");
		for (String entry : varSplit){
			if (entry.equals("")) continue;
			
			String[] vals = entry.split(" ");
			String x = vals[0].replace("\"", "");
			String y = vals[1].replace("\"", "");
			
			//Add the entry.
			if (!varWrites.containsKey(x)){
				varWrites.put(x, new Vector<String>());
			}
			varWrites.get(x).add(y);	
		}
		
		return true;
	}

	@Override
	public InteractionCreator outputResults(InteractionCreator creator) {
		//Adds the pertinent fields to the creator.
		boolean res = creator.createFeatureMatrixSheet(SHEET_NAME, results);
		if (!res) return null;
		res = creator.addVarColumnBelow(SHEET_NAME, varWrites, results.keySet().size());
		if (!res) return null;
		
		return creator;
	}

}
