package ca.uwaterloo.bmuscede.process;

import java.util.SortedMap;
import java.util.TreeMap;

public abstract class MasterProcess {
	protected final String SEPARATOR = "==========================";
	
	protected final String NONE = InteractionCreator.NO_COMM_TEXT;
	protected final String DIRECT = InteractionCreator.DIRECT_TEXT;
	protected final String INDIRECT = InteractionCreator.INDIRECT_TEXT;
	
	public abstract boolean performAnalysis(String content);
	public abstract InteractionCreator outputResults(InteractionCreator creator);
	
	protected SortedMap<String, SortedMap<String, String>> generateTableEntry(SortedMap<String, SortedMap<String, String>> results, 
			String data, String tableLabel){
		String[] lines = data.split("\n");
		for (String curPair : lines){
			if (curPair.equals("")) continue;
			
			String[] vals = curPair.split(" ");
			String x = vals[0].replace("\"", "");
			String y = vals[1].replace("\"", "");
			
			//Add the entry.
			if (!results.containsKey(x)){
				results.put(x, new TreeMap<String,String>());
			}
			results.get(x).put(y, tableLabel);
		}
		
		return results;
	}
}
