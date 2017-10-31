package ca.uwaterloo.bmuscede.process;

import java.util.HashMap;
import java.util.Vector;

public class DoubleWriteProcess extends MasterProcess {
	private boolean emptyFlag = true;
	private HashMap<String, Integer> outdegreeVals;
	private HashMap<String, String> functionParent;
	private HashMap<String, Vector<String>> results;
	
	private final String SHEET_NAME = "Multiple Publisher Info";
	private final String EMPTY_MESSAGE = "No cases exist where multiple publishers write to a topic.";
	
	public DoubleWriteProcess(){
		outdegreeVals = new HashMap<String, Integer>();
		functionParent = new HashMap<String, String>();
		results = new HashMap<String, Vector<String>>();
	}
	
	@Override
	public boolean performAnalysis(String content) {
		//Checks if empty.
		if (content.equals("\n")){
			emptyFlag = true;
			return true;
		} else {
			emptyFlag = false;
		}
		
		//Splits the values.
		String[] splitEntries = content.split(SEPARATOR);
		
		//Now processes the outdegree.
		String[] outdegrees = splitEntries[0].split("\n");
		for (String curDegree : outdegrees){			
			//Adds the entry.
			String[] vals = curDegree.split(" ");
			String x = vals[0].replace("\"", "");
			outdegreeVals.put(x, Integer.parseInt(vals[1]));
		}
		
		//Next, we process the function parent lookup.
		String[] lookups = splitEntries[1].split("\n");
		for (String curEntry : lookups){
			if (curEntry.equals("")) continue;
			
			//Adds the entry.
			String[] vals = curEntry.split(" ");
			String y = vals[0].replace("\"", "");
			String x = vals[1].replace("\"", "");
			functionParent.put(x, y);
		}
		
		//Last, process the entries.
		for (int i = 2; i < splitEntries.length; i++){			
			String[] vals = splitEntries[i].split("\n");
			String currentItem = "";
			
			for (int j = 1; j < vals.length; j++){
				if (j == 1){
					currentItem = vals[j].replace("\"", "");
				} else {
					if (!results.containsKey(currentItem)){
						results.put(currentItem, new Vector<String>());
					}
					
					results.get(currentItem).add(vals[j].replace("\"", ""));
				}
			}
		}
		
		return true;
	}

	@Override
	public InteractionCreator outputResults(InteractionCreator creator) {
		if (emptyFlag){
			boolean succ = creator.createEmptySheet(SHEET_NAME, EMPTY_MESSAGE);
			if (!succ) return null;
			return creator;
		}
		
		//Runs the ouput system.
		boolean succ = creator.createMultiWriteSheet(SHEET_NAME, results, outdegreeVals, functionParent);
		if (!succ) return null;
		return creator;
	}
	
}
