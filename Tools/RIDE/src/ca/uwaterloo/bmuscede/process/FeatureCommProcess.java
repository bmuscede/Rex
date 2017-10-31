package ca.uwaterloo.bmuscede.process;

import java.util.SortedMap;
import java.util.TreeMap;

public class FeatureCommProcess extends MasterProcess{
	private String direct;
	private String indirect;
	private String allPairs;
	private SortedMap<String, SortedMap<String, String>> results;
	
	private final String SHEET_NAME = "Feature Communication Matrix";
	
	public FeatureCommProcess() {
		//Creates all pairs.
		results = new TreeMap<String, SortedMap<String, String>>();
		direct = "";
		indirect = "";
		allPairs = "";
	}
	
	@Override
	public boolean performAnalysis(String content) {
		//Splits by the content string.
		String[] cSplit = content.split(SEPARATOR);
		
		//Moves around the data.
		direct = cSplit[0];
		indirect = cSplit[1];
		allPairs = cSplit[2];
		
		//Now, we process the matrix.
		results = generateTableEntry(results, allPairs, NONE);
		results = generateTableEntry(results, direct, DIRECT);
		results = generateTableEntry(results, indirect, INDIRECT);
		
		return true;
	}

	@Override
	public InteractionCreator outputResults(InteractionCreator creator) {
		//Adds the pertinent fields to the creator.
		boolean res = creator.createFeatureMatrixSheet(SHEET_NAME, results);
		if (!res) return null;
		
		return creator;
	}

}
