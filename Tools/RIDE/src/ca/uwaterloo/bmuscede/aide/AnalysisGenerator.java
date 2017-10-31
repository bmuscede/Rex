package ca.uwaterloo.bmuscede.aide;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import java.nio.charset.StandardCharsets;
import java.util.List;

import ca.uwaterloo.bmuscede.process.ControlStructureProcess;
import ca.uwaterloo.bmuscede.process.DoubleWriteProcess;
import ca.uwaterloo.bmuscede.process.FeatureCommProcess;
import ca.uwaterloo.bmuscede.process.InteractionCreator;
import ca.uwaterloo.bmuscede.process.MasterProcess;
import ca.uwaterloo.cs.ql.LineInterp;
import ca.uwaterloo.cs.ql.interp.Env;
import ca.uwaterloo.cs.ql.interp.ScriptUnitNode;
import ca.uwaterloo.cs.ql.interp.Value;
import ca.uwaterloo.cs.ql.lib.InvocationException;
import ca.uwaterloo.cs.ql.lib.source;

public class AnalysisGenerator {
	public enum AnalysisTypes {
		FEAT_COMM("Feature Communication", "<html><center>Shows whether components interact with each other directly or indirectly"
				+ " through other components. Generates an interaction matrix.</center></html>", "featureComm.ql", 
				new FeatureCommProcess()),
		DOUB_WRITE("Multiple Communication", "<html><center>Finds components where multiple components message the same topic."
				+ " Shows all topics with two or more. </center></html>", "doubleMessage.ql", new DoubleWriteProcess()),
		CONT_STRUCT("Control Structures", "<html><center>Shows components that cause other components to change their behaviour."
				+ " Generates a matrix and list of variables.</center></html>", "controlFlow.ql", 
				new ControlStructureProcess());
		
		private String name;
		private String desc;
		private String fileName;
		private MasterProcess processor;
		
		private AnalysisTypes(String name, String desc, String fileName, MasterProcess p){
			this.name = name;
			this.desc = desc;
			this.fileName = fileName;
			this.processor = p;
		}
		
		@Override
		public String toString(){
			return name;
		}
		
		public String getDescription(){
			return desc;
		}
		
		public String getFileName(){
			return fileName;
		}
		
		public boolean performAnalysis(String content){
			return processor.performAnalysis(content);
		}
		
		public InteractionCreator outputResults(InteractionCreator creator){
			return processor.outputResults(creator);
		}
	}
	
	private boolean outputFlag;
	private List<AnalysisTypes> ops;
	private String input;
	private String output;
	private String errorMessage;
	
	private final String SCRIPT_DIR = System.getProperty("user.dir") + "/scripts/";
	
	public AnalysisGenerator(List<AnalysisTypes> operations){
		outputFlag = false;
		ops = operations;
		input = "";
		output = "";
		errorMessage = "";
	}
	
	public boolean runAnalysis(String inputTA, String outputDir){
		input = inputTA;
		output = outputDir;
		outputFlag = true;
		return runAnalysis();
	}
	
	public boolean runAnalysis(String inputTA){
		input = inputTA;
		outputFlag = false;
		return runAnalysis();
	}
	
	public String getErrorMessage() {
		return errorMessage;
	}
	
	@SuppressWarnings("unused")
	private boolean runScript(AnalysisTypes type){
		//Creates a new environment.
		Env env = new Env();
		LineInterp interp = new LineInterp();
		ScriptUnitNode unit = new ScriptUnitNode();
		env.setMainUnit(unit);
		env.pushScope(unit);
	    
		//Creates a value to hold the script args.
		Value[] vals = new Value[2];
		vals[0] = new Value(SCRIPT_DIR + type.getFileName()); ;
		vals[1] = new Value(input);
		
		//Creates a print stream to string.
		ByteArrayOutputStream printB = new ByteArrayOutputStream();
		ByteArrayOutputStream printE = new ByteArrayOutputStream();
		PrintStream ps;
		PrintStream es;
		try {
			ps = new PrintStream(printB, true, "utf-8");
			es = new PrintStream(printE, true, "utf-8");
		} catch (UnsupportedEncodingException e1) {
			errorMessage += "- Invalid encoding encountered for " + type.toString() + " operation!";
			return false;
		}
		env.out = ps;
		env.err = es;
		
		//Runs the script.
		source s = new source();
		try {
			//Run the script.
		    s.invoke(env, vals);
		    
		    //Get the results.
		    String error = new String(printE.toByteArray(), StandardCharsets.UTF_8);
			String content = new String(printB.toByteArray(), StandardCharsets.UTF_8);
			ps.close();
			es.close();
			
			//Check for error.
			if (!error.equals("")) {
				errorMessage += "- An error was returned while running the script for " + type.toString() + " operation!";
				return false;
			}
			
			//Run the analysis.
			return type.performAnalysis(content);
		} catch(InvocationException e) {
			//Gets messages.
		    ps.close();
		    es.close();
		    
		    //Sets the error message.
			errorMessage += "- Malformed TA file supplied for " + type.toString() + " operation!";
			return false;
		}
	}
	
	private boolean runAnalysis(){
		boolean finalResult = true;
		
		//Runs all the scripts.
		for (AnalysisTypes type : ops){
			boolean result = runScript(type);
			if (!result){
				errorMessage += "- There was an error while analyzing the results for " + type.toString() + " operation!";
				finalResult = false;
				continue;
			}
		}
		
		//Next, checks for the output.
		if (outputFlag){
			//Creates the interaction system.
			InteractionCreator creator = new InteractionCreator();
			for (AnalysisTypes type : ops) {
				creator = type.outputResults(creator);
				
				if (creator == null){
					errorMessage += " - Output failed for " + type.toString() + " operation!";
					return false;
				}
			}
			
			//Finally, output results.
			boolean success = creator.outputWorkbook(output);
			if (!success){
				errorMessage += " - Output failed while saving workbook!";
				return false;
			}
		}
		
		return finalResult;
	}

}
