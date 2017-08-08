package ca.uwaterloo.bmuscede.rex;

public class Analyzer {
	public static void main(String[] args){
		//Load in program options.
		if (args.length != 2){
			if (args.length == 1 && args[0].equals("--help")){
				
			} else {
				
			}
			
			return;
		}
		
		//Next, gets the input and output filenames.
		String taIn = args[0];
		String excelOut = args[1];
		
		//Loads in the TA file.
		
	}
}
