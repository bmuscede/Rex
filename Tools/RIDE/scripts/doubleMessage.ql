////////////////////////////////////////////////////////////////////////
// doubleMessage.ql
// By: Bryan J Muscedere
//
// Detects components that are messaged by multiple components
// and where those messages feed to the same callback function.
// This only deals with direct communications since indirect are
// irrelevant for this script.
// This script doesn't work if a callback function is used by multiple
// topics.
////////////////////////////////////////////////////////////////////////

$INSTANCE = eset;

//Sets the input file and loads.
inputFile = $1;
getta(inputFile);

//Gets topics that are written to.
direct = publish o subscribe;
direct = inv @label o (contain o direct);

//Gets the indegree
inset = indegree(direct);
inset = inset - (inset o {2});
inset = inv(call) o inset;

//Prints the results.
if (#inset == 0){
	print "";
} else {
	inv(@label) o inset;
	print "==========================";
	
	//Print the classes that each of these callback functions belong to.
	classes = contain o dom(inset);
	inv(@label) o classes o @label;

	//Now, dives deeper for each function.
	for item in dom(inset) {
		print "==========================";
		cFunction = (rng({item} o @label));
		cFunction;

		//Get the publishers.
		direct . (call . {item});
	}
}