$INSTANCE = eset;

//Sets the input file and loads.
inputFile = $1;
getta(inputFile);

//Processes the regular expression system.
if $# == 2 {
	getcsv($2);
}

//Gets the direct and indirect relations.
direct = contain o publish o subscribe o call;
indirect = contain o publish o subscribe o inv contain;
indirect = indirect+;

//Generates relations to track the flow of data. 
callbackFuncs = rng(subscribe o call);
if $# == 2 {
	for str in dom CSVDATA {
		names = callbackFuncs . @label;
		toRemove = grep(names, str);
		toRemove = toRemove . inv @label;
		callbackFuncs = callbackFuncs - toRemove;
	}
}

controlFlowVars = @isControlFlow . {"\"1\""};
if $# == 2 {
        for str in dom CSVDATA {
                names = controlFlowVars . @label;
		toRemove = grep(names, str);
		toRemove = toRemove . inv @label;
                controlFlowVars = controlFlowVars - toRemove;
	}
}

masterRel = varWrite + call + write + varInfFunc;
masterRel = masterRel+;

//Gets the behaviour alterations.
behAlter = callbackFuncs o masterRel o controlFlowVars;

//Loops through the results.
for item in dom behAlter {
	cbS = {item} . @label;

	//Get the items for this domain.
	vars = {item} . behAlter;
	for var in vars {
		varS = {var} . @label;

		// Print the combination.
		print "####";
		print cbS;
		print varS;
		showpath(item, var, masterRel);
	}
}
