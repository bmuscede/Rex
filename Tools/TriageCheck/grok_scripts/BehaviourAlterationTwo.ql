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
		toRemove = grep(callbackFuncs, str);
		callbackFuncs = callbackFuncs - toRemove;
	}
}

controlFlowVars = @isControlFlow . {"\"1\""};
if $# == 2 {
        for str in dom CSVDATA {
                toRemove = grep(controlFlowVars, str);
                controlFlowVars = controlFlowVars - toRemove;
        }
}

masterRel = varWrite + call + write + varInfFunc;
masterRel = masterRel+;

//Gets the behaviour alterations.
behAlter = callbackFuncs o masterRel o controlFlowVars;

//Print the results.
if #behAlter > 0 {
	print "There are " + #behAlter + " cases of behaviour alteration across " + #(dom behAlter) + " callback functions.";
	print "";
} else {
        print "There are no cases of behaviour alteration.";
        quit;
}

//Loops through and presents the results.
for item in dom behAlter {
	print "---------------------------------------------------------";
	{item} . @label;
	print "";

	print "Affects Variables:"
	for var in {item} . behAlter {
		res = item + var;
		sizes = {res} . pathSize;
		
		strVar = {var} . @label;
		for strVarItem in strVar {
			print strVarItem + " - ";
			for size in sizes {
				print "\t" + size;
			}
		} 	
	}
	print "";

	print "Influenced By - Direct:"
	dirInf = direct . {item};
	if (#dirInf > 0) {
		dirInf . @label;
	} else {
		print "<NONE>";
	}
	print "";

	inInf = indirect . (direct . {item});
	print "Influenced By - Indirect:";
	inInf = inInf - dirInf;
	if (#inInf > 0) {
		inInf . @label;
	} else {
		print "<NONE>";
	}

        print "---------------------------------------------------------";
}
