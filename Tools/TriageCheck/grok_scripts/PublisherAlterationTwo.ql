$INSTANCE = eset;

//Sets the input file and loads.
inputFile = $1;
getta(inputFile);

//Processes the regular expression system.
if $# == 2 {
	getcsv($2);
}

//Gets the relations important for all phases.
direct = contain o publish o subscribe o call;
indirect = contain o publish o subscribe o inv contain;
indirect = indirect+;

callbackFuncs = rng(subscribe o call);
if $# == 2 {
	for str in dom CSVDATA {
		names = callbackFuncs . @label;
		toRemove = grep(names, str);
		toRemove = toRemove . inv @label;
		callbackFuncs = callbackFuncs - toRemove;
	}
}

masterRel = varWrite + varInfluence + varInfFunc + call + write;
masterRel = masterRel+;

//Gets all publisher alterations.
pubAlter = callbackFuncs o masterRel o publish;

//Print the results.
if #pubAlter > 0 {
	print "There are " + #pubAlter + " cases of publisher alteration.";
	print "";
} else {
	print "There are no cases of publisher alteration.";
	quit;
}

//Loops through and presents the results.
for item in dom pubAlter {
	print "---------------------------------------------------------";
	{item} . @label;
	print "";

	for topic in {item} . pubAlter {
		res = item + topic;
		sizes = {res} . pathSize;

		strTopic = {topic} . @label;
		for strTopicItem in strTopic {
			print strTopicItem + " - ";
			for size in sizes {
				print "\t" + size;
			}
		}
	}
	print "";

	dirInf = direct . {item};
	print "Influenced By - Direct:";
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
