$INSTANCE = eset;

//Sets the input file and loads.
inputFile = $1;
getta(inputFile);

//Processes the regular expression system.
if $# >= 2 {
	getcsv($2);
}

//Gets the relations important for all phases.
direct = contain o publish o subscribe o call;
indirect = contain o publish o subscribe o inv contain;
indirect = indirect+;

callbackFuncs = rng(subscribe o call);
if $# >= 2 {
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
comboM = masterRel + publish;
plainComboM = inv @label o comboM o @label;

//Loops through the results.
for item in dom pubAlter {
	cbS = {item} . @label;
	
	//Get the items for this domain.
	vars = {item} . pubAlter;
	for var in vars {
		varS = {var} . @label;

		//Print the combination.
		print "####";
		print cbS;
		print varS;
		if $# == 3 {
 			for cbSS in cbS { for varSS in varS { showpath(cbSS, varSS, plainComboM); } }
		} else {
			showpath(item, var, comboM);
		}
	}
}
