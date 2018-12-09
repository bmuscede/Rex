$INSTANCE = eset;

//Sets the input file and loads.
inputFile = $1;
getta(inputFile);

//Processes the regular expression system.
if $# >= 2 {
	getcsv($2);
}

//Gets the relations important for all phases.
direct = publish o subscribe o inv contain;
indirect = contain o publish o subscribe o inv contain;
indirect = indirect+;

//Gets all the timers and timer callbacks.
timers = $INSTANCE . {"rosTimer"};
if $# >= 2 {
	for str in dom CSVDATA {
		names = timers . @label;
		toRemove = grep(names, str);
		toRemove = toRemove . inv @label;
		timers = timers - toRemove;
	}
}
tmrCallback = rng time;
if $# >= 2 {
	for str in dom CSVDATA {
		names = tmrCallback . @label;
		toRemove = grep(names, str);
		toRemove = toRemove . inv @label;
		tmrCallback = tmrCallback - toRemove;
	}
}

//Generates a master relation.
masterRel = varWrite + varInfluence + varInfFunc + call + write;
masterRel = masterRel+;

//Gets all cases of timer alteration.
timerAlter = timers o time o tmrCallback o masterRel o publish;
timeCombo = time + masterRel + publish;
plainTimeCombo = inv @label o timeCombo o @label;

//Loops through the results.
for item in dom timerAlter {
	cbS = {item} . @label;

	//Get the items for this domain.
	vars = {item} . timerAlter;
	for var in vars {
		varS = {var} . @label;

		//Print the combination.
		print "####"
		print cbS;
		print varS;
		if $# == 3 {
			for cbSS in cbS { for varSS in varS { showpath(cbSS, varSS, plainTimeCombo); } }
		} else {
			showpath(item, var, timeCombo);
		}
	}
}
