$INSTANCE = eset;

//Sets the input file and loads.
inputFile = $1;
getta(inputFile);

//Gets the relations important for all phases.
direct = publish o subscribe o inv contain;
indirect = contain o publish o subscribe o inv contain;
indirect = indirect+;

//Gets all the timers and timer callbacks.
timers = $INSTANCE . {"rosTimer"};
tmrCallback = rng time;

//Generates a master relation.
masterRel = varWrite + varInfluence + varInfFunc + call + write;
masterRel = masterRel+;

//Gets all cases of timer alteration.
timerAlter = timers o time o tmrCallback o masterRel o publish;
timeCombo = time + masterRel + publish;

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
		showpath(item, var, timeCombo);
	}
}
