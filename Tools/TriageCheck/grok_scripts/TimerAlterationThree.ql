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
if $2 == "true" {
	timerAlter = inv @label o timeAlter o @label;
}
timerAlter;
