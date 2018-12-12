$INSTANCE = eset;

//Sets the input file and loads.
inputFile = $1;
getta(inputFile);

//Gets the relations important for all phases.
direct = contain o publish o subscribe o call;
indirect = contain o publish o subscribe o inv contain;
indirect = indirect+;

callbackFuncs = rng(subscribe o call);
masterRel = varWrite + varInfluence + varInfFunc + call + write;
masterRel = masterRel+;

//Gets all publisher alterations.
pubAlter = callbackFuncs o masterRel o publish;
if $2 == "true" {
	pubAlter = inv @label o pubAlter o @label;
}
pubAlter;
