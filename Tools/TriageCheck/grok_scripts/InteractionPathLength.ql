$INSTANCE = eset;

//Sets the input file and loads.
inputFile = $1;
getta(inputFile);

containTC = contain+;

//Prints the path length from origin feature to callback function.
//Gets the direct communications
direct = containTC o (publish o subscribe) o (inv containTC);

//Gets callback functions.
callbackFuncs = rng(subscribe o call);
cbBelong = containTC o callbackFuncs;
if $2 == "true" {
        callbackFuncs = callbackFuncs . @label;
}

direct = direct + cbBelong;
indirect = direct+;
if $2 == "true" {
	indirect = inv @label o indirect o @label;
}

//Finally, shows the path.
for item in dom indirect {
	for cb in callbackFuncs {
		//Print the combination.
		showpath(item, cb, indirect);
	}
}
