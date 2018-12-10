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

direct = direct + cbBelong;
indirect = direct+;

//Finally, shows the path.
for item in dom indirect {
	for cb in callbackFuncs {
		//Print the combination.
		showpath(item, cb, indirect);
	}
}
