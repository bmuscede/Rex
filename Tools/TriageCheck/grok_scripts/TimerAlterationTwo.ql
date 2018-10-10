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
if $# == 2 {
	for str in dom CSVDATA {
		toRemove = grep(timers, str);
		tim = timers - toRemove;
	}
}
tmrCallback = rng time;
if $# == 2 {
	for str in dom CSVDATA {
		toRemove = grep(tmrCallback, str);
		tmrCallback = tmrCallback - toRemove;
	}
}

//Generates a master relation.
masterRel = varWrite + varInfluence + varInfFunc + call + write;
masterRel = masterRel+;

//Gets all cases of timer alteration.
timerAlter = timers o time o tmrCallback o masterRel o publish;

//Print the results.
if #timerAlter > 0 {
	print "There are " + #timerAlter + " cases of timer alteration.";
	print "";
} else {
	print "There are no cases of timer alteration.";
	quit;
}

//Loops through and presents the results.
for timer in dom timerAlter {
	print "---------------------------------------------------------";
	print "Timer:"
	{timer} . @label;
	print "";
	
	for topic in {timer} . timerAlter {
		res = timer + topic;
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

	dirInf = {timer} . timerAlter . direct;
	print "Directly Influences:";
	if (#dirInf > 0) {
		dirInf . @label;
	} else {
		print "<NONE>";
	}
	print "";
	
	inInf = ({timer} . timerAlter . direct) . indirect;
	print "Indirectly Influences:";
	inInf = inInf - dirInf;
	if (#inInf > 0) {
	      	inInf . @label;
	} else {
		print "<NONE>";
	}

	print "---------------------------------------------------------";
}
