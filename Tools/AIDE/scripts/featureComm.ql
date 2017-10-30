////////////////////////////////////////////////////////////////////////
// featureComm.ql
// By: Bryan J Muscedere
//
// Detects the flow of information between features.
// Sees how information flows from one component to another
// via publishers and subscribers.
////////////////////////////////////////////////////////////////////////

$INSTANCE = eset;

//Sets the input file and loads.
inputFile = $1;
getta(inputFile);

//Ges the direct communications.
direct = publish o subscribe;

//Now, resolves the classes.
direct = contain o direct;
direct = direct o (inv (contain));

//Finally, resolve the plain-English class names. 
inv @label o direct o @label;
print "==========================";

//Gets the indirect communications.
indirect = direct+;
indirect = indirect - direct;

//Finally, resolve the plain-English class names.
inv @label o indirect o @label;
print "==========================";

//Now, get a collection of all pairs.
classes = $INSTANCE . {"cClass"};
matrixClass = classes X classes;
matrixClass = inv @label o matrixClass o @label;
matrixClass;
