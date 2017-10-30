////////////////////////////////////////////////////////////////////////
// controlFlow.ql
// By: Bryan J Muscedere
//
// Detects hotspots between components where one component
// affects another component's variable that then alters how
// the component operates. Looks for control flow information.
////////////////////////////////////////////////////////////////////////

$INSTANCE = eset;

//Sets the input file and loads.
inputFile = $1;
getta(inputFile);

//Gets a list of subscribers.
subs = $INSTANCE . {"rosSubscriber"};

//Get the direct component calls.
direct = publish o subscribe;

//Gets the variables that have a control flow of 1.
controlVars = @isControlFlow . {"\"1\""}
callbackFuncs = subs . call;
callbackVars = callbackFuncs o write;
callbackControlVars = callbackVars o controlVars;

//Display the subscribers and the variables they write to.
(inv @label) o callbackControlVars o @label;
print "==========================";

//Next, we want to get the components which directly affect this component.
directDst = (subs o call) . dom (callbackControlVars);
directComp = direct o directDst;
directComp = contain o directComp o (inv (contain));

//Finally, we get the components which indirectly affect this component.
direct = (contain o direct) o (inv (contain));
indirect = direct+;
indirectComp = indirect o (contain . directDst);
indirectComp = indirectComp - directComp;

//Now, we display communications.
(inv @label) o directComp o @label;
print "==========================";
(inv @label) o indirectComp o @label;
print "==========================";

//Now, get a collection of all pairs.
classes = $INSTANCE . {"cClass"};
matrixClass = classes X classes;
matrixClass = inv @label o matrixClass o @label;
matrixClass;