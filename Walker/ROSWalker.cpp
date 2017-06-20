#include <fstream>
#include <iostream>
#include "clang/AST/Mangle.h"
#include "ROSWalker.h"
#include "../Graph/RexNode.h"

using namespace std;

TAGraph* ROSWalker::graph = new TAGraph();
vector<TAGraph*> ROSWalker::graphList = vector<TAGraph*>();

ROSWalker::ROSWalker(ASTContext *Context) : Context(Context) { }

ROSWalker::~ROSWalker(){ }

void ROSWalker::deleteTAGraphs(){
    delete graph;
    for (int i = 0; i < graphList.size(); i++)
        delete graphList.at(i);

    graphList = vector<TAGraph*>();
}

void ROSWalker::deleteTAGraph(int num){
    if (num < 0 || num >= graphList.size()) return;
    delete graphList.at(num);
    graphList.erase(graphList.begin() + num);
}

int ROSWalker::getNumGraphs(){
    return (int) graphList.size();
}

int ROSWalker::endCurrentGraph(){
    //Moves the current graph.
    graphList.push_back(graph);
    graph = new TAGraph();

    return (int) graphList.size() - 1;
}

int ROSWalker::generateCurrentTAModel(string fileName){
    return ROSWalker::generateTAModel(graph, fileName);
}

int ROSWalker::generateTAModel(int num, string fileName){
    //Get the graph at the number.
    if (num >= graphList.size() || num < 0) return 0;
    return ROSWalker::generateTAModel(graphList.at(num), fileName);
}

int ROSWalker::generateAllTAModels(vector<string> fileNames){
    if (fileNames.size() != graphList.size()) return 0;
    for (int i = 0; i < fileNames.size(); i++){
        int code = ROSWalker::generateTAModel(graphList.at(i), fileNames.at(i));
        if (code != 1) return 0;
    }

    return 1;
}

bool ROSWalker::VisitStmt(Stmt *statement) {
    if (isInSystemHeader(statement)) return true;

    //Handle ROS Publisher and Subscriber object creations.
    if (CXXConstructExpr* cxxExpr = dyn_cast<CXXConstructExpr>(statement)) {
        //Generate the printing policy.
        clang::LangOptions LangOpts;
        clang::PrintingPolicy Policy(LangOpts);

        if (isNodeHandlerObj(cxxExpr)) {
            recordNodeHandle(cxxExpr);
            recordParentNodeHandle(cxxExpr);
        } else if (isSubscriberObj(cxxExpr)) {
            recordParentSubscribe(cxxExpr);
        } else if (isPublisherObj(cxxExpr)) {
            recordParentPublish(cxxExpr);
        }
    }

    //Handle ROS statements.
    if (CallExpr* callExpr = dyn_cast<CallExpr>(statement)){
        //Deal with the expression.
        if (isPublish(callExpr)) recordPublish(callExpr);
        else if (isSubscribe(callExpr)) recordSubscribe(callExpr);
        else if (isAdvertise(callExpr)) recordAdvertise(callExpr);

        //Next, deal with call expressions.
        recordCallExpr(callExpr);
    }

    if (DeclRefExpr* usageExpr = dyn_cast<DeclRefExpr>(statement)){
        //Deal with the expression.
        recordVarUsage(usageExpr);
    }

    return true;
}

bool ROSWalker::VisitFunctionDecl(FunctionDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Record the function declaration.
    recordFunctionDecl(decl);
    return true;
}

bool ROSWalker::VisitCXXRecordDecl(CXXRecordDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Check what type of structure it is.
    if (decl->isClass()){
        recordClassDecl(decl);
    }
    return true;
}

bool ROSWalker::VisitVarDecl(VarDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Record the function declaration.
    recordVarDecl(decl);
    return true;
}

bool ROSWalker::VisitFieldDecl(FieldDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Record the function declaration.
    recordFieldDecl(decl);
    return true;
}

int ROSWalker::generateTAModel(TAGraph* graph, string fileName){
    //Purge the edges.
    graph->purgeUnestablishedEdges(true);

    //Gets the string for the model.
    string model = graph->getTAModel();

    //Creates the file stream.
    ofstream file(fileName);
    if (!file.is_open()) return 0;

    //Writes to the file.
    file << model;

    file.close();
    return 1;
}

void ROSWalker::recordFunctionDecl(const FunctionDecl* decl){
    //Generates some fields.
    string ID = generateID(decl);
    string name = generateName(decl);

    //Creates the node.
    RexNode* node = new RexNode(ID, name, RexNode::FUNCTION);
    graph->addNode(node);

    //Get the parent.
    addParentRelationship(decl, ID);
}

void ROSWalker::recordClassDecl(const CXXRecordDecl *decl){
    //Generates some fields.
    string ID = generateID(decl);
    string name = generateName(decl);

    //Creates the node.
    RexNode* node = new RexNode(ID, name, RexNode::CLASS);
    graph->addNode(node);

    //Get the parent.
    addParentRelationship(decl, ID);
}

void ROSWalker::recordVarDecl(const VarDecl* decl){
    //Generates some fields.
    string ID = generateID(decl);
    string name = generateName(decl);

    //Creates the node.
    RexNode* node = new RexNode(ID, name, RexNode::VARIABLE);
    graph->addNode(node);

    //Get the parent.
    addParentRelationship(decl, ID);
}

void ROSWalker::recordFieldDecl(const FieldDecl* decl){
    //Generates some fields.
    string ID = generateID(decl);
    string name = generateName(decl);

    //Creates the node.
    RexNode* node = new RexNode(ID, name, RexNode::VARIABLE);
    graph->addNode(node);

    //Get the parent.
    addParentRelationship(decl, ID);
}

void ROSWalker::recordCallExpr(const CallExpr* expr){
    //Get the sub-function.
    auto callee = expr->getCalleeDecl();
    if (callee == nullptr) return;
    auto cDecl = dyn_cast<FunctionDecl>(callee);
    if (cDecl == nullptr) return;

    //Gets the ID for the cDecl.
    string calleeID = generateID(cDecl);
    RexNode* calleeNode = graph->findNode(calleeID);

    //Gets the parent expression.
    auto parDecl = getParentFunction(expr);
    if (parDecl == nullptr) return;

    //Gets the ID for the parent.
    string callerID = generateID(parDecl);
    RexNode* callerNode = graph->findNode(callerID);

    //Adds the edge.
    if (graph->doesEdgeExist(callerID, calleeID, RexEdge::CALLS)) return;
    RexEdge* edge = (calleeNode == nullptr) ?
                    new RexEdge(callerNode, calleeID, RexEdge::CALLS) :
                    new RexEdge(callerNode, calleeNode, RexEdge::CALLS);
    graph->addEdge(edge);
}

//TODO: This doesn't work quite well yet. Fields and ParmDecls aren't captured.
void ROSWalker::recordVarUsage(const DeclRefExpr* expr){
    //Get the sub-variable.
    auto subVar = expr->getFoundDecl();
    string subVarID = generateID(subVar);
    RexNode* subVarNode = graph->findNode(subVarID);

    //Get the parent expression.
    auto parDecl = getParentFunction(expr);
    if (parDecl == nullptr) return;

    //Get the ID for the parent.
    string callerID = generateID(parDecl);
    RexNode* callerNode = graph->findNode(callerID);

    //Checks whether we can resolve.
    if (callerNode == nullptr){
        return;
    }

    //Adds the edge.
    if (graph->doesEdgeExist(callerID, subVarID, RexEdge::REFERENCES)) return;
    RexEdge* edge = (subVarNode == nullptr) ?
                    new RexEdge(callerNode, subVarID, RexEdge::REFERENCES) :
                    new RexEdge(callerNode, subVarNode, RexEdge::REFERENCES);
    graph->addEdge(edge);
}

bool ROSWalker::isNodeHandlerObj(const CXXConstructExpr* ctor){
    return isClass(ctor, NODE_HANDLE_CLASS);
}

bool ROSWalker::isSubscriberObj(const CXXConstructExpr* ctor){
    return isClass(ctor, SUBSCRIBER_CLASS);
}

bool ROSWalker::isPublisherObj(const CXXConstructExpr* ctor){
    return isClass(ctor, PUBLISHER_CLASS);
}

bool ROSWalker::isPublish(const CallExpr *expr) {
    return isFunction(expr, PUBLISH_FUNCTION);
}

bool ROSWalker::isSubscribe(const CallExpr *expr) {
    return isFunction(expr, SUBSCRIBE_FUNCTION);
}

bool ROSWalker::isAdvertise(const CallExpr* expr){
    return isFunction(expr, ADVERTISE_FUNCTION);
}

bool ROSWalker::isFunction(const CallExpr *expr, string functionName){
    //Gets the underlying callee.
    if (expr->getCalleeDecl() == nullptr) return false;
    auto callee = expr->getCalleeDecl()->getAsFunction();
    if (callee == nullptr) return false;

    //Checks the value of the callee.
    if (callee->getQualifiedNameAsString().compare(functionName)) return false;
    return true;
}

bool ROSWalker::isClass(const CXXConstructExpr* ctor, string className){
    //Get the underlying class.
    QualType type = ctor->getBestDynamicClassTypeExpr()->getType();
    if (type->isArrayType()) return false; //TODO: Bandaid fix! Not sure why this works.

    auto record = ctor->getBestDynamicClassType();
    if (record == nullptr) return false;

    //Check the qualified name.
    if (record->getQualifiedNameAsString().compare(className)) return false;
    return true;
}

void ROSWalker::recordParentSubscribe(const CXXConstructExpr* expr){
    //First, see if we have parents.
    const NamedDecl* parentVar = getParentAssign(expr);
    if (!parentVar) return;

    //Next, we record the relationship between the two items.
    RexNode* src = graph->findNode(generateID(parentVar));
    src->addSingleAttribute(ROS_SUB_VAR_FLAG, "1");
    RexNode* dst = graph->generateSubscriberNode(generateID(parentVar), generateName(parentVar));

    //Creates the edge.
    RexEdge* edge = new RexEdge(src, dst, RexEdge::SUBSCRIBE);
    graph->addEdge(edge);

    //Keeps track of the node being published.
    currentSubscriber = dst;
}

void ROSWalker::recordParentPublish(const CXXConstructExpr* expr){
    //First, see if we have parents.
    const NamedDecl* parentVar = getParentAssign(expr);
    if (!parentVar) return;

    //Next, we record the relationship between the two items.
    RexNode* src = graph->findNode(generateID(parentVar));
    src->addSingleAttribute(ROS_PUB_VAR_FLAG, "1");
    RexNode* dst = graph->generatePublisherNode(generateID(parentVar), generateName(parentVar));

    //Creates the edge.
    RexEdge* edge = new RexEdge(src, dst, RexEdge::SUBSCRIBE);
    graph->addEdge(edge);

    //Keeps track of the node being published.
    currentPublisher = dst;
}

void ROSWalker::recordParentNodeHandle(const CXXConstructExpr* expr){

}

void ROSWalker::recordTopic(string name){
    string ID = TOPIC_PREFIX + name;
    if (graph->doesNodeExist(ID)) return;

    //Create the node.
    RexNode* node = new RexNode(ID, name, RexNode::TOPIC);
    graph->addNode(node);
}

void ROSWalker::recordNodeHandle(const CXXConstructExpr* expr){

}

void ROSWalker::recordSubscribe(const CallExpr* expr){
    if (currentSubscriber == nullptr) return;

    //Now, adds information to the destination node.
    int numArgs = expr->getNumArgs();
    auto subscriberArgs = getArgs(expr);

    //Get the name of the topic and record it if not present.
    string topicName = validateStringArg(subscriberArgs[0]);
    recordTopic(topicName);
    RexNode* topic = graph->findNode(TOPIC_PREFIX + topicName);
    RexEdge* topEdge = new RexEdge(currentSubscriber, topic, RexEdge::SUBSCRIBE);
    graph->addEdge(topEdge);

    //Record attributes.
    currentSubscriber->addSingleAttribute(ROS_TOPIC_BUF_SIZE, subscriberArgs[1]);
    currentSubscriber->addSingleAttribute(ROS_NUM_ATTRIBUTES, to_string(numArgs));
    currentSubscriber->addSingleAttribute(ROS_CALLBACK, subscriberArgs[2]);

    //Gets the callback information.
    RexNode* callback = findCallbackFunction(subscriberArgs[2]);

    //Finally, adds in the callback function.
    RexEdge* callbackEdge;
    if (callback){
        callbackEdge = new RexEdge(currentSubscriber, callback, RexEdge::CALLS);
    } else {
        callbackEdge = new RexEdge(currentSubscriber, subscriberArgs[2], RexEdge::CALLS);
    }
    graph->addEdge(callbackEdge);

    currentSubscriber = nullptr;
}

//TODO: This won't work since the qualified name isn't right.
void ROSWalker::recordPublish(const CallExpr* expr){
    //Get the publisher object.
    auto parent = getParentVariable(expr);
    if (parent.compare(string()) == 0) return;
    RexNode* parentVar = graph->findNodeByName(parent);
    if (!parentVar) return;

    //Next, gets the publisher that it's pointing to.
    auto destinations = graph->findEdgesBySrc(parentVar->getID(), false);
    RexNode* pubItem = nullptr;
    for (int i = 0; i < destinations.size(); i++){
        auto item = destinations.at(i);

        //Narrows down the item.
        if (item->getType() != RexEdge::SUBSCRIBE) continue;
        auto dest = item->getDestination();
        if (dest->getType() != RexNode::PUBLISHER) continue;

        if (pubItem == nullptr){
            pubItem = dest;
        } else if (atoi(pubItem->getSingleAttribute(ROS_NUMBER).c_str()) < atoi(item->getSingleAttribute(ROS_NUMBER).c_str())){
            pubItem = dest;
        }
    }
    if (!pubItem) return;

    //Next, gets the topic it publishes to.
    destinations = graph->findEdgesBySrc(pubItem->getID(), false);
    RexNode* topic = nullptr;
    for (int i = 0; i < destinations.size(); i++){
        auto item = destinations.at(i);

        //Narrows the item.
        if (item->getType() != RexEdge::ADVERTISE) continue;
        topic = item->getDestination();
    }
    if (!topic) return;

    //Now, generates the edge.
    RexEdge* edge = new RexEdge(parentVar, topic, RexEdge::PUBLISH);

    //Gets the publish data.
    auto args = getArgs(expr);
    string data = validateStringArg(args.at(0));
    if (data.size() > PUB_MAX) data = data.substr(0, PUB_MAX);
    edge->addSingleAttribute(ROS_PUB_DATA, data);

    //Adds the result to the graph.
    graph->addEdge(edge);
}

void ROSWalker::recordAdvertise(const CallExpr* expr) {
    if (currentPublisher == nullptr) return;

    //First, get the arguments.
    int numArgs = expr->getNumArgs();
    auto publisherArgs = getArgs(expr);

    //Get the name of the topic and record it if not present.
    string topicName = validateStringArg(publisherArgs[0]);
    recordTopic(topicName);
    RexNode* topic = graph->findNode(TOPIC_PREFIX + topicName);
    RexEdge* topEdge = new RexEdge(currentPublisher, topic, RexEdge::ADVERTISE);
    graph->addEdge(topEdge);

    //Record specific attributes.
    currentPublisher->addSingleAttribute(ROS_TOPIC_BUF_SIZE, publisherArgs[1]);
    currentPublisher->addSingleAttribute(ROS_NUM_ATTRIBUTES, to_string(numArgs));
    currentPublisher->addSingleAttribute(ROS_PUB_TYPE, getPublisherType(expr));

    currentPublisher = nullptr;
}

RexNode* ROSWalker::findCallbackFunction(std::string callbackQualified){
    //First, check to see if the string starts with &.
    if (callbackQualified.at(0) == '&'){
        callbackQualified = callbackQualified.erase(0, 1);
    }

    //Find a node by name.
    return graph->findNodeByName(callbackQualified);
}

vector<string> ROSWalker::getArgs(const CallExpr* expr){
    string sBuffer;

    //Creates a vector.
    vector<string> args;

    //Get the arguments for the CallExpr.
    int numArgs = expr->getNumArgs();
    auto argList = expr->getArgs();
    for (int i = 0; i < numArgs; i++){
        sBuffer = "";
        auto curArg = argList[i];

        //Pushes the argument string into a vector.
        llvm::raw_string_ostream strStream(sBuffer);
        curArg->printPretty(strStream, nullptr, Context->getPrintingPolicy());
        sBuffer = strStream.str();
        args.push_back(sBuffer);
    }

    return args;
}

string ROSWalker::getPublisherType(const CallExpr* expr) {
    //Get the string representation of the expression.
    string sBuffer = "";
    llvm::raw_string_ostream strStream(sBuffer);
    expr->printPretty(strStream, nullptr, Context->getPrintingPolicy());
    sBuffer = strStream.str();

    //Loop through the string.
    string type = "";
    bool inBetween = false;
    for (int i = 0; i < sBuffer.size(); i++){
        if (sBuffer.at(i) == '<'){
            inBetween = true;
        } else if (inBetween && sBuffer.at(i) == '>'){
            break;
        } else if (inBetween) {
            type += sBuffer.at(i);
        }
    }

    if (type.compare("") == 0) return "none";
    return type;
}

bool ROSWalker::isInSystemHeader(const Stmt *statement) {
    if (statement == nullptr) return false;

    //Get the system header.
    bool isIn;
    try {
        //Get the source manager.
        auto &manager = Context->getSourceManager();

        //Check if in header.
        isIn = isInSystemHeader(manager, statement->getLocStart());
    } catch (...) {
        return false;
    }

    return isIn;
}

bool ROSWalker::isInSystemHeader(const Decl *decl){
    if (decl == nullptr) return false;

    //Get the system header.
    bool isIn;
    try {
        //Get the source manager.
        auto &manager = Context->getSourceManager();

        //Check if in header.
        isIn = isInSystemHeader(manager, decl->getLocStart());
    } catch (...) {
        return false;
    }

    return isIn;
}

bool ROSWalker::isInSystemHeader(const SourceManager& manager, SourceLocation loc) {
    //Get the expansion location.
    auto expansionLoc = manager.getExpansionLoc(loc);

    //Check if we have a valid location.
    if (expansionLoc.isInvalid()) {
        return false;
    }
    //Get if we have a system header.
    return manager.isInSystemHeader(loc);
}

void ROSWalker::addParentRelationship(const NamedDecl* baseDecl, string baseID){
    bool getParent = true;
    auto currentDecl = baseDecl;

    //Get the parent.
    auto parent = Context->getParents(*currentDecl);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current decl as named.
        currentDecl = parent[0].get<clang::NamedDecl>();
        if (currentDecl) {
            //Get the parent as a NamedDecl.
            string parentID = "";

            if (isa<clang::FunctionDecl>(currentDecl)){
                const FunctionDecl* funcDec = static_cast<const FunctionDecl*>(currentDecl);
                parentID = generateID(funcDec);
            } else {
                parentID = generateID(currentDecl);
            }

            //Get the IDs.
            RexNode* dst = graph->findNode(baseID);
            RexNode* src = graph->findNode(parentID);
            if (graph->doesEdgeExist(parentID, baseID, RexEdge::CONTAINS)){
                return;
            }

            RexEdge* edge = (!src) ? new RexEdge(parentID, dst, RexEdge::CONTAINS) :
                                     new RexEdge(src, dst, RexEdge::CONTAINS);
            graph->addEdge(edge);
            return;
        }

        parent = Context->getParents(parent[0]);
    }
}

const FunctionDecl* ROSWalker::getParentFunction(const Expr* callExpr){
    bool getParent = true;

    //Get the parent.
    auto parent = Context->getParents(*callExpr);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current decl as named.
        auto currentDecl = parent[0].get<clang::NamedDecl>();
        if (currentDecl && isa<clang::FunctionDecl>(currentDecl)){
            return dyn_cast<FunctionDecl>(currentDecl);
        }

        parent = Context->getParents(parent[0]);
    }

    return nullptr;
}

string ROSWalker::getParentVariable(const Expr *callExpr) {
    //Get the CXX Member Call.
    auto call = dyn_cast<CXXMemberCallExpr>(callExpr);
    if (call == nullptr) return string();

    //Next, get the implicit object.
    auto parentVar = call->getImplicitObjectArgument();
    if (parentVar == nullptr) return string();

    //Finally, works on a print pretty system.
    string sBuffer = "";
    llvm::raw_string_ostream strStream(sBuffer);
    parentVar->printPretty(strStream, nullptr, Context->getPrintingPolicy());
    return strStream.str();
}

const NamedDecl* ROSWalker::getParentAssign(const CXXConstructExpr* expr){
    bool getParent = true;

    //Get the parent.
    auto parent = Context->getParents(*expr);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current decl as named.
        auto currentDecl = parent[0].get<clang::NamedDecl>();
        if (currentDecl && (isa<clang::VarDecl>(currentDecl) || isa<clang::FieldDecl>(currentDecl)
                           || isa<clang::ParmVarDecl>(currentDecl))){
            return currentDecl;
        } else if (currentDecl && isa<clang::FunctionDecl>(currentDecl)){
            getParent = false;
            continue;
        }

        parent = Context->getParents(parent[0]);
    }

    return nullptr;
}

string ROSWalker::generateID(const FunctionDecl* decl){
    auto mangleContext = Context->createMangleContext();

    //Check whether we need to mangle.
    if (!mangleContext->shouldMangleDeclName(decl)) {
        if (decl->getDefinition() != nullptr)
            decl = decl->getDefinition();

        //Gets the return type and then the qualified name.
        string qualName = decl->getReturnType().getAsString() + "--";
        qualName += decl->getQualifiedNameAsString();

        //Gets the return types.
        int numParam = decl->getNumParams();
        for (int i = 0; i < numParam; i++){
            const ParmVarDecl* parm = decl->getParamDecl(i);
            qualName += "--" + parm->getType().getAsString();
        }

        return qualName;
    }

    //Mangle the name
    string mangledName;
    raw_string_ostream stream(mangledName);

    //Check the type.
    if (isa<CXXDestructorDecl>(decl)){
        auto dtorDecl = dyn_cast<CXXDestructorDecl>(decl);
        mangleContext->mangleCXXDtor(dtorDecl, CXXDtorType::Dtor_Complete, stream);
    } else if (isa<CXXConstructorDecl>(decl)){
        auto ctorDecl = dyn_cast<CXXConstructorDecl>(decl);
        mangleContext->mangleCXXCtor(ctorDecl, CXXCtorType::Ctor_Complete, stream);
    } else {
        mangleContext->mangleCXXName(decl,stream);
    }

    stream.flush();
    return mangledName;
}

string ROSWalker::generateID(const NamedDecl* decl){
    string name = decl->getNameAsString();
    bool getParent = true;
    bool recurse = false;
    const NamedDecl* originalDecl = decl;

    //Get the parent.
    auto parent = Context->getParents(*decl);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current decl as named.
        decl = parent[0].get<clang::NamedDecl>();
        if (decl) {
            name = generateID(decl) + "::" + name;
            recurse = true;
            getParent = false;
            continue;
        }

        parent = Context->getParents(parent[0]);
    }

    //Sees if no true qualified name was used.
    Decl::Kind kind = originalDecl->getKind();
    if (!recurse) {
        if (kind == Decl::Function || kind == Decl::CXXMethod){
            name = originalDecl->getQualifiedNameAsString();
        } else {
            //We need to get the parent function.
            const DeclContext *parentContext = originalDecl->getParentFunctionOrMethod();

            //If we have nullptr, get the parent function.
            if (parentContext != nullptr) {
                string parentQualName = generateID(static_cast<const FunctionDecl *>(parentContext));
                name = parentQualName + "::" + originalDecl->getNameAsString();
            }
        }
    }

    int lineNum = Context->getSourceManager().getSpellingLineNumber(originalDecl->getSourceRange().getBegin());
    name = name + "--" + std::to_string(lineNum);
    return name;
}

string ROSWalker::generateName(const NamedDecl* decl){
    return decl->getQualifiedNameAsString();
}

string ROSWalker::validateStringArg(string name){
    //Checks the topic name.
    std::string prefix("\"");
    if (!name.compare(0, prefix.size(), prefix)){
        name = name.substr(1);
    }
    if (!name.compare(name.size() - 1, prefix.size(), prefix)){
        name = name.substr(0, name.size() - 1);
    }

    return name;
}

ROSConsumer::ROSConsumer(ASTContext *Context) : Visitor(Context) {}

void ROSConsumer::HandleTranslationUnit(ASTContext &Context) {
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());
}

std::unique_ptr<ASTConsumer> ROSAction::CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile) {
    return std::unique_ptr<ASTConsumer>(new ROSConsumer(&Compiler.getASTContext()));
}