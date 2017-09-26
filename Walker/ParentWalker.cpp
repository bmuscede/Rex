//
// Created by bmuscede on 06/07/17.
//

#include <fstream>
#include <iostream>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "ParentWalker.h"

using namespace std;

TAGraph* ParentWalker::graph = new TAGraph();
vector<TAGraph*> ParentWalker::graphList = vector<TAGraph*>();

ParentWalker::ParentWalker(ASTContext *Context) : Context(Context) {
    ignoreLibraries.push_back(STANDARD_IGNORE);
}

ParentWalker::~ParentWalker() {}

void ParentWalker::deleteTAGraphs(){
    delete graph;
    for (int i = 0; i < graphList.size(); i++)
        delete graphList.at(i);

    graphList = vector<TAGraph*>();
}

void ParentWalker::deleteTAGraph(int num){
    if (num < 0 || num >= graphList.size()) return;
    delete graphList.at(num);
    graphList.erase(graphList.begin() + num);
}

int ParentWalker::getNumGraphs(){
    return (int) graphList.size();
}

int ParentWalker::endCurrentGraph(){
    //Moves the current graph.
    graphList.push_back(graph);
    graph = new TAGraph();

    return (int) graphList.size() - 1;
}

int ParentWalker::generateCurrentTAModel(string fileName){
    return ParentWalker::generateTAModel(graph, fileName);
}

int ParentWalker::generateTAModel(int num, string fileName){
    //Get the graph at the number.
    if (num >= graphList.size() || num < 0) return 0;
    return ParentWalker::generateTAModel(graphList.at(num), fileName);
}

int ParentWalker::generateAllTAModels(vector<string> fileNames){
    if (fileNames.size() != graphList.size()) return 0;
    for (int i = 0; i < fileNames.size(); i++){
        int code = ParentWalker::generateTAModel(graphList.at(i), fileNames.at(i));
        if (code != 1) return 0;
    }

    return 1;
}

void ParentWalker::setCurrentGraphMinMode(bool minMode){
    graph->setMinMode(minMode);
}

void ParentWalker::addLibrariesToIgnore(vector<string> libraries){
    ignoreLibraries = libraries;
}

map<string, ParentWalker::AccessMethod> ParentWalker::getAccessType(const BinaryOperator* op){
    map<string, ParentWalker::AccessMethod> usageMap;

    //Checks the lefthand side.
    Expr* lhs = op->getLHS();
    if (isa<DeclRefExpr>(lhs)){
        usageMap[generateID(dyn_cast<DeclRefExpr>(lhs)->getDecl())] = determineAccess(true, op->getOpcode());
    } else if (isa<MemberExpr>(lhs)) {
        usageMap[generateID(dyn_cast<MemberExpr>(lhs)->getMemberDecl())] = determineAccess(true, op->getOpcode());
    } else  {
        //Get the opcode.
        ParentWalker::AccessMethod method = determineAccess(true, op->getOpcode());

        //Get the usage map.
        usageMap = buildAccessMap(method, lhs);

        //Iterates through the map and combines.
        map<string, ParentWalker::AccessMethod>::iterator it;
        for (it = usageMap.begin(); it != usageMap.end(); it++){
            if ((usageMap[it->first] == ParentWalker::AccessMethod::READ && method == ParentWalker::AccessMethod::WRITE ||
                 usageMap[it->first] == ParentWalker::AccessMethod::WRITE && method == ParentWalker::AccessMethod::READ)){
                usageMap[it->first] = ParentWalker::BOTH;
            }
        }
    }

    //Checks the righthand side.
    Expr* rhs = op->getRHS();
    if (isa<DeclRefExpr>(rhs) || isa<MemberExpr>(rhs)){
        string addID = (isa<DeclRefExpr>(rhs) ? generateID(dyn_cast<DeclRefExpr>(rhs)->getDecl()) :
                        generateID(dyn_cast<MemberExpr>(rhs)->getMemberDecl()));
        ParentWalker::AccessMethod method = determineAccess(false, op->getOpcode());

        //Checks what we do.
        if (usageMap.find(addID) != usageMap.end() &&
                (usageMap[addID] == ParentWalker::AccessMethod::READ && method == ParentWalker::AccessMethod::WRITE ||
                        usageMap[addID] == ParentWalker::AccessMethod::WRITE && method == ParentWalker::AccessMethod::READ)){
            usageMap[addID] = ParentWalker::AccessMethod::BOTH;
        } else {
            usageMap[addID] = method;
        }
    } else {
        //Get the opcode.
        ParentWalker::AccessMethod method = determineAccess(false, op->getOpcode());

        //First, merge into map.
        map<string, ParentWalker::AccessMethod> otherMap = buildAccessMap(method, rhs);
        map<string, ParentWalker::AccessMethod>::iterator it;
        for (it = otherMap.begin(); it != otherMap.end(); it++){
            //Checks what we do.
            if (usageMap.find(it->first) != usageMap.end() &&
                (usageMap[it->first] == ParentWalker::AccessMethod::READ && it->second == ParentWalker::AccessMethod::WRITE ||
                 usageMap[it->first] == ParentWalker::AccessMethod::WRITE && it->second == ParentWalker::AccessMethod::READ)){
                usageMap[it->first] = ParentWalker::AccessMethod::BOTH;
            } else {
                usageMap[it->first] = it->second;
            }

            //Now, checks to see what the method is.
            if (usageMap[it->first] == ParentWalker::AccessMethod::READ && method == ParentWalker::AccessMethod::WRITE ||
                usageMap[it->first] == ParentWalker::AccessMethod::WRITE && method == ParentWalker::AccessMethod::READ){
                usageMap[it->first] = ParentWalker::AccessMethod::BOTH;
            }
        }

    }

    return usageMap;
}

map<string, ParentWalker::AccessMethod> ParentWalker::getAccessType(const UnaryOperator* op){
    map<string, ParentWalker::AccessMethod> usageMap;

    //First, determine the inner expr. We want to be sure we have the correct ID.
    Expr* inner = op->getSubExpr();

    //Checks if we're dealing with a singular item.
    if (isa<DeclRefExpr>(inner)){
        //Get the opcode.
        ParentWalker::AccessMethod method = determineAccess(op->getOpcode());
        if (method != ParentWalker::AccessMethod::NONE) {
            usageMap[generateID(dyn_cast<DeclRefExpr>(inner)->getDecl())] = method;
        }
    } else if (isa<MemberExpr>(inner)) {
        //Get the opcode.
        ParentWalker::AccessMethod method = determineAccess(op->getOpcode());
        if (method != ParentWalker::AccessMethod::NONE) {
            usageMap[generateID(dyn_cast<MemberExpr>(inner)->getMemberDecl())] = method;
        }
    } else {
        //Get the opcode.
        ParentWalker::AccessMethod method = determineAccess(op->getOpcode());

        //We just return the base usage map.
        usageMap = buildAccessMap(method, inner);
    }

    return usageMap;
}

void ParentWalker::handleMinimalStmt(Stmt *statement) {
    //Looks for advertise and subscribe.
    if (CXXOperatorCallExpr* overload = dyn_cast<CXXOperatorCallExpr>(statement)){
        string type = overload->getType().getAsString();
        if (type.compare("class " + PUBLISHER_CLASS) == 0){
            const NamedDecl* assignee = getAssignee(overload);
            const MemberExpr* assignStmt = getAssignStmt(overload);
            if (!assignee || !assignStmt) return;

            recordAssociations(assignee, assignStmt, type);
        } else if (type.compare("class " + SUBSCRIBER_CLASS) == 0) {
            const NamedDecl *assignee = getAssignee(overload);
            const MemberExpr *assignStmt = getAssignStmt(overload);
            if (!assignee || !assignStmt) return;

            recordAssociations(assignee, assignStmt, type);
        }
    }

    //Also looks for advertise and subscribe.
    //TODO: There are some cases where this is used as opposed to the block above. Figure this out.
    if (CXXConstructExpr* cxxExpr = dyn_cast<CXXConstructExpr>(statement)) {
/*
        //Generate the printing policy.
        clang::LangOptions LangOpts;
        clang::PrintingPolicy Policy(LangOpts);

        string stmt;
        raw_string_ostream stream(stmt);
        cxxExpr->printPretty(stream, NULL, PrintingPolicy(LangOptions()));

        //Flush ostream buffer.
        stream.flush();
        cout << stmt << endl;

        if (isNodeHandlerObj(cxxExpr)) {
            recordNodeHandle(cxxExpr);
            recordParentNodeHandle(cxxExpr);
        } else if (isSubscriberObj(cxxExpr)) {
            recordParentSubscribe(cxxExpr, fileName);
*/
        if (isSubscriberObj(cxxExpr)) {
            // recordParentSubscribe(cxxExpr, fileName);
        } else if (isPublisherObj(cxxExpr)) {
            //  recordParentPublish(cxxExpr, fileName);
        }
    }

    //Handles the internals of each action.
    if (CallExpr* callExpr = dyn_cast<CallExpr>(statement)){
        //Deal with the expression.
        if (isPublish(callExpr)) {
            recordPublish(callExpr);
        } else if (isSubscribe(callExpr)) {
            recordSubscribe(callExpr);
        } else if (isAdvertise(callExpr)) {
            recordAdvertise(callExpr);
        }
    }
}

void ParentWalker::handleMinimalVarDecl(VarDecl *decl, bool pubEdge) {
    //Add ROS specific nodes and fields.
    CXXRecordDecl* parent = decl->getType()->getAsCXXRecordDecl();
    if (!parent) return;

    //Checks if we have a publisher or such.
    string parentName = parent->getQualifiedNameAsString();
    if (parentName.compare(PUBLISHER_CLASS) == 0 ||
        parentName.compare(SUBSCRIBER_CLASS) == 0 ||
        parentName.compare(NODE_HANDLE_CLASS) == 0){
        recordROSActionMinimal(decl, parentName, pubEdge);
    }
}

void ParentWalker::handleMinimalFieldDecl(FieldDecl *decl, bool pubEdge) {
    //Add ROS specific nodes and fields.
    CXXRecordDecl* parent = decl->getType()->getAsCXXRecordDecl();
    if (!parent) return;

    //Checks if we have a publisher or such.
    string parentName = parent->getQualifiedNameAsString();
    if (parentName.compare(PUBLISHER_CLASS) == 0 || parentName.compare(SUBSCRIBER_CLASS) == 0 ||
        parentName.compare(NODE_HANDLE_CLASS) == 0){
        recordROSActionMinimal(decl, parentName, pubEdge);
    }
}

bool ParentWalker::isNodeHandlerObj(const CXXConstructExpr* ctor){
    return isClass(ctor, NODE_HANDLE_CLASS);
}

bool ParentWalker::isSubscriberObj(const CXXConstructExpr* ctor){
    return isClass(ctor, SUBSCRIBER_CLASS);
}

bool ParentWalker::isPublisherObj(const CXXConstructExpr* ctor){
    return isClass(ctor, PUBLISHER_CLASS);
}

bool ParentWalker::isPublish(const CallExpr *expr) {
    return isFunction(expr, PUBLISH_FUNCTION);
}

bool ParentWalker::isSubscribe(const CallExpr *expr) {
    return isFunction(expr, SUBSCRIBE_FUNCTION);
}

bool ParentWalker::isAdvertise(const CallExpr* expr){
    return isFunction(expr, ADVERTISE_FUNCTION);
}

void ParentWalker::recordAssociations(const NamedDecl* assignee, const MemberExpr* assign, string type) {
    //Get the assignee with the name.
    string assigneeID = generateID(assignee);
    RexNode *assigneeNode = graph->findNode(assigneeID);

    //Get the NH object.
    MemberExpr *base = dyn_cast<MemberExpr>(assign->getBase());
    if (base != nullptr) {
        NamedDecl *assigner = dyn_cast<NamedDecl>(base->getReferencedDeclOfCallee());
        string assignerID = generateID(assigner);
        RexNode *assignerNode = graph->findNode(assignerID);

        //Now, creates an edge
        RexEdge *edge = new RexEdge(assignerNode, assigneeNode, RexEdge::EdgeType::REFERENCES);
        graph->addEdge(edge);
    }

    //Sets the type.
    if (type.compare("class " + PUBLISHER_CLASS) == 0){
        currentPublisher = assigneeNode;
    } else {
        currentSubscriber = assigneeNode;
    }
}

void ParentWalker::recordROSActionMinimal(const NamedDecl* decl, string type, bool pubEdge){
    //First, get the parent class.
    auto parent = Context->getParents(*decl);
    const CXXRecordDecl* parentFinal = nullptr;
    while(true){
        //Check if it's empty.
        if (parent.empty()) break;

        //Get the current decl as named.
        parentFinal = parent[0].get<clang::CXXRecordDecl>();
        if (parentFinal) break;

        parent = Context->getParents(parent[0]);
    }

    //First, we want to add the class as a node.
    RexNode* classNode = nullptr;
    if (parentFinal){
        string className = parentFinal->getQualifiedNameAsString();

        classNode = new RexNode(className, className, RexNode::NodeType::CLASS);
        graph->addNode(classNode);
    }

    //Next, we make our Rex variable object.
    string varNodeID = generateID(decl);
    string varNodeName = generateName(decl);
    RexNode::NodeType nType = (type.compare(PUBLISHER_CLASS) == 0 ? RexNode::PUBLISHER :
                               ((type.compare(SUBSCRIBER_CLASS) == 0) ? RexNode::SUBSCRIBER :
                                RexNode::NodeType::NODE_HANDLE));

    RexNode* rexVarNode = new RexNode(varNodeID, varNodeName, nType);
    graph->addNode(rexVarNode);

    if (classNode && pubEdge) {
        RexEdge *edge = new RexEdge(classNode, rexVarNode, RexEdge::EdgeType::CONTAINS);
        graph->addEdge(edge);
    }
}

const NamedDecl* ParentWalker::getAssignee(const CXXOperatorCallExpr* parent){
    //Gets the lhs of the expression.
    int num = parent->getNumArgs();
    if (num != 2) return nullptr;
    const Expr* lhs = parent->getArg(0);

    //Gets the base expression type.
    const MemberExpr* lhsCast = dyn_cast<MemberExpr>(lhs);
    if (!lhsCast) return nullptr;

    //Next, gets the referenced item.
    const NamedDecl* decl = dyn_cast<NamedDecl>(lhsCast->getReferencedDeclOfCallee());
    return decl;
}

const MemberExpr* ParentWalker::getAssignStmt(const CXXOperatorCallExpr* parent){
    int num = parent->getNumArgs();
    if (num != 2) return nullptr;
    const Expr* rhs = parent->getArg(1);

    //Digs down into all the temporary expressions.
    const MaterializeTemporaryExpr* tempExpr1 = dyn_cast<MaterializeTemporaryExpr>(rhs);
    const CXXMemberCallExpr* tempExpr2 = dyn_cast<CXXMemberCallExpr>(dyn_cast<CXXBindTemporaryExpr>(
            dyn_cast<ImplicitCastExpr>(tempExpr1->GetTemporaryExpr())->getSubExpr())->getSubExpr());
    const MemberExpr* lastItem = dyn_cast<MemberExpr>(tempExpr2->getCallee());
    return lastItem;
}

bool ParentWalker::isClass(const CXXConstructExpr* ctor, string className){
    //Get the underlying class.
    QualType type = ctor->getType(); //Different clang version QualType type = ctor->getBestDynamicClassTypeExpr()->getType();
    if (type->isArrayType()) return false; //TODO: Bandaid fix! Not sure why this works.

    auto record = ctor->getBestDynamicClassType();
    if (record == nullptr) return false;

    //Check the qualified name.
    if (record->getQualifiedNameAsString().compare(className)) return false;
    return true;
}

bool ParentWalker::isFunction(const CallExpr *expr, string functionName){
    //Gets the underlying callee.
    if (expr->getCalleeDecl() == nullptr) return false;
    auto callee = expr->getCalleeDecl()->getAsFunction();
    if (callee == nullptr) return false;

    //Checks the value of the callee.
    if (callee->getQualifiedNameAsString().compare(functionName)) return false;
    return true;
}

const NamedDecl* ParentWalker::getParentAssign(const CXXConstructExpr* expr){
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

RexNode* ParentWalker::findCallbackFunction(std::string callbackQualified){
    //First, check to see if the string starts with &.
    if (callbackQualified.at(0) == '&'){
        callbackQualified = callbackQualified.erase(0, 1);
    }

    //Find a node by name.
    return graph->findNodeByName(callbackQualified);
}

vector<string> ParentWalker::getArgs(const CallExpr* expr){
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

string ParentWalker::getPublisherType(const CallExpr* expr) {
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

NamedDecl* ParentWalker::getParentVariable(const Expr *callExpr) {
    //Get the CXX Member Call.
    auto call = dyn_cast<CXXMemberCallExpr>(callExpr);
    if (call == nullptr) return nullptr;

    //Next, get the implicit object.
    auto parentVar = call->getImplicitObjectArgument();
    if (parentVar == nullptr) return nullptr;

    return dyn_cast<NamedDecl>(parentVar->getReferencedDeclOfCallee());

    //Finally, works on a print pretty system.
    //string sBuffer = "";
    //llvm::raw_string_ostream strStream(sBuffer);
    //parentVar->printPretty(strStream, nullptr, Context->getPrintingPolicy());
    //return strStream.str();
}

ParentWalker::AccessMethod ParentWalker::determineAccess(bool lhs, BinaryOperator::Opcode opcode){
    if (!lhs) return ParentWalker::AccessMethod::READ;

    switch(opcode){
        case BinaryOperator::Opcode::BO_Assign:
        case BinaryOperator::Opcode::BO_AddAssign:
        case BinaryOperator::Opcode::BO_DivAssign:
        case BinaryOperator::Opcode::BO_AndAssign:
        case BinaryOperator::Opcode::BO_MulAssign:
        case BinaryOperator::Opcode::BO_OrAssign:
        case BinaryOperator::Opcode::BO_RemAssign:
        case BinaryOperator::Opcode::BO_ShlAssign:
        case BinaryOperator::Opcode::BO_ShrAssign:
        case BinaryOperator::Opcode::BO_SubAssign:
        case BinaryOperator::Opcode::BO_XorAssign:
            return ParentWalker::AccessMethod::WRITE;
        case BinaryOperator::Opcode::BO_PtrMemD:
        case BinaryOperator::Opcode::BO_PtrMemI:
            return ParentWalker::AccessMethod::NONE;
        default:
            return ParentWalker::AccessMethod::READ;
    }
}

ParentWalker::AccessMethod ParentWalker::determineAccess(UnaryOperator::Opcode opcode){
    switch(opcode){
        case UnaryOperator::Opcode::UO_PostDec:
        case UnaryOperator::Opcode::UO_PostInc:
        case UnaryOperator::Opcode::UO_PreDec:
        case UnaryOperator::Opcode::UO_PreInc:
            //Add write.
            return ParentWalker::AccessMethod::WRITE;
        case UnaryOperator::Opcode::UO_Plus:
        case UnaryOperator::Opcode::UO_Minus:
            //Add read.
            return ParentWalker::AccessMethod::READ;
        default:
            return ParentWalker::AccessMethod::NONE;
    }
}

map<string, ParentWalker::AccessMethod> ParentWalker::buildAccessMap(ParentWalker::AccessMethod prevAccess,
                                                                     const Expr* curExpr){
    const Expr* prevExpr;

    //Rip out any unneeded items.
    do {
        prevExpr = curExpr;
        curExpr = curExpr->IgnoreParenCasts();
        curExpr = curExpr->IgnoreImpCasts();
        curExpr = curExpr->IgnoreImplicit();
        curExpr = curExpr->IgnoreCasts();
        curExpr = curExpr->IgnoreParens();
        curExpr = curExpr->IgnoreConversionOperator();
        curExpr = curExpr->IgnoreParenImpCasts();
        curExpr = curExpr->IgnoreParenLValueCasts();
    } while (prevExpr != curExpr);

    //Next, we determine the type of expression.
    if (isa<BinaryOperator>(curExpr)){
        return getAccessType(dyn_cast<BinaryOperator>(curExpr));
    } else if (isa<UnaryOperator>(curExpr))  {
        return getAccessType(dyn_cast<UnaryOperator>(curExpr));
    } else if (isa<DeclRefExpr>(curExpr)){
        map<string, ParentWalker::AccessMethod> singleMap;
        singleMap[generateID(dyn_cast<DeclRefExpr>(curExpr)->getDecl())] = prevAccess;
        return singleMap;
    } else if (isa<MemberExpr>(curExpr)){
        map<string, ParentWalker::AccessMethod> singleMap;
        singleMap[generateID(dyn_cast<MemberExpr>(curExpr)->getMemberDecl())] = prevAccess;
        return singleMap;
    }

    //Check if we're dealing with other, unimportant expressions.
    if (isa<IntegerLiteral>(curExpr) || isa<CharacterLiteral>(curExpr) || isa<FloatingLiteral>(curExpr)
        || isa<ImaginaryLiteral>(curExpr) || isa<UserDefinedLiteral>(curExpr) || isa<GNUNullExpr>(curExpr)){
        return map<string, ParentWalker::AccessMethod>();
    }

#ifdef EXPR_DEBUG
    //TODO: Experimental! Fix later.
    cerr << "Error: Expression cannot be detected!" << endl;
    cerr << "Expression on line " << Context->getSourceManager().getSpellingLineNumber(curExpr->getLocStart())
         << " in file " << Context->getSourceManager().getFilename(curExpr->getLocStart()).str() << endl;
    curExpr->dump();
#endif
    return map<string, ParentWalker::AccessMethod>();
}

bool ParentWalker::isInSystemHeader(const Stmt *statement) {
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

bool ParentWalker::isInSystemHeader(const Decl *decl){
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

void ParentWalker::recordParentSubscribe(const CXXConstructExpr* expr){
    //First, see if we have parents.
    const NamedDecl* parentVar = getParentAssign(expr);
    if (!parentVar) return;

    recordParentGeneric(generateID(parentVar), generateName(parentVar), RexNode::SUBSCRIBER);
}

void ParentWalker::recordParentPublish(const CXXConstructExpr* expr){
    //First, see if we have parents.
    const NamedDecl* parentVar = getParentAssign(expr);
    if (!parentVar) return;

    recordParentGeneric(generateID(parentVar), generateName(parentVar), RexNode::PUBLISHER);
}

void ParentWalker::recordParentGeneric(string parentID, string parentName, RexNode::NodeType type){
    //Next, we record the relationship between the two items.
    RexNode* src = graph->findNode(parentID);
    RexNode* dst;
    if (type == RexNode::PUBLISHER) {
        src->addSingleAttribute(ROS_PUB_VAR_FLAG, "1");
        dst = graph->generatePublisherNode(parentID, parentName);
    } else {
        src->addSingleAttribute(ROS_SUB_VAR_FLAG, "1");
        dst = graph->generateSubscriberNode(parentID, parentName);
    }

    //Creates the edge.
    RexEdge* edge = new RexEdge(src, dst, ((type == RexNode::PUBLISHER) ? RexEdge::PUBLISH : RexEdge::SUBSCRIBE));
    graph->addEdge(edge);

    //Keeps track of the node being published/subscribed.
    if (type == RexNode::PUBLISHER) {
        currentPublisher = dst;
    } else {
        currentSubscriber = dst;
    }

}

void ParentWalker::recordParentNodeHandle(const CXXConstructExpr* expr){
    //TODO
}

void ParentWalker::recordNodeHandle(const CXXConstructExpr* expr){
    //TODO
}


void ParentWalker::recordSubscribe(const CallExpr* expr){
    if (currentSubscriber == nullptr) return;

    //Now, adds information to the destination node.
    int numArgs = expr->getNumArgs();
    auto subscriberArgs = getArgs(expr);

    //Get the name of the topic and record it if not present.
    string topicName = validateStringArg(subscriberArgs[0]);
    recordTopic(topicName);
    RexNode* topic = graph->findNode(TOPIC_PREFIX + topicName);
    RexEdge* topEdge = new RexEdge(topic, currentSubscriber, RexEdge::SUBSCRIBE);
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

void ParentWalker::recordPublish(const CallExpr* expr){
    //Get the publisher object.
    auto parent = getParentVariable(expr);
    if (!parent) return;
    RexNode* parentVar = graph->findNodeByName(generateID(parent));
    if (!parentVar) return;

    //Next, gets the topic it publishes to.
    vector<RexEdge*> destinations = graph->findEdgesBySrc(parentVar->getID(), false);
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

void ParentWalker::recordAdvertise(const CallExpr* expr) {
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

void ParentWalker::recordTopic(string name){
    string ID = TOPIC_PREFIX + name;
    if (graph->doesNodeExist(ID)) return;

    //Create the node.
    RexNode* node = new RexNode(ID, name, RexNode::TOPIC);
    graph->addNode(node);
}

string ParentWalker::generateID(const NamedDecl* decl){
    //Gets the canonical decl.
    decl = dyn_cast<NamedDecl>(decl->getCanonicalDecl());
    string name = "";

    //Generates a special name for function overloading.
    if (isa<FunctionDecl>(decl) || isa<CXXMethodDecl>(decl)){
        const FunctionDecl* cur = decl->getAsFunction();
        name = cur->getReturnType().getAsString() + "-" + decl->getNameAsString();
        for (int i = 0; i < cur->getNumParams(); i++){
            name += "-" + cur->parameters().data()[i]->getType().getAsString();
        }
    } else {
        name = decl->getNameAsString();
    }


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
        if (kind != Decl::Function && kind == Decl::CXXMethod){
            //We need to get the parent function.
            const DeclContext *parentContext = originalDecl->getParentFunctionOrMethod();

            //If we have nullptr, get the parent function.
            if (parentContext != nullptr) {
                string parentQualName = generateID(static_cast<const FunctionDecl *>(parentContext));
                name = parentQualName + "::" + originalDecl->getNameAsString();
            }
        }
    }

    //Finally, check if we have a main method.
    if (name.compare("main") == 0){
        name = Context->getSourceManager().getFilename(originalDecl->getLocStart()).str() + "--" + name;
    }

    return name;
}

string ParentWalker::generateName(const NamedDecl* decl){
    string name = decl->getQualifiedNameAsString();
    //Finally, check if we have a main method.
    if (name.compare("main") == 0){
        name = Context->getSourceManager().getFilename(decl->getLocStart()).str() + "\'s " + name;
    }

    return name;
}

string ParentWalker::validateStringArg(string name){
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

int ParentWalker::generateTAModel(TAGraph* graph, string fileName){
    //Purge the edges.
    graph->purgeUnestablishedEdges(true);
    string correctnessMsg = graph->checkCorrectness();

    //Gets the string for the model.
    string model = graph->getTAModel();

    //Checks the correctness.
    if (correctnessMsg.compare("") != 0){
        cerr << "Warning: TA model has some inconsistencies. See the TA file for error information." << endl;
        model = correctnessMsg + model;
    }

    //Creates the file stream.
    ofstream file(fileName);
    if (!file.is_open()) return 0;

    //Writes to the file.
    file << model;

    file.close();
    return 1;
}

bool ParentWalker::isInSystemHeader(const SourceManager& manager, SourceLocation loc) {
    //Get the expansion location.
    auto expansionLoc = manager.getExpansionLoc(loc);

    //Check if we have a valid location.
    if (expansionLoc.isInvalid()) {
        return false;
    }

    //Get if we have a system header.
    bool sysHeader = manager.isInSystemHeader(loc);
    if (sysHeader) return sysHeader;

    //Now, check to see if we have a ROS library.
    string libLoc = expansionLoc.printToString(manager);
    libLoc = libLoc.substr(0, libLoc.find(":"));
    if (boost::algorithm::ends_with(libLoc, ".h") || boost::algorithm::ends_with(libLoc, ".hpp")){
        boost::filesystem::path loc = boost::filesystem::canonical(boost::filesystem::path(libLoc));
        for (string curLibrary : ignoreLibraries){
            if (loc.string().find(curLibrary) != string::npos) return true;
        }
    }

    return false;
}