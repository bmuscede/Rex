#include <fstream>
#include <iostream>
#include "clang/AST/Mangle.h"
#include "ROSWalker.h"
#include "RexNode.h"

using namespace std;

TAGraph* ROSWalker::graph = new TAGraph();

ROSWalker::ROSWalker(ASTContext *Context) : Context(Context) { }

ROSWalker::~ROSWalker(){ }

bool ROSWalker::VisitStmt(Stmt *statement) {
    if (isInSystemHeader(statement)) return true;

    //Handle ROS Publisher and Subscriber object creations.
    if (CXXConstructExpr* cxxExpr = dyn_cast<CXXConstructExpr>(statement)){
        cout << cxxExpr->getBestDynamicClassType()->getQualifiedNameAsString() << endl;
        if (isNodeHandlerObj(cxxExpr)){
            //TODO
        } else if (isSubscriberObj(cxxExpr)){
            //TODO
        } else if (isPublisherObj(cxxExpr)){
            //TODO
        }
    }

    //Handle ROS statements.
    if (CallExpr* callExpr = dyn_cast<CallExpr>(statement)){
        //Deal with the expression.
        if (isPublish(callExpr)) recordPublish(callExpr);
        else if (isSubscribe(callExpr)) recordSubscribe(callExpr);

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

void ROSWalker::deleteTAGraph(){
    delete graph;
}

void ROSWalker::flushTAGraph(){
    ROSWalker::deleteTAGraph();
    graph = new TAGraph();
}

int ROSWalker::generateTAModel(string fileName){
    //Purge the edges.
    graph->purgeUnestablishedEdges(true);

    //Gets the string for the model.
    string model = graph->getTAModel();

    //Creates the file stream.
    ofstream file(fileName);
    if (!file.is_open()) return 1;

    //Writes to the file.
    file << model;

    file.close();
    return 0;
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
    auto record = ctor->getBestDynamicClassType();
    if (record == nullptr) return false;

    //Check the qualified name.
    if (record->getQualifiedNameAsString().compare(className)) return false;
    return true;
}

//TODO: Actually implement.
void ROSWalker::recordPublish(const CallExpr *expr) {
    //Gets certain arguments about the publisher.
    int numArgs = expr->getNumArgs();

    //Check if the number of arguments is valid.
    if (numArgs != 1) return;

    //Get the argument(s).
    string publisherVar = getArgs(expr)[0];

}

//TODO: Actually implement.
void ROSWalker::recordSubscribe(const CallExpr *expr) {
    //Gets certain arguments about the subscriber.
    int numArgs = expr->getNumArgs();
    vector<string> subscriberArgs = getArgs(expr);

    //Creates the
    //
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

ROSConsumer::ROSConsumer(ASTContext *Context) : Visitor(Context) {}

void ROSConsumer::HandleTranslationUnit(ASTContext &Context) {
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());
}

std::unique_ptr<ASTConsumer> ROSAction::CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile) {
    return std::unique_ptr<ASTConsumer>(new ROSConsumer(&Compiler.getASTContext()));
}