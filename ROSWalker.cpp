#include <fstream>
#include "ROSWalker.h"
#include "RexNode.h"

using namespace std;

TAGraph* ROSWalker::graph = new TAGraph();

ROSWalker::ROSWalker(ASTContext *Context) : Context(Context) { }

ROSWalker::~ROSWalker(){
    delete graph;
}

bool ROSWalker::VisitStmt(Stmt *statement) {
    if (isInSystemHeader(statement)) return true;

    //Handle ROS statements.
    if (CallExpr* expr = dyn_cast<CallExpr>(statement)){
        recordPublish(expr);

        //Deal with the expression.
        if (isPublish(expr)) recordPublish(expr);
        else if (isSubscribe(expr)) recordSubscribe(expr);
    }

    return true;
}

bool ROSWalker::VisitFunctionDecl(FunctionDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Record the function declaration.
    recordFunctionDecl(decl);
    return true;
}

void ROSWalker::flushTAGraph(){
    delete graph;
    graph = new TAGraph();
}

int ROSWalker::generateTAModel(string fileName){
    //Purge the edges.
    graph->purgeUnestablishedEdges();

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

void ROSWalker::recordPublish(const CallExpr *expr) {
    //Gets certain arguments about the publisher.
    int numArgs = expr->getNumArgs();

    //Check if the number of arguments is valid.
    if (numArgs != 1) return;

    //Get the argument(s).
    vector<string> publisherArgs = getArgs(expr);
}

void ROSWalker::recordSubscribe(const CallExpr *expr) {
    //Gets certain arguments about the subscriber.
    int numArgs = expr->getNumArgs();
    vector<string> subscriberArgs = getArgs(expr);


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

string ROSWalker::generateID(const FunctionDecl* decl){
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