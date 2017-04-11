#include "ROSWalker.h"

using namespace std;

ROSWalker::ROSWalker(ASTContext *Context) : Context(Context) {}

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

    outs() << "Function: " << decl->getQualifiedNameAsString() << "\n";
    return true;
}

bool ROSWalker::VisitCXXRecordDecl(CXXRecordDecl *decl) {
    if (isInSystemHeader(decl)) return true;

    outs() << "Class: " << decl->getQualifiedNameAsString() << "\n";
    return true;
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

ROSConsumer::ROSConsumer(ASTContext *Context) : Visitor(Context) {}

void ROSConsumer::HandleTranslationUnit(ASTContext &Context) {
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());
}

std::unique_ptr<ASTConsumer> ROSAction::CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile) {
    return std::unique_ptr<ASTConsumer>(new ROSConsumer(&Compiler.getASTContext()));
}