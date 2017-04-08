#include "ROSWalker.h"

ROSWalker::ROSWalker(ASTContext *Context) : Context(Context) {}

bool ROSWalker::VisitFunctionDecl(FunctionDecl* decl){
    outs() << decl->getQualifiedNameAsString() << "\n";
    return true;
}

bool ROSWalker::VisitCXXRecordDecl(CXXRecordDecl *decl) {
    outs() << decl->getQualifiedNameAsString() << "\n";
    return true;
}

ROSConsumer::ROSConsumer(ASTContext *Context) : Visitor(Context) {}

void ROSConsumer::HandleTranslationUnit(ASTContext &Context) {
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());
}

std::unique_ptr<ASTConsumer> ROSAction::CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile) {
    return std::unique_ptr<ASTConsumer>(new ROSConsumer(&Compiler.getASTContext()));
}