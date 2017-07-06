//
// Created by bmuscede on 06/07/17.
//

#include "MinimalROSWalker.h"

MinimalROSWalker::MinimalROSWalker(ASTContext *Context) : Context(Context) { }

MinimalROSWalker::~MinimalROSWalker(){ }


MinimalROSConsumer::MinimalROSConsumer(ASTContext *Context) : Visitor(Context) {}

void MinimalROSConsumer::HandleTranslationUnit(ASTContext &Context) {
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());
}

std::unique_ptr<ASTConsumer> MinimalROSAction::CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile) {
    return std::unique_ptr<ASTConsumer>(new MinimalROSConsumer(&Compiler.getASTContext()));
}