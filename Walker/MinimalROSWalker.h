//
// Created by bmuscede on 06/07/17.
//

#ifndef REX_MINIMALROSWALKER_H
#define REX_MINIMALROSWALKER_H

#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;
using namespace clang;
using namespace clang::tooling;

class MinimalROSWalker : public RecursiveASTVisitor<MinimalROSWalker>{
public:
    explicit MinimalROSWalker(ASTContext *Context);
    ~MinimalROSWalker();

private:
    ASTContext *Context;
};

class MinimalROSConsumer : public ASTConsumer {
public:
    explicit MinimalROSConsumer(ASTContext *Context);
    virtual void HandleTranslationUnit(ASTContext &Context);

private:
    MinimalROSWalker Visitor;
};

class MinimalROSAction : public ASTFrontendAction {
public:
    virtual std::unique_ptr<ASTConsumer> CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile);
};

#endif //REX_MINIMALROSWALKER_H
