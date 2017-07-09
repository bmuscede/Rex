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
#include <string>
#include "ParentWalker.h"

using namespace llvm;
using namespace clang;
using namespace clang::tooling;

class MinimalROSWalker : public RecursiveASTVisitor<MinimalROSWalker>, public ParentWalker {
public:
    explicit MinimalROSWalker(ASTContext *Context);
    ~MinimalROSWalker();

    bool VisitStmt(Stmt *statement);
    bool VisitFunctionDecl(FunctionDecl* decl);

private:
    void recordFileLoc(SourceLocation loc);

    std::string getFileName(SourceLocation loc);
};

#endif //REX_MINIMALROSWALKER_H
