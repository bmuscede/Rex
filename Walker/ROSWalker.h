//
// Created by bmuscede on 07/04/17.
//

#ifndef REX_ROSWALKER_H
#define REX_ROSWALKER_H

#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include "llvm/Support/CommandLine.h"
#include "../Graph/TAGraph.h"
#include "ParentWalker.h"

using namespace llvm;
using namespace clang;
using namespace clang::tooling;

class ROSWalker : public RecursiveASTVisitor<ROSWalker>, public ParentWalker {
public:
    explicit ROSWalker(ASTContext *Context);
    ~ROSWalker();

    //ASTWalker Functions
    bool VisitStmt(Stmt* statement);
    bool VisitFunctionDecl(FunctionDecl* decl);
    bool VisitCXXRecordDecl(CXXRecordDecl* decl);
    bool VisitVarDecl(VarDecl* decl);
    bool VisitFieldDecl(FieldDecl* decl);

private:
    //C++ Detectors
    void recordFunctionDecl(const FunctionDecl* decl);
    void recordClassDecl(const CXXRecordDecl* decl);
    void recordVarDecl(const VarDecl* decl);
    void recordFieldDecl(const FieldDecl* decl);

    //Expr Recorders
    void recordCallExpr(const CallExpr* expr);
    void recordVarUsage(const DeclRefExpr* expr);

    //Secondary Helper Functions
    void addParentRelationship(const NamedDecl* baseDecl, std::string baseID);
    const FunctionDecl* getParentFunction(const Expr* callExpr);
};

#endif //REX_ROSWALKER_H
