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
    bool VisitVarDecl(VarDecl* decl);
    bool VisitFieldDecl(FieldDecl* decl);
    bool VisitFunctionDecl(FunctionDecl* decl);
    bool VisitCXXRecordDecl(CXXRecordDecl* decl);
    bool VisitDeclRefExpr(DeclRefExpr* declRef);

private:
    const std::string CONTROL_FLAG = "isControlFlow";
    const std::string ROS_CONTROL_FLAG = "isUnderControl";
    const std::string PARAM_FLAG = "isParam";
    
    //C++ Detectors
    void recordFunctionDecl(const FunctionDecl* decl);
    void recordClassDecl(const CXXRecordDecl* decl);
    void recordVarDecl(const VarDecl* decl);
    void recordFieldDecl(const FieldDecl* decl);

    //ROS Managers
    void handleFullPub(const Stmt* statement);

    //Expr Recorders
    void recordCallExpr(const CallExpr* expr);
    void recordVarUsage(const FunctionDecl* decl, std::map<std::string, ParentWalker::AccessMethod> accesses);
    void recordControlFlow(const DeclRefExpr* expr);
    RexEdge* recordParentFunction(const Stmt* statement, RexNode* baseItem);
    void recordROSControl(const Stmt* baseStmt, RexNode* rosItem);

    //Var Recorders
    std::vector<const NamedDecl*> getVars(const IfStmt* stmt);
    std::vector<const NamedDecl*> getVars(const ForStmt* stmt);
    std::vector<const NamedDecl*> getVars(const WhileStmt* stmt);
    std::vector<const NamedDecl*> getVars(const DoStmt* stmt);
    std::vector<const NamedDecl*> getVars(const SwitchStmt* stmt);
    std::vector<const NamedDecl*> getVars(const Stmt* stmt);

    //Callback Recorder
    void checkForCallbacks(const FunctionDecl* decl);

    //Control Flow Detectors
    bool usedInIfStatement(const DeclRefExpr* declRef);
    bool usedInLoop(const DeclRefExpr* declRef);

    //Secondary Helper Functions
    void addParentRelationship(const NamedDecl* baseDecl, std::string baseID);
    const FunctionDecl* getParentFunction(const Stmt* expr);
};

#endif //REX_ROSWALKER_H
