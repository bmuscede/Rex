////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROSWalker.h
//
// Created By: Bryan J Muscedere
// Date: 07/04/17.
//
// Walks through Clang's AST using the full analysis
// mode methodology. This is achieved using the parent
// walker class to help obtain information about each
// AST node.
//
// Copyright (C) 2017, Bryan J. Muscedere
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
/////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    //Constructor/Destructor
    explicit ROSWalker(ASTContext *Context);
    ~ROSWalker();

    //ASTWalker Functions
    bool VisitStmt(Stmt* statement);
    bool VisitVarDecl(VarDecl* decl);
    bool VisitFieldDecl(FieldDecl* decl);
    bool VisitFunctionDecl(FunctionDecl* decl);
    bool VisitCXXRecordDecl(CXXRecordDecl* decl);
    bool VisitMemberExpr(MemberExpr* memExpr);
    bool VisitDeclRefExpr(DeclRefExpr* declRef);

private:
    const std::string CONTROL_FLAG = "isControlFlow";
    const std::string PARAM_FLAG = "isParam";

    std::vector<clang::Expr*> parentExpression;

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
    void recordControlFlow(const MemberExpr* expr);
    void recordControlFlow(const ValueDecl* decl);
    RexEdge* recordParentFunction(const Stmt* statement, RexNode* baseItem);
    void recordASTControl(const Stmt* baseStmt, RexNode* rosItem, std::string rosID, RexNode::NodeType type);

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
    bool usedInIfStatement(const Expr* declRef);
    bool usedInLoop(const Expr* declRef);
    bool findExpression(const Expr* expression);

    //Secondary Helper Functions
    void addParentRelationship(const NamedDecl* baseDecl, std::string baseID);
    const FunctionDecl* getParentFunction(const Stmt* expr);
};

#endif //REX_ROSWALKER_H
