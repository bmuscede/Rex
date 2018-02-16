/////////////////////////////////////////////////////////////////////////////////////////////////////////
// MinimalROSWalker.h
//
// Created By: Bryan J Muscedere
// Date: 06/07/17.
//
// Walks through the code being processed by Clang to add
// minimal ROS elements to the resultant TA file. This adds
// information about ROS components and classes.
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
    //Constructor/Destructor
    explicit MinimalROSWalker(ASTContext *Context);
    ~MinimalROSWalker();

    //ASTWalker Functions
    bool VisitStmt(Stmt *statement);
    bool VisitVarDecl(VarDecl* decl);
    bool VisitFieldDecl(FieldDecl* decl);

private:
    //ROS Managers
    void recordParentSubscribe(const CXXConstructExpr* expr, std::string fileName);
    void recordParentPublish(const CXXConstructExpr* expr, std::string fileName);
};

#endif //REX_MINIMALROSWALKER_H
