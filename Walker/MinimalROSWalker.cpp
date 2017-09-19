//
// Created by bmuscede on 06/07/17.
//

#include <iostream>
#include "MinimalROSWalker.h"

using namespace std;

MinimalROSWalker::MinimalROSWalker(ASTContext *Context) : ParentWalker(Context) { }

MinimalROSWalker::~MinimalROSWalker(){ }

bool MinimalROSWalker::VisitStmt(Stmt *statement) {
    if (isInSystemHeader(statement)) return true;

    //Now, we just handle the statement for minimal ROS handler.
    handleMinimalStmt(statement);

    return true;
}

bool MinimalROSWalker::VisitVarDecl(VarDecl* decl) {
    if (isInSystemHeader(decl)) return true;

    //Now, we just handle the varDecl for minimal ROS handler.
    handleMinimalVarDecl(decl);

    return true;
}

bool MinimalROSWalker::VisitFieldDecl(FieldDecl* decl) {
    if (isInSystemHeader(decl)) return true;

    //Now, we just handle the fieldDecl for minimal ROS handler.
    handleMinimalFieldDecl(decl);

    return true;
}

void MinimalROSWalker::recordParentSubscribe(const CXXConstructExpr* expr, string className){
    if (className.compare("") == 0) return;
    recordParentGeneric(className, className, RexNode::SUBSCRIBER);
}

void MinimalROSWalker::recordParentPublish(const CXXConstructExpr* expr, string fileName) {
    if (fileName.compare("") == 0) return;
    recordParentGeneric(fileName, fileName, RexNode::PUBLISHER);
}