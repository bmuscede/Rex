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

    //Next, check for parent class information.
    recordParentClassLoc(decl->getAsFunction());

    return true;
}

bool MinimalROSWalker::VisitFieldDecl(FieldDecl* decl) {
    if (isInSystemHeader(decl)) return true;

    //Now, we just handle the fieldDecl for minimal ROS handler.
    handleMinimalFieldDecl(decl);

    //Next, check for parent class information.
    recordParentClassLoc(decl->getAsFunction());

    return true;
}