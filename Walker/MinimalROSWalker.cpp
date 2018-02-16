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

#include <iostream>
#include "MinimalROSWalker.h"

using namespace std;

/**
 * Constructor for the minimal ROS walker.
 * @param Context The AST context.
 */
MinimalROSWalker::MinimalROSWalker(ASTContext *Context) : ParentWalker(Context) { }

/**
 * Destructor.
 */
MinimalROSWalker::~MinimalROSWalker(){ }

/**
 * Visits a C++ statement.
 * @param statement The statment to visit.
 * @return Whether to continue processing.
 */
bool MinimalROSWalker::VisitStmt(Stmt *statement) {
    if (isInSystemHeader(statement)) return true;

    //Now, we just handle the statement for minimal ROS handler.
    handleMinimalStmt(statement);

    return true;
}

/**
 * Visits a variable declaration.
 * @param statement The declaration to visit.
 * @return Whether to continue processing.
 */
bool MinimalROSWalker::VisitVarDecl(VarDecl* decl) {
    if (isInSystemHeader(decl)) return true;

    //Now, we just handle the varDecl for minimal ROS handler.
    handleMinimalVarDecl(decl);

    //Next, check for parent class information.
    recordParentClassLoc(decl->getAsFunction());

    return true;
}

/**
 * Visits a field declaration.
 * @param statement The declaration to visit.
 * @return Whether to continue processing.
 */
bool MinimalROSWalker::VisitFieldDecl(FieldDecl* decl) {
    if (isInSystemHeader(decl)) return true;

    //Now, we just handle the fieldDecl for minimal ROS handler.
    handleMinimalFieldDecl(decl);

    //Next, check for parent class information.
    recordParentClassLoc(decl->getAsFunction());

    return true;
}