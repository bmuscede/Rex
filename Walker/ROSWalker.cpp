////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROSWalker.cpp
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

#include <fstream>
#include <iostream>
#include "clang/AST/Mangle.h"
#include "ROSWalker.h"
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include "../Graph/RexNode.h"

using namespace std;

/**
 * Constructor
 * @param Context AST Context
 */
ROSWalker::ROSWalker(ASTContext *Context) : ParentWalker(Context) { }

/**
 * Destructor
 */
ROSWalker::~ROSWalker(){ }

/**
 * Visits statements for ROS components.
 * @param statement The statement to visit.
 * @return Whether we should continue.
 */
bool ROSWalker::VisitStmt(Stmt *statement) {
    if (isInSystemHeader(statement)) return true;

    //First, handle ROS messages.
    ROSType type = handleMinimalStmt(statement);

    //Check if we have a publisher.
    if (type == ROSType::PUB) handleFullPub(statement);

    //Deal with call expressions.
    if (CallExpr* callExpr = dyn_cast<CallExpr>(statement)){
        //Deal with function calls.
        recordCallExpr(callExpr);
    }

    //Deal with uses of control flow.
    if (IfStmt* ifStmt = dyn_cast<IfStmt>(statement)){
        parentExpression.push_back(ifStmt->getCond());
    } else if (SwitchStmt* switchStmt = dyn_cast<SwitchStmt>(statement)) {
        parentExpression.push_back(switchStmt->getCond());
    } else if (ForStmt* forStmt = dyn_cast<ForStmt>(statement)){
        parentExpression.push_back(forStmt->getCond());
    } else if (WhileStmt* whileStmt = dyn_cast<WhileStmt>(statement)){
        parentExpression.push_back(whileStmt->getCond());
    }

    //Deal with variable usages.
    if (BinaryOperator* binOp = dyn_cast<BinaryOperator>(statement)){
        //Get all variable usages.
        auto accesses = getAccessType(binOp);
        recordVarUsage(getParentFunction(binOp), accesses);
    } else if (UnaryOperator* unaryOp = dyn_cast<UnaryOperator>(statement)){
        auto accesses = getAccessType(unaryOp);
        recordVarUsage(getParentFunction(unaryOp), accesses);
    } else if (DeclStmt* declOp = dyn_cast<DeclStmt>(statement)){
        auto accesses = getAccessType(declOp);
        recordVarUsage(getParentFunction(declOp), accesses);
    }

    return true;
}

/**
 * Visits variable declarations.
 * @param decl The variable declaration to visit.
 * @return Whether we should continue.
 */
bool ROSWalker::VisitVarDecl(VarDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Handle ROS messages.
    bool rosBased = handleMinimalVarDecl(decl);
    if (rosBased) return true;

    //Record the variable declaration.
    recordVarDecl(decl);

    //Next, check for parent class information.
    recordParentClassLoc(decl->getAsFunction());

    return true;
}

/**
 * Visits field declarations.
 * @param decl The variable declaration to visit.
 * @return Whether we should continue.
 */
bool ROSWalker::VisitFieldDecl(FieldDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Handle ROS messages.
    bool rosBased = handleMinimalFieldDecl(decl);
    if (rosBased) return true;

    //Record the function declaration.
    recordFieldDecl(decl);

    //Next, check for parent class information.
    recordParentClassLoc(decl->getAsFunction());

    return true;
}

/**
 * Visits function declarations.
 * @param decl The function declaration to visit.
 * @return Whether we should continue.
 */
bool ROSWalker::VisitFunctionDecl(FunctionDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Record the function declaration.
    recordFunctionDecl(decl);
    checkForCallbacks(decl);

    //Next, check for parent class information.
    recordParentClassLoc(decl);

    return true;
}

/**
 * Visits class declarations.
 * @param decl The class declaration to visit.
 * @return Whether we should continue.
 */
bool ROSWalker::VisitCXXRecordDecl(CXXRecordDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Check what type of structure it is.
    if (decl->isClass()){
        recordClassDecl(decl);
    }

    return true;
}

/**
 * Visits member expressions.
 * @param decl The member expression to visit.
 * @return Whether we should continue.
 */
bool ROSWalker::VisitMemberExpr(MemberExpr *memExpr) {
    if (isInSystemHeader(memExpr)) return true;

    //Check if we have an if-statement noted here.
    if (usedInIfStatement(memExpr) || usedInLoop(memExpr)){
        recordControlFlow(memExpr);
    }

    return true;
}

/**
 * Visits declaration reference expressions.
 * @param decl The declaration reference to visit.
 * @return Whether we should continue.
 */
bool ROSWalker::VisitDeclRefExpr(DeclRefExpr *declRef) {
    if (isInSystemHeader(declRef)) return true;

    //Check if we have an if-statement noted here.
    if (usedInIfStatement(declRef) || usedInLoop(declRef)){
        recordControlFlow(declRef);
    }

    return true;
}

/**
 * Records a basic function declaration to the TA model.
 * @param decl The declaration to add.
 */
void ROSWalker::recordFunctionDecl(const FunctionDecl* decl){
    //Generates the fields.
    string ID = generateID(decl);
    string name = generateName(decl);

    //Creates the node.
    RexNode* node = new RexNode(ID, name, RexNode::FUNCTION);
    node->addSingleAttribute(CALLBACK_FLAG, "0");
    graph->addNode(node);

    //Get the parent.
    addParentRelationship(decl, ID);
}

/**
 * Records a basic class declaration to the TA model.
 * @param decl The declaration to add.
 */
void ROSWalker::recordClassDecl(const CXXRecordDecl *decl) {
    //Generates some fields.
    string ID = generateID(decl);
    string name = generateName(decl);

    //Creates the node.
    if (!graph->doesNodeExist(ID)) {
        RexNode *node = new RexNode(ID, name, RexNode::CLASS);
        graph->addNode(node);
    }

    //Resolves the filename.
    string filename = generateFileName(decl);
    RexNode* node = graph->findNode(ID);
    node->addMultiAttribute(FILENAME_ATTR, filename);

    //Get the parent.
    addParentRelationship(decl, ID);
}

/**
 * Records a basic variable declaration to the TA model.
 * @param decl The declaration to add.
 */
void ROSWalker::recordVarDecl(const VarDecl* decl){
    //Generates some fields.
    string ID = generateID(decl);
    string name = generateName(decl);

    //Creates the node.
    RexNode* node = new RexNode(ID, name, RexNode::VARIABLE);
    node->addSingleAttribute(CONTROL_FLAG, "0");
    if (isa<ParmVarDecl>(decl)) node->addSingleAttribute(PARAM_FLAG, "1");
    else node->addSingleAttribute(PARAM_FLAG, "0");
    graph->addNode(node);

    //Get the parent.
    addParentRelationship(decl, ID);
}

/**
 * Records a basic field declaration to the TA model.
 * @param decl The declaration to add.
 */
void ROSWalker::recordFieldDecl(const FieldDecl* decl){
    //Generates some fields.
    string ID = generateID(decl);
    string name = generateName(decl);

    //Creates the node.
    RexNode* node = new RexNode(ID, name, RexNode::VARIABLE);
    node->addSingleAttribute(CONTROL_FLAG, "0");
    graph->addNode(node);

    //Get the parent.
    addParentRelationship(decl, ID);
}

/**
 * Handles any extra publisher operations after the basic ones were handled by
 * the parent walker.
 * @param statement The statement with the publisher call.
 */
void ROSWalker::handleFullPub(const Stmt* statement){
    //Gets the publisher.
    if (!currentPublisherOutdated) return;

    //Parent Function Relation Adder.
    RexEdge* callEdge = recordParentFunction(statement, currentPublisherOutdated);

    //Control Structure Adder.
    recordASTControl(statement, currentPublisherOutdated,
                     currentPublisherOutdated->getID(), currentPublisherOutdated->getType());
}

/**
 * Records a basic call expression.
 * @param expr The expression to record.
 */
void ROSWalker::recordCallExpr(const CallExpr* expr){
    //Get the sub-function.
    auto callee = expr->getCalleeDecl();
    if (callee == nullptr) return;
    auto cDecl = dyn_cast<FunctionDecl>(callee);
    if (cDecl == nullptr) return;

    //Gets the ID for the cDecl.
    string calleeID = generateID(cDecl);
    RexNode* calleeNode = graph->findNode(calleeID);

    //Gets the parent expression.
    auto parDecl = getParentFunction(expr);
    if (parDecl == nullptr) return;

    //Gets the ID for the parent.
    string callerID = generateID(parDecl);
    RexNode* callerNode = graph->findNode(callerID);

    //Adds the edge.
    if (graph->doesEdgeExist(callerID, calleeID, RexEdge::CALLS)) return;
    RexEdge* edge = (calleeNode == nullptr) ?
                    new RexEdge(callerNode, calleeID, RexEdge::CALLS) :
                    new RexEdge(callerNode, calleeNode, RexEdge::CALLS);
    graph->addEdge(edge);

    //Last, adds a relation if its a control structure.
    recordASTControl(expr, calleeNode, calleeID, RexNode::FUNCTION);
}

/**
 * Checks a function declaration for whether its a callback function.
 * @param decl The declaration to inspect.
 */
void ROSWalker::checkForCallbacks(const FunctionDecl* decl){
    if (decl == nullptr) return;

    //Gets the node.
    RexNode* node = graph->findNode(generateID(decl));

    string qualName = decl->getQualifiedNameAsString();
    vector<RexEdge*> edges = graph->findEdgesByDst(qualName, true);

    //Go through each.
    for (RexEdge* cur : edges){
        cur->setDestination(node);
        cur->setDestinationID(node->getID());
    }
}

/**
 * Check if a declaration reference expression is used in an if statement.
 * @param declRef The declaration reference expression.
 * @return Whether the variable is used.
 */
bool ROSWalker::usedInIfStatement(const Expr* declRef){
    //Iterate through and find if we have an if statement parent.
    bool getParent = true;
    auto parent = Context->getParents(*declRef);
    auto previous = Context->getParents(*declRef);

    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current parent as an if statement.
        auto ifStmt = parent[0].get<clang::IfStmt>();
        if (ifStmt) {
            auto conditionExpression = previous[0].get<clang::Expr>();
            if (findExpression(conditionExpression)) return true;
            return false;
        }

        auto switchStmt = parent[0].get<clang::CaseStmt>();
        if (switchStmt){
            auto conditionExpression = previous[0].get<clang::Expr>();
            if (findExpression(conditionExpression)) return true;
            return false;
        }

        previous = parent;
        parent = Context->getParents(parent[0]);
    }

    return false;
}

/**
 * Check if a declaration reference expression is used in a loop.
 * @param declRef The declaration reference expression.
 * @return Whether the variable is used.
 */
bool ROSWalker::usedInLoop(const Expr* declRef){
    //Iterate through and find if we have an if statement parent.
    bool getParent = true;
    auto parent = Context->getParents(*declRef);
    auto previous = Context->getParents(*declRef);

    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current parent as an if statement.
        auto whileStmt = parent[0].get<clang::WhileStmt>();
        if (whileStmt) {
            auto conditionExpression = previous[0].get<clang::Expr>();
            if (findExpression(conditionExpression)) return true;
            return false;
        }

        auto forStmt = parent[0].get<clang::ForStmt>();
        if (forStmt){
            auto conditionExpression = previous[0].get<clang::Expr>();
            if (findExpression(conditionExpression)) return true;
            return false;
        }

        previous = parent;
        parent = Context->getParents(parent[0]);
    }

    return false;
}

/**
 * Finds an expression based on stored expressions.
 * @param expression The expression to find.
 * @return Whether the expression was found.
 */
bool ROSWalker::findExpression(const Expr* expression){
    for (Expr* cur : parentExpression){
        if (cur == expression) return true;
    }

    return false;
}

/**
 * Records any variable usages.
 * @param decl The declaration to add.
 * @param accesses The access map.
 */
void ROSWalker::recordVarUsage(const FunctionDecl* decl, map<string, ParentWalker::AccessMethod> accesses){
    if (decl == nullptr) return;

    //Iterate through the map.
    map<string, ParentWalker::AccessMethod>::iterator it;
    for (it = accesses.begin(); it != accesses.end(); it++){
        //Gets the ID for the function.
        string functionID = generateID(decl);
        RexNode* functionNode = graph->findNode(functionID);

        //Gets the ID for the variable.
        string varID = it->first;
        RexNode* varNode = graph->findNode(varID);

        //Adds the edge.
        switch(it->second){
            case ParentWalker::AccessMethod::BOTH:
            case ParentWalker::AccessMethod::WRITE:
                if (!graph->doesEdgeExist(functionID, varID, RexEdge::WRITES)){
                    RexEdge* edge = (varNode == nullptr) ?
                                    new RexEdge(functionNode, varID, RexEdge::WRITES) :
                                    new RexEdge(functionNode, varNode, RexEdge::WRITES);
                    graph->addEdge(edge);
                }

                if (it->second == ParentWalker::AccessMethod::WRITE) break;
            case ParentWalker::AccessMethod::READ:
                if (!graph->doesEdgeExist(varID, functionID, RexEdge::READS)){
                    RexEdge* edge = (varNode == nullptr) ?
                                    new RexEdge(varID, functionNode, RexEdge::READS) :
                                    new RexEdge(varNode, functionNode, RexEdge::READS);
                    graph->addEdge(edge);
                }

                break;
            case ParentWalker::AccessMethod::NONE:
                continue;
        }
    }
}

/**
 * Records whether a variable is used in a control flow statement.
 * @param expr The expression to analyze.
 */
void ROSWalker::recordControlFlow(const DeclRefExpr* expr){
    //Get the decl responsible.
    const ValueDecl* decl = expr->getDecl();
    recordControlFlow(decl);
}

/**
 * Records whether a variable is used in a control flow statement.
 * @param expr The expression to analyze.
 */
void ROSWalker::recordControlFlow(const MemberExpr* expr){
    const ValueDecl* decl = expr->getMemberDecl();
    recordControlFlow(decl);
}

/**
 * Records whether a variable is used in a control flow statement.
 * @param decl The value declaration to analyze.
 */
void ROSWalker::recordControlFlow(const ValueDecl* decl){
    //Get the ID and find it.
    string declID = generateID(decl);
    RexNode* refNode = graph->findNode(declID);
    if (!refNode) return;

    //Adds 1 to the control flag attr.
    refNode->addSingleAttribute(CONTROL_FLAG, "1");
}

/**
 * Records the parent function in a statement.
 * @param statement The statement.
 * @param baseItem The baseItem of the parent.
 * @return The generated Rex edge.
 */
RexEdge* ROSWalker::recordParentFunction(const Stmt* statement, RexNode* baseItem){
    //Gets the parent function.
    const FunctionDecl* decl = getParentFunction(statement);
    RexNode* funcNode = graph->findNode(generateID(decl));

    //Checks if an edge already exists.
    if (graph->doesEdgeExist(generateID(decl), baseItem->getID(), RexEdge::CALLS))
        return graph->findEdge(generateID(decl), baseItem->getID(), RexEdge::CALLS);

    //Adds a call relation between the two.
    RexEdge* edge;
    if (funcNode) {
        edge = new RexEdge(funcNode, baseItem, RexEdge::CALLS);
    } else {
        edge = new RexEdge(generateID(decl), baseItem, RexEdge::CALLS);
    }
    graph->addEdge(edge);
    return edge;
}

/**
 * Records whether a statement is part of a control structure.
 * @param baseStmt The base statement to process.
 * @param astItem The node to add as parent.
 * @param astID The ID of the node being added as parent.
 * @param astType The node type beind added.
 */
void ROSWalker::recordASTControl(const Stmt* baseStmt, RexNode* astItem, string astID, RexNode::NodeType astType){
    //First, we need to determine if this is part of some control structure.
    bool getParent = true;
    bool isControlStmt = false;
    vector<const NamedDecl*> vars;

    //Get the parent.
    auto parent = Context->getParents(*baseStmt);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Cast to some control structures.
        auto curIfStmt = parent[0].get<clang::IfStmt>();
        if (curIfStmt && isa<clang::IfStmt>(curIfStmt)){
            isControlStmt = true;
            vars = getVars(curIfStmt);
            break;
        }
        auto curForStmt = parent[0].get<clang::ForStmt>();
        if (curForStmt && isa<clang::ForStmt>(curForStmt)){
            isControlStmt = true;
            vars = getVars(curForStmt);
            break;
        }
        auto curWhileStmt = parent[0].get<clang::WhileStmt>();
        if (curWhileStmt && isa<clang::WhileStmt>(curWhileStmt)){
            isControlStmt = true;
            vars = getVars(curWhileStmt);
            break;
        }
        auto curDoStmt = parent[0].get<clang::DoStmt>();
        if (curDoStmt && isa<clang::DoStmt>(curDoStmt)){
            isControlStmt = true;
            vars = getVars(curDoStmt);
            break;
        }
        auto curSwitchStmt = parent[0].get<clang::SwitchStmt>();
        if (curSwitchStmt && isa<clang::SwitchStmt>(curSwitchStmt)){
            isControlStmt = true;
            vars = getVars(curSwitchStmt);
            break;
        }
        parent = Context->getParents(parent[0]);
    }

    //Now, adds a relation from the control vars to the publisher.
    if (!isControlStmt) return;
    for (const NamedDecl* var : vars){
        //Looks up the node of the var.
        RexNode* varNode = graph->findNode(generateID(var));
        RexEdge* infEdge;
        if (astItem) {
            if (varNode) {
                infEdge = new RexEdge(varNode, astItem, (astItem->getType() == RexNode::FUNCTION) ?
                                                        RexEdge::VAR_INFLUENCE_FUNC : RexEdge::VAR_INFLUENCE);
            } else {
                infEdge = new RexEdge(generateID(var), astItem, (astItem->getType() == RexNode::FUNCTION) ?
                                                                RexEdge::VAR_INFLUENCE_FUNC : RexEdge::VAR_INFLUENCE);
            }
        } else {
            if (varNode) {
                infEdge = new RexEdge(varNode, astID, (astType == RexNode::FUNCTION) ?
                                                        RexEdge::VAR_INFLUENCE_FUNC : RexEdge::VAR_INFLUENCE);
            } else {
                infEdge = new RexEdge(generateID(var), astID, (astType == RexNode::FUNCTION) ?
                                                                RexEdge::VAR_INFLUENCE_FUNC : RexEdge::VAR_INFLUENCE);
            }
        }
        graph->addEdge(infEdge);
    }
}

/**
 * Gets all variables in the condition of an if statement.
 * @param stmt The statement.
 * @return The variables.
 */
vector<const NamedDecl*> ROSWalker::getVars(const IfStmt* stmt){
    return getVars(stmt->getCond());
}


/**
 * Gets all variables in the condition of an for loop.
 * @param stmt The statement.
 * @return The variables.
 */
vector<const NamedDecl*> ROSWalker::getVars(const ForStmt* stmt){
    return getVars(stmt->getCond());
}


/**
 * Gets all variables in the condition of an while loop.
 * @param stmt The statement.
 * @return The variables.
 */
vector<const NamedDecl*> ROSWalker::getVars(const WhileStmt* stmt){
    return getVars(stmt->getCond());
}

/**
 * Gets all variables in the condition of a do loop.
 * @param stmt The statement.
 * @return The variables.
 */
vector<const NamedDecl*> ROSWalker::getVars(const DoStmt* stmt){
    return getVars(stmt->getCond());
}


/**
 * Gets all variables in the condition of a switch statement.
 * @param stmt The statement.
 * @return The variables.
 */
vector<const NamedDecl*> ROSWalker::getVars(const SwitchStmt* stmt){
    return getVars(stmt->getCond());
}


/**
 * Gets all variables in the condition of a statement. This is the helper method.
 * @param stmt The statement.
 * @return The variables.
 */
vector<const NamedDecl*> ROSWalker::getVars(const Stmt* condition){
    vector<const NamedDecl*> vars;

    //Gets the children of this statement.
    Stmt::const_child_range rng = condition->children();
    for (Stmt::const_child_iterator it = rng.begin(); it != rng.end(); it++) {

        if (isa<DeclRefExpr>(*it)) {
            //Adds the value to the array.
            auto decl = dyn_cast<DeclRefExpr>(*it);
            vars.push_back(dyn_cast<NamedDecl>(decl->getDecl()));
        } else if (isa<MemberExpr>(*it)) {
            //Adds the value to the array.
            auto decl = dyn_cast<MemberExpr>(*it);
            vars.push_back(dyn_cast<NamedDecl>(decl->getMemberDecl()));
        } else {
            //Gets the children.
            vector<const NamedDecl*> inner = getVars(*it);
            vars.insert(vars.end(), inner.begin(), inner.end());
        }
    }

    return vars;
}

/**
 * Adds the parent relationship.
 * @param baseDecl The base declaration.
 * @param baseID The base ID.
 */
void ROSWalker::addParentRelationship(const NamedDecl* baseDecl, string baseID){
    bool getParent = true;
    auto currentDecl = baseDecl;

    //Get the parent.
    auto parent = Context->getParents(*currentDecl);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current decl as named.
        currentDecl = parent[0].get<clang::NamedDecl>();
        if (currentDecl) {
            //Get the parent as a NamedDecl.
            string parentID = "";

            //Now, check to see what our current ID is.
            if (isa<FunctionDecl>(currentDecl) || isa<CXXRecordDecl>(currentDecl) ||
                    isa<CXXMethodDecl>(currentDecl) || isa<VarDecl>(currentDecl) || isa<FieldDecl>(currentDecl)){
                parentID = generateID(currentDecl);

                //Get the IDs.
                RexNode* dst = graph->findNode(baseID);
                RexNode* src = graph->findNode(parentID);
                if (graph->doesEdgeExist(parentID, baseID, RexEdge::CONTAINS)){
                    return;
                }

                RexEdge* edge = (!src) ? new RexEdge(parentID, dst, RexEdge::CONTAINS) :
                                new RexEdge(src, dst, RexEdge::CONTAINS);
                graph->addEdge(edge);
                return;
            }
        }

        parent = Context->getParents(parent[0]);
    }
}

/**
 * Gets the parent function to process.
 * @param baseFunc The base function.
 * @return The function declaration object.
 */
const FunctionDecl* ROSWalker::getParentFunction(const Stmt* baseFunc){
    bool getParent = true;

    //Get the parent.
    auto parent = Context->getParents(*baseFunc);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current decl as named.
        auto currentDecl = parent[0].get<clang::NamedDecl>();
        if (currentDecl && isa<clang::FunctionDecl>(currentDecl)){
            return dyn_cast<FunctionDecl>(currentDecl);
        }

        parent = Context->getParents(parent[0]);
    }

    return nullptr;
}