#include <fstream>
#include <iostream>
#include "clang/AST/Mangle.h"
#include "ROSWalker.h"
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include "../Graph/RexNode.h"

using namespace std;

ROSWalker::ROSWalker(ASTContext *Context) : ParentWalker(Context) { }

ROSWalker::~ROSWalker(){ }

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

    //Deal with variable usages.
    if (BinaryOperator* binOp = dyn_cast<BinaryOperator>(statement)){
        //Get all variable usages.
        auto accesses = getAccessType(binOp);
        recordVarUsage(getParentFunction(binOp), accesses);
    } else if (UnaryOperator* unaryOp = dyn_cast<UnaryOperator>(statement)){
        auto accesses = getAccessType(unaryOp);
        recordVarUsage(getParentFunction(unaryOp), accesses);
    }

    return true;
}

bool ROSWalker::VisitVarDecl(VarDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Handle ROS messages.
    bool rosBased = handleMinimalVarDecl(decl);
    if (rosBased) return true;

    //Record the variable declaration.
    recordVarDecl(decl);

    return true;
}

bool ROSWalker::VisitFieldDecl(FieldDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Handle ROS messages.
    bool rosBased = handleMinimalFieldDecl(decl);
    if (rosBased) return true;

    //Record the function declaration.
    recordFieldDecl(decl);

    return true;
}

bool ROSWalker::VisitFunctionDecl(FunctionDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Record the function declaration.
    recordFunctionDecl(decl);
    checkForCallbacks(decl);

    return true;
}

bool ROSWalker::VisitCXXRecordDecl(CXXRecordDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Check what type of structure it is.
    if (decl->isClass()){
        recordClassDecl(decl);
    }

    return true;
}

bool ROSWalker::VisitDeclRefExpr(DeclRefExpr *declRef) {
    if (isInSystemHeader(declRef)) return true;

    //Check if we have an if-statement noted here.
    if (usedInIfStatement(declRef) || usedInLoop(declRef)){
        recordControlFlow(declRef);
    }

    return true;
}

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

void ROSWalker::recordClassDecl(const CXXRecordDecl *decl){
    //Generates some fields.
    string ID = generateID(decl);
    string name = generateName(decl);

    //Creates the node.
    RexNode* node = new RexNode(ID, name, RexNode::CLASS);
    graph->addNode(node);

    //Get the parent.
    addParentRelationship(decl, ID);
}

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

void ROSWalker::handleFullPub(const Stmt* statement){
    //Gets the publisher.
    if (!currentPublisherOutdated) return;

    //Parent Function Relation Adder.
    RexEdge* callEdge = recordParentFunction(statement, currentPublisherOutdated);

    //Control Structure Adder.
    recordROSControl(statement, currentPublisherOutdated, callEdge);
}

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
}

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

bool ROSWalker::usedInIfStatement(const DeclRefExpr* declRef){
    //Iterate through and find if we have an if statement parent.
    bool getParent = true;
    auto parent = Context->getParents(*declRef);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current parent as an if statement.
        auto ifStmt = parent[0].get<clang::IfStmt>();
        if (ifStmt) {
            return true;
        }

        parent = Context->getParents(parent[0]);
    }

    return false;
}

bool ROSWalker::usedInLoop(const DeclRefExpr* declRef){
    //Iterate through and find if we have an if statement parent.
    bool getParent = true;
    auto parent = Context->getParents(*declRef);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current parent as an if statement.
        auto whileStmt = parent[0].get<clang::WhileStmt>();
        if (whileStmt) {
            return true;
        }

        auto forStmt = parent[0].get<clang::ForStmt>();
        if (forStmt){
            return true;
        }

        parent = Context->getParents(parent[0]);
    }

    return false;
}

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
                if (!graph->doesEdgeExist(functionID, varID, RexEdge::READS)){
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

void ROSWalker::recordControlFlow(const DeclRefExpr* expr){
    //Get the decl responsible.
    const ValueDecl* decl = expr->getDecl();

    //Get the ID and find it.
    string declID = generateID(decl);
    RexNode* refNode = graph->findNode(declID);
    if (!refNode) return;

    //Adds 1 to the control flag attr.
    refNode->addSingleAttribute(CONTROL_FLAG, "1");
}

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

void ROSWalker::recordROSControl(const Stmt* baseStmt, RexNode* rosItem, RexEdge* callsRel){
    //First, we need to determine if this is part of some control structure.
    bool getParent = true;
    bool isControlStmt = false;

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
            break;
        }
        auto curForStmt = parent[0].get<clang::ForStmt>();
        if (curForStmt && isa<clang::ForStmt>(curForStmt)){
            isControlStmt = true;
            break;
        }
        auto curWhileStmt = parent[0].get<clang::WhileStmt>();
        if (curWhileStmt && isa<clang::WhileStmt>(curWhileStmt)){
            isControlStmt = true;
            break;
        }
        auto curDoStmt = parent[0].get<clang::DoStmt>();
        if (curDoStmt && isa<clang::DoStmt>(curDoStmt)){
            isControlStmt = true;
            break;
        }
        auto curSwitchStmt = parent[0].get<clang::SwitchStmt>();
        if (curSwitchStmt && isa<clang::SwitchStmt>(curSwitchStmt)){
            isControlStmt = true;
            break;
        }
        parent = Context->getParents(parent[0]);
    }

    //Records the relationship.
    if (isControlStmt){
        rosItem->addSingleAttribute(ROS_CONTROL_FLAG, "1");
    } else if (rosItem->getSingleAttribute(ROS_CONTROL_FLAG).compare("0") == 0) {
        rosItem->addSingleAttribute(ROS_CONTROL_FLAG, "0");
    }

    //Now, adds it on the calls edge.
    if (callsRel){
        if (isControlStmt){
            callsRel->addSingleAttribute(ROS_CONTROL_FLAG, "1");
        } else if (callsRel->getSingleAttribute(ROS_CONTROL_FLAG).compare("1") != 0) {
            callsRel->addSingleAttribute(ROS_CONTROL_FLAG, "0");
        }
    }
}

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