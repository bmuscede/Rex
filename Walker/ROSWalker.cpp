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

    //Handle ROS Publisher and Subscriber object creations.
    if (CXXConstructExpr* cxxExpr = dyn_cast<CXXConstructExpr>(statement)) {
        //Generate the printing policy.
        clang::LangOptions LangOpts;
        clang::PrintingPolicy Policy(LangOpts);

        if (isNodeHandlerObj(cxxExpr)) {
            recordNodeHandle(cxxExpr);
            recordParentNodeHandle(cxxExpr);
        } else if (isSubscriberObj(cxxExpr)) {
            recordParentSubscribe(cxxExpr);
        } else if (isPublisherObj(cxxExpr)) {
            recordParentPublish(cxxExpr);
        }
    }

    //Handle ROS statements.
    if (CallExpr* callExpr = dyn_cast<CallExpr>(statement)){
        //Deal with the expression.
        if (isPublish(callExpr)) recordPublish(callExpr);
        else if (isSubscribe(callExpr)) recordSubscribe(callExpr);
        else if (isAdvertise(callExpr)) recordAdvertise(callExpr);

        //Next, deal with call expressions.
        recordCallExpr(callExpr);
    }

    if (DeclRefExpr* usageExpr = dyn_cast<DeclRefExpr>(statement)){
        //Deal with the expression.
        recordVarUsage(usageExpr);
    }

    return true;
}

bool ROSWalker::VisitFunctionDecl(FunctionDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Record the function declaration.
    recordFunctionDecl(decl);
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

bool ROSWalker::VisitVarDecl(VarDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Record the function declaration.
    recordVarDecl(decl);
    return true;
}

bool ROSWalker::VisitFieldDecl(FieldDecl* decl){
    if (isInSystemHeader(decl)) return true;

    //Record the function declaration.
    recordFieldDecl(decl);
    return true;
}

void ROSWalker::recordFunctionDecl(const FunctionDecl* decl){
    //Generates some fields.
    string ID = generateID(decl);
    string name = generateName(decl);

    //Creates the node.
    RexNode* node = new RexNode(ID, name, RexNode::FUNCTION);
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
    graph->addNode(node);

    //Get the parent.
    addParentRelationship(decl, ID);
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

//TODO: This doesn't work quite well yet. Fields and ParmDecls aren't captured.
void ROSWalker::recordVarUsage(const DeclRefExpr* expr){
    //Get the sub-variable.
    auto subVar = expr->getFoundDecl();
    string subVarID = generateID(subVar);
    RexNode* subVarNode = graph->findNode(subVarID);

    //Get the parent expression.
    auto parDecl = getParentFunction(expr);
    if (parDecl == nullptr) return;

    //Get the ID for the parent.
    string callerID = generateID(parDecl);
    RexNode* callerNode = graph->findNode(callerID);

    //Checks whether we can resolve.
    if (callerNode == nullptr){
        return;
    }

    //Adds the edge.
    if (graph->doesEdgeExist(callerID, subVarID, RexEdge::REFERENCES)) return;
    RexEdge* edge = (subVarNode == nullptr) ?
                    new RexEdge(callerNode, subVarID, RexEdge::REFERENCES) :
                    new RexEdge(callerNode, subVarNode, RexEdge::REFERENCES);
    graph->addEdge(edge);
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

            if (isa<clang::FunctionDecl>(currentDecl)){
                const FunctionDecl* funcDec = static_cast<const FunctionDecl*>(currentDecl);
                parentID = generateID(funcDec);
            } else {
                parentID = generateID(currentDecl);
            }

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

        parent = Context->getParents(parent[0]);
    }
}

const FunctionDecl* ROSWalker::getParentFunction(const Expr* callExpr){
    bool getParent = true;

    //Get the parent.
    auto parent = Context->getParents(*callExpr);
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