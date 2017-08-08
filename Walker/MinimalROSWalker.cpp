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

    //Looks for advertise and subscribe.
    if (CXXOperatorCallExpr* overload = dyn_cast<CXXOperatorCallExpr>(statement)){
        string type = overload->getType().getAsString();
        if (type.compare("class " + PUBLISHER_CLASS) == 0){
            const NamedDecl* assignee = getAssignee(overload);
            const MemberExpr* assignStmt = getAssignStmt(overload);
            if (!assignee || !assignStmt) return true;

            recordAssociations(assignee, assignStmt, type);
        } else if (type.compare("class " + SUBSCRIBER_CLASS) == 0) {
            const NamedDecl *assignee = getAssignee(overload);
            const MemberExpr *assignStmt = getAssignStmt(overload);
            if (!assignee || !assignStmt) return true;

            recordAssociations(assignee, assignStmt, type);
        }
    }

    //Also looks for advertise and subscribe.
    //TODO: There are some cases where this is used as opposed to the block above. Figure this out.
    if (CXXConstructExpr* cxxExpr = dyn_cast<CXXConstructExpr>(statement)) {
/*
        //Generate the printing policy.
        clang::LangOptions LangOpts;
        clang::PrintingPolicy Policy(LangOpts);

        string stmt;
        raw_string_ostream stream(stmt);
        cxxExpr->printPretty(stream, NULL, PrintingPolicy(LangOptions()));

        //Flush ostream buffer.
        stream.flush();
        cout << stmt << endl;

        if (isNodeHandlerObj(cxxExpr)) {
            recordNodeHandle(cxxExpr);
            recordParentNodeHandle(cxxExpr);
        } else if (isSubscriberObj(cxxExpr)) {
            recordParentSubscribe(cxxExpr, fileName);
*/
        if (isSubscriberObj(cxxExpr)) {
           // recordParentSubscribe(cxxExpr, fileName);
        } else if (isPublisherObj(cxxExpr)) {
          //  recordParentPublish(cxxExpr, fileName);
        }
    }

    //Handles the internals of each action.
    if (CallExpr* callExpr = dyn_cast<CallExpr>(statement)){
        //Deal with the expression.
        if (isPublish(callExpr)) {
            recordPublish(callExpr);
        } else if (isSubscribe(callExpr)) {
            recordSubscribe(callExpr);
        } else if (isAdvertise(callExpr)) {
            recordAdvertise(callExpr);
        }
    }

    return true;
}

bool MinimalROSWalker::VisitVarDecl(VarDecl* decl) {
    if (isInSystemHeader(decl)) return true;

    //Add ROS specific nodes and fields.
    CXXRecordDecl* parent = decl->getType()->getAsCXXRecordDecl();
    if (!parent) return true;

    //Checks if we have a publisher or such.
    string parentName = parent->getQualifiedNameAsString();
    if (parentName.compare(PUBLISHER_CLASS) == 0 ||
            parentName.compare(SUBSCRIBER_CLASS) == 0 ||
            parentName.compare(NODE_HANDLE_CLASS) == 0){
        recordROSActionMinimal(decl, parentName);
    }

    return true;
}

bool MinimalROSWalker::VisitFieldDecl(FieldDecl* decl) {
    if (isInSystemHeader(decl)) return true;

    //Add ROS specific nodes and fields.
    CXXRecordDecl* parent = decl->getType()->getAsCXXRecordDecl();
    if (!parent) return true;

    //Checks if we have a publisher or such.
    string parentName = parent->getQualifiedNameAsString();
    if (parentName.compare(PUBLISHER_CLASS) == 0 || parentName.compare(SUBSCRIBER_CLASS) == 0 ||
            parentName.compare(NODE_HANDLE_CLASS) == 0){
        recordROSActionMinimal(decl, parentName);
    }

    return true;
}

void MinimalROSWalker::recordAssociations(const NamedDecl* assignee, const MemberExpr* assign, string type) {
    //Get the assignee with the name.
    string assigneeID = generateID(assignee);
    RexNode *assigneeNode = graph->findNode(assigneeID);

    //Get the NH object.
    MemberExpr *base = dyn_cast<MemberExpr>(assign->getBase());
    if (base != nullptr) {
        NamedDecl *assigner = dyn_cast<NamedDecl>(base->getReferencedDeclOfCallee());
        string assignerID = generateID(assigner);
        RexNode *assignerNode = graph->findNode(assignerID);

        //Now, creates an edge
        RexEdge *edge = new RexEdge(assignerNode, assigneeNode, RexEdge::EdgeType::REFERENCES);
        graph->addEdge(edge);
    }

    //Sets the type.
    if (type.compare("class " + PUBLISHER_CLASS) == 0){
        currentPublisher = assigneeNode;
    } else {
        currentSubscriber = assigneeNode;
    }
}

void MinimalROSWalker::recordParentSubscribe(const CXXConstructExpr* expr, string className){
    if (className.compare("") == 0) return;
    recordParentGeneric(className, className, RexNode::SUBSCRIBER);
}

void MinimalROSWalker::recordParentPublish(const CXXConstructExpr* expr, string fileName){
    if (fileName.compare("") == 0) return;
    recordParentGeneric(fileName, fileName, RexNode::PUBLISHER);
}

void MinimalROSWalker::recordROSActionMinimal(const NamedDecl* decl, string type){
    //First, get the parent class.
    auto parent = Context->getParents(*decl);
    const CXXRecordDecl* parentFinal = nullptr;
    while(true){
        //Check if it's empty.
        if (parent.empty()) break;

        //Get the current decl as named.
        parentFinal = parent[0].get<clang::CXXRecordDecl>();
        if (parentFinal) break;

        parent = Context->getParents(parent[0]);
    }

    //First, we want to add the class as a node.
    RexNode* classNode = nullptr;
    if (parentFinal){
        string className = parentFinal->getQualifiedNameAsString();

        classNode = new RexNode(className, className, RexNode::NodeType::CLASS);
        graph->addNode(classNode);
    }

    //Next, we make our Rex variable object.
    string varNodeID = generateID(decl);
    string varNodeName = generateName(decl);
    RexNode::NodeType nType = (type.compare(PUBLISHER_CLASS) == 0 ? RexNode::PUBLISHER :
                               ((type.compare(SUBSCRIBER_CLASS) == 0) ? RexNode::SUBSCRIBER :
                                RexNode::NodeType::NODE_HANDLE));

    RexNode* rexVarNode = new RexNode(varNodeID, varNodeName, nType);
    graph->addNode(rexVarNode);

    if (classNode) {
        RexEdge *edge = new RexEdge(classNode, rexVarNode, RexEdge::EdgeType::CONTAINS);
        graph->addEdge(edge);
    }
}

const NamedDecl* MinimalROSWalker::getAssignee(const CXXOperatorCallExpr* parent){
    //Gets the lhs of the expression.
    int num = parent->getNumArgs();
    if (num != 2) return nullptr;
    const Expr* lhs = parent->getArg(0);

    //Gets the base expression type.
    const MemberExpr* lhsCast = dyn_cast<MemberExpr>(lhs);
    if (!lhsCast) return nullptr;

    //Next, gets the referenced item.
    const NamedDecl* decl = dyn_cast<NamedDecl>(lhsCast->getReferencedDeclOfCallee());
    return decl;
}

const MemberExpr* MinimalROSWalker::getAssignStmt(const CXXOperatorCallExpr* parent){
    int num = parent->getNumArgs();
    if (num != 2) return nullptr;
    const Expr* rhs = parent->getArg(1);

    //Digs down into all the temporary expressions.
    const MaterializeTemporaryExpr* tempExpr1 = dyn_cast<MaterializeTemporaryExpr>(rhs);
    const CXXMemberCallExpr* tempExpr2 = dyn_cast<CXXMemberCallExpr>(dyn_cast<CXXBindTemporaryExpr>(
            dyn_cast<ImplicitCastExpr>(tempExpr1->GetTemporaryExpr())->getSubExpr())->getSubExpr());
    const MemberExpr* lastItem = dyn_cast<MemberExpr>(tempExpr2->getCallee());
    return lastItem;
}

string MinimalROSWalker::recordFileLoc(SourceLocation loc){
    //First, get the filename.
    string fileName = getFileName(loc);
    if (fileName.compare("") == 0) return "";

    //Check if the node exists.
    if (graph->doesNodeExist(fileName)) return fileName;

    //Next, creates the node.
    RexNode* node = new RexNode(fileName, fileName, RexNode::FILE);
    graph->addNode(node);

    return fileName;
}

string MinimalROSWalker::getFileName(SourceLocation loc){
    //Generates the filename.
    SourceManager& SrcMgr = Context->getSourceManager();
    const FileEntry* Entry = SrcMgr.getFileEntryForID(SrcMgr.getFileID(loc));

    //Returns the entry.
    if (!Entry) return "";
    auto name = Entry->tryGetRealPathName();
    string nEnt = name.str();
    if (nEnt.compare("") == 0) return Entry->getName();
    return nEnt;
}