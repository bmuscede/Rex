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

    //Adds the file to the TA graph.
    string fileName = recordFileLoc(statement->getLocStart());

    //Starts by checking if there is a ROS component.
    if (CXXConstructExpr* cxxExpr = dyn_cast<CXXConstructExpr>(statement)) {
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
        } else if (isPublisherObj(cxxExpr)) {
            recordParentPublish(cxxExpr, fileName);
        }
    }

    //Handle ROS statements.
    if (CallExpr* callExpr = dyn_cast<CallExpr>(statement)){
        //Deal with the expression.
        if (isPublish(callExpr)) recordPublish(callExpr);
        else if (isSubscribe(callExpr)) recordSubscribe(callExpr);
        else if (isAdvertise(callExpr)) recordAdvertise(callExpr);
    }

    return true;
}

bool MinimalROSWalker::VisitFunctionDecl(FunctionDecl* decl) {
    if (isInSystemHeader(decl)) return true;

    //Adds the file to the TA graph.
    recordFileLoc(decl->getLocation());

    return true;
}

void MinimalROSWalker::recordParentSubscribe(const CXXConstructExpr* expr, string fileName){
    if (fileName.compare("") == 0) return;
    recordParentGeneric(fileName, fileName, RexNode::SUBSCRIBER);
}

void MinimalROSWalker::recordParentPublish(const CXXConstructExpr* expr, string fileName){
    if (fileName.compare("") == 0) return;
    recordParentGeneric(fileName, fileName, RexNode::PUBLISHER);
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