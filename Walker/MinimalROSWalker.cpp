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
    recordFileLoc(statement->getLocStart());

    //Starts by checking if there is a ROS component.
    //TODO

    return true;
}

bool MinimalROSWalker::VisitFunctionDecl(FunctionDecl* decl) {
    if (isInSystemHeader(decl)) return true;

    //Adds the file to the TA graph.
    recordFileLoc(decl->getLocation());

    return true;
}

void MinimalROSWalker::recordFileLoc(SourceLocation loc){
    //First, get the filename.
    string fileName = getFileName(loc);
    if (fileName.compare("") == 0) return;

    //Check if the node exists.
    if (graph->doesNodeExist(fileName)) return;

    //Next, creates the node.
    RexNode* node = new RexNode(fileName, fileName, RexNode::FILE);
    graph->addNode(node);
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