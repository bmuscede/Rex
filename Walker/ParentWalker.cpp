//
// Created by bmuscede on 06/07/17.
//

#include <fstream>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "ParentWalker.h"

using namespace std;

TAGraph* ParentWalker::graph = new TAGraph();
vector<TAGraph*> ParentWalker::graphList = vector<TAGraph*>();

ParentWalker::ParentWalker(ASTContext *Context) : Context(Context) {}
ParentWalker::~ParentWalker() {}

void ParentWalker::deleteTAGraphs(){
    delete graph;
    for (int i = 0; i < graphList.size(); i++)
        delete graphList.at(i);

    graphList = vector<TAGraph*>();
}

void ParentWalker::deleteTAGraph(int num){
    if (num < 0 || num >= graphList.size()) return;
    delete graphList.at(num);
    graphList.erase(graphList.begin() + num);
}

int ParentWalker::getNumGraphs(){
    return (int) graphList.size();
}

int ParentWalker::endCurrentGraph(){
    //Moves the current graph.
    graphList.push_back(graph);
    graph = new TAGraph();

    return (int) graphList.size() - 1;
}

int ParentWalker::generateCurrentTAModel(string fileName){
    return ParentWalker::generateTAModel(graph, fileName);
}

int ParentWalker::generateTAModel(int num, string fileName){
    //Get the graph at the number.
    if (num >= graphList.size() || num < 0) return 0;
    return ParentWalker::generateTAModel(graphList.at(num), fileName);
}

int ParentWalker::generateAllTAModels(vector<string> fileNames){
    if (fileNames.size() != graphList.size()) return 0;
    for (int i = 0; i < fileNames.size(); i++){
        int code = ParentWalker::generateTAModel(graphList.at(i), fileNames.at(i));
        if (code != 1) return 0;
    }

    return 1;
}

bool ParentWalker::isInSystemHeader(const Stmt *statement) {
    if (statement == nullptr) return false;

    //Get the system header.
    bool isIn;
    try {
        //Get the source manager.
        auto &manager = Context->getSourceManager();

        //Check if in header.
        isIn = isInSystemHeader(manager, statement->getLocStart());
    } catch (...) {
        return false;
    }

    return isIn;
}

bool ParentWalker::isInSystemHeader(const Decl *decl){
    if (decl == nullptr) return false;

    //Get the system header.
    bool isIn;
    try {
        //Get the source manager.
        auto &manager = Context->getSourceManager();

        //Check if in header.
        isIn = isInSystemHeader(manager, decl->getLocStart());
    } catch (...) {
        return false;
    }

    return isIn;
}

int ParentWalker::generateTAModel(TAGraph* graph, string fileName){
    //Purge the edges.
    graph->purgeUnestablishedEdges(true);

    //Gets the string for the model.
    string model = graph->getTAModel();

    //Creates the file stream.
    ofstream file(fileName);
    if (!file.is_open()) return 0;

    //Writes to the file.
    file << model;

    file.close();
    return 1;
}

bool ParentWalker::isInSystemHeader(const SourceManager& manager, SourceLocation loc) {
    //Get the expansion location.
    auto expansionLoc = manager.getExpansionLoc(loc);

    //Check if we have a valid location.
    if (expansionLoc.isInvalid()) {
        return false;
    }

    //Get if we have a system header.
    bool sysHeader = manager.isInSystemHeader(loc);
    if (sysHeader) return sysHeader;

    //Now, check to see if we have a ROS library.
    string libLoc = expansionLoc.printToString(manager);
    libLoc = libLoc.substr(0, libLoc.find(":"));
    if (boost::algorithm::ends_with(libLoc, ".h") || boost::algorithm::ends_with(libLoc, ".hpp")){
        boost::filesystem::path loc = boost::filesystem::canonical(boost::filesystem::path(libLoc));
        if (loc.string().find("/ros/") != string::npos) return true;
    }
    return false;
}