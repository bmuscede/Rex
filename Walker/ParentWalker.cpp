/////////////////////////////////////////////////////////////////////////////////////////////////////////
// ParentWalker.cpp
//
// Created By: Bryan J Muscedere
// Date: 06/07/17.
//
// Contains the majority of the logic for adding
// nodes and relations to the graph. Many of these
// are helper functions like ID generation or
// class resolution.
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
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include "ParentWalker.h"
#include "../Graph/LowMemoryTAGraph.h"

using namespace std;

TAGraph* ParentWalker::graph = new TAGraph();
vector<TAGraph*> ParentWalker::graphList = vector<TAGraph*>();
std::string ParentWalker::CALLBACK_FLAG = "isCallbackFunc";

/**
 * Constructor for the parent walker class.
 * @param Context The AST context.
 */
ParentWalker::ParentWalker(ASTContext *Context) : Context(Context) {
    ignoreLibraries.push_back(STANDARD_IGNORE);
}

/**
 * Destructor.
 */
ParentWalker::~ParentWalker() {}

/**
 * Adds graphs to the graph list.
 * @param graphs The graphs to add.
 */
void ParentWalker::addGraphs(vector<TAGraph *> graphs) {
    graphList.insert(graphList.end(), graphs.begin(), graphs.end());
}

/**
 * Gets the graph at the specified number.
 * @param num The specified number to get.
 * @return The graph at that location.
 */
TAGraph* ParentWalker::getGraph(int num) {
    return graphList.at(num);
}

/**
 * Deletes all TA graphs being maintained.
 */
void ParentWalker::deleteTAGraphs(){
    delete graph;
    for (int i = 0; i < graphList.size(); i++)
        delete graphList.at(i);

    graphList = vector<TAGraph*>();
}

/**
 * Deletes a TA graph by graph number.
 * @param num The number to delete.
 */
void ParentWalker::deleteTAGraph(int num){
    if (num < 0 || num >= graphList.size()) return;
    delete graphList.at(num);
    graphList.erase(graphList.begin() + num);
}

/**
 * Gets the number of graphs being maintained.
 * @return The number of graphs.
 */
int ParentWalker::getNumGraphs(){
    return (int) graphList.size();
}

/**
 * Stops processing the current graph.
 * @return The graph number of this graph.
 */
int ParentWalker::endCurrentGraph(){
    //Moves the current graph.
    graphList.push_back(graph);
    graph = new TAGraph();

    return (int) graphList.size() - 1;
}

/**
 * Outputs the current model.
 * @param fileName The filename to output.
 * @return Integer of graph number.
 */
int ParentWalker::generateCurrentTAModel(string fileName){
    return ParentWalker::generateTAModel(graph, fileName);
}

/**
 * Generates a TA model by number.
 * @param num The number to generate.
 * @param fileName The file to output it as.
 * @return Integer of graph number.
 */
int ParentWalker::generateTAModel(int num, string fileName){
    //Get the graph at the number.
    if (num >= graphList.size() || num < 0) return 0;
    return ParentWalker::generateTAModel(graphList.at(num), fileName);
}

/**
 * Outputs all graph models.
 * @param fileNames The file names to output as.
 * @return The return code.
 */
int ParentWalker::generateAllTAModels(vector<string> fileNames){
    if (fileNames.size() != graphList.size()) return 0;
    for (int i = 0; i < fileNames.size(); i++){
        int code = ParentWalker::generateTAModel(graphList.at(i), fileNames.at(i));
        if (code != 1) return 0;
    }

    return 1;
}

/**
 * Sets the mode of the graph.
 * @param minMode The processing mode.
 */
void ParentWalker::setCurrentGraphMinMode(bool minMode){
    graph->setMinMode(minMode);
}

/**
 * Resolves all TA files based on a compile commands database map.
 * @param databaseMap The database map.
 * @return Whether the operation was successful.
 */
bool ParentWalker::resolveAllTAModels(map<string, vector<string>> databaseMap){
    //Goes through the graphs and forces them to resolve.
    for (TAGraph* curGraph : graphList){
        bool status  = curGraph->resolveComponents(databaseMap);
        if (!status) return false;
    }

    return true;
}

/**
 * Only keeps features of a particular string.
 * @param features The features to keep.
 * @return Boolean indicating success.
 */
bool ParentWalker::onlyKeepFeatures(std::vector<std::string> features){
    //Goes through the graph and eliminates certain features.
    for (TAGraph* curGraph : graphList){
        if (dynamic_cast<LowMemoryTAGraph*>(curGraph)) {
            cerr << "Error: Scenario resolution cannot be conducted in low-memory mode! "
                    "Reprocess this graph using the regular generation system!" << endl;
            return false;
        }
        bool status  = curGraph->keepFeatures(features);
        if (!status) return false;
    }

    return true;
}

/**
 * Changes the type of the graph.
 * @param lowMem Low Memory mode toggle.
 * @param lowMemPath Low memory path.
 * @return Boolean indicating success.
 */
bool ParentWalker::changeGraphType(bool lowMem, string lowMemPath){
    if (!graph->isEmpty()) return false;
    bool minMode = graph->getMinMode();
    delete graph;

    if (lowMem) {
        if (lowMemPath == "") graph = new LowMemoryTAGraph();
        else graph = new LowMemoryTAGraph(lowMemPath);
    } else {
        graph = new TAGraph();
    }

    graph->setMinMode(minMode);
    return true;
}

/**
 * Purges the current graph.
 * @return Boolean indicating success.
 */
bool ParentWalker::purgeCurrentGraph(){
    if (dynamic_cast<LowMemoryTAGraph*>(graph)){
        dynamic_cast<LowMemoryTAGraph*>(graph)->purgeCurrentGraph();
        return true;
    }

    return false;
}

/**
 * Dumps current file to disk.
 * @param fileNum The current file number.
 * @param fileName The current file.
 * @return Boolean indicating success.
 */
bool ParentWalker::dumpCurrentFile(int fileNum, string fileName){
    if (dynamic_cast<LowMemoryTAGraph*>(graph)){
        dynamic_cast<LowMemoryTAGraph*>(graph)->dumpCurrentFile(fileNum, fileName);
        return true;
    }

    return false;
}

/**
 * Dumps the settings to disk.
 * @param files A list of files.
 * @param minMode Min mode toggle.
 * @return Boolean indicating success.
 */
bool ParentWalker::dumpCurrentSettings(vector<bs::path> files, bool minMode){
    if (dynamic_cast<LowMemoryTAGraph*>(graph)){
        dynamic_cast<LowMemoryTAGraph*>(graph)->dumpSettings(files, minMode);
        return true;
    }

    return false;
}

/*
 * Sets particular libraries to ignore when processing.
 * @param libraries The libraries to ignore.
 */
void ParentWalker::addLibrariesToIgnore(vector<string> libraries){
    ignoreLibraries = libraries;
}

/**
 * Gets the variable access method. Whether its a variable read or write.
 * @param op The declaration statement being analyzed.
 * @return A collection of access methods for each variable.
 */
map<string, ParentWalker::AccessMethod> ParentWalker::getAccessType(const DeclStmt* op){
    map<string, ParentWalker::AccessMethod> usageMap;

    if (op->isSingleDecl()){
        map<string, ParentWalker::AccessMethod> rhs;

        //Gets the variable declared here.
        auto var = dyn_cast<VarDecl>(op->getSingleDecl());
        if (!var) return usageMap;

        //Gets the expression.
        auto expr = var->getInit();
        if (!expr) return usageMap;
        usageMap[generateID(var)] = AccessMethod::WRITE;
        rhs = buildAccessMap(AccessMethod::READ, expr);

        //Generates the var linkage.
        generateVarLinkage(usageMap, rhs);
        usageMap.insert(rhs.begin(), rhs.end());
    } else {
        map<string, ParentWalker::AccessMethod> curUsage;
        map<string, ParentWalker::AccessMethod>  curRHS;
        for (auto it = op->decl_begin(); it != op->decl_end(); it++){
            auto curVar = dyn_cast<VarDecl>(*it);
            if (!curVar) continue;

            //Gets the expression.
            auto expr = curVar->getInit();
            if (!expr) return usageMap;
            curUsage[generateID(curVar)] = AccessMethod::WRITE;
            curRHS = buildAccessMap(AccessMethod::READ, expr);

            //Generates the var linkage.
            generateVarLinkage(curUsage, curRHS);
            usageMap.insert(curUsage.begin(), curUsage.end());
            usageMap.insert(curRHS.begin(), curRHS.end());
        }
    }

    return usageMap;
}

/**
 * Gets the access type based on binary operator.
 * @param op The binary operator to analyze.
 * @return A collection of access methods for each variable.
 */
map<string, ParentWalker::AccessMethod> ParentWalker::getAccessType(const BinaryOperator* op){
    map<string, ParentWalker::AccessMethod> lhsMap;
    map<string, ParentWalker::AccessMethod> rhsMap;
    map<string, ParentWalker::AccessMethod> usageMap;

    //Checks the lefthand side.
    Expr* lhs = op->getLHS();
    if (isa<DeclRefExpr>(lhs)){
        ParentWalker::AccessMethod method = determineAccess(true, op->getOpcode());
        usageMap[generateID(dyn_cast<DeclRefExpr>(lhs)->getDecl())] = method;
        lhsMap[generateID(dyn_cast<DeclRefExpr>(lhs)->getDecl())] = method;
    } else if (isa<MemberExpr>(lhs)) {
        ParentWalker::AccessMethod method = determineAccess(true, op->getOpcode());
        usageMap[generateID(dyn_cast<MemberExpr>(lhs)->getMemberDecl())] = method;
        lhsMap[generateID(dyn_cast<MemberExpr>(lhs)->getMemberDecl())] = method;
    } else  {
        //Get the opcode.
        ParentWalker::AccessMethod method = determineAccess(true, op->getOpcode());

        //Get the usage map.
        usageMap = buildAccessMap(method, lhs);
        lhsMap = usageMap;

        //Iterates through the map and combines.
        map<string, ParentWalker::AccessMethod>::iterator it;
        for (it = usageMap.begin(); it != usageMap.end(); it++){
            if ((usageMap[it->first] == ParentWalker::AccessMethod::READ && method == ParentWalker::AccessMethod::WRITE ||
                 usageMap[it->first] == ParentWalker::AccessMethod::WRITE && method == ParentWalker::AccessMethod::READ)){
                usageMap[it->first] = ParentWalker::BOTH;
                lhsMap[it->first] = ParentWalker::BOTH;
            }
        }
    }

    //Checks the righthand side.
    Expr* rhs = op->getRHS();
    if (isa<DeclRefExpr>(rhs) || isa<MemberExpr>(rhs)){
        string addID = (isa<DeclRefExpr>(rhs) ? generateID(dyn_cast<DeclRefExpr>(rhs)->getDecl()) :
                        generateID(dyn_cast<MemberExpr>(rhs)->getMemberDecl()));
        ParentWalker::AccessMethod method = determineAccess(false, op->getOpcode());

        //Checks what we do.
        if (usageMap.find(addID) != usageMap.end() &&
                (usageMap[addID] == ParentWalker::AccessMethod::READ && method == ParentWalker::AccessMethod::WRITE ||
                        usageMap[addID] == ParentWalker::AccessMethod::WRITE && method == ParentWalker::AccessMethod::READ)){
            usageMap[addID] = ParentWalker::AccessMethod::BOTH;
            rhsMap[addID] = ParentWalker::AccessMethod::BOTH;
        } else {
            usageMap[addID] = method;
            rhsMap[addID] = method;
        }
    } else {
        //Get the opcode.
        ParentWalker::AccessMethod method = determineAccess(false, op->getOpcode());

        //First, merge into map.
        map<string, ParentWalker::AccessMethod> otherMap = buildAccessMap(method, rhs);
        map<string, ParentWalker::AccessMethod>::iterator it;
        for (it = otherMap.begin(); it != otherMap.end(); it++){
            //Checks what we do.
            if (usageMap.find(it->first) != usageMap.end() &&
                (usageMap[it->first] == ParentWalker::AccessMethod::READ && it->second == ParentWalker::AccessMethod::WRITE ||
                 usageMap[it->first] == ParentWalker::AccessMethod::WRITE && it->second == ParentWalker::AccessMethod::READ)){
                usageMap[it->first] = ParentWalker::AccessMethod::BOTH;
                rhsMap[it->first] = ParentWalker::AccessMethod::BOTH;
            } else {
                usageMap[it->first] = it->second;
                rhsMap[it->first] = it->second;
            }

            //Now, checks to see what the method is.
            if (usageMap[it->first] == ParentWalker::AccessMethod::READ && method == ParentWalker::AccessMethod::WRITE ||
                usageMap[it->first] == ParentWalker::AccessMethod::WRITE && method == ParentWalker::AccessMethod::READ){
                usageMap[it->first] = ParentWalker::AccessMethod::BOTH;
                rhsMap[it->first] = it->second;
            }
        }

    }

    generateVarLinkage(lhsMap, rhsMap);
    return usageMap;
}

/**
 * Gets access types for variables in a unary expression.
 * @param op The unary operator expression to analyze.
 * @return A collection of access methods for each variable.
 */
map<string, ParentWalker::AccessMethod> ParentWalker::getAccessType(const UnaryOperator* op){
    map<string, ParentWalker::AccessMethod> usageMap;

    //First, determine the inner expr. We want to be sure we have the correct ID.
    Expr* inner = op->getSubExpr();

    //Checks if we're dealing with a singular item.
    if (isa<DeclRefExpr>(inner)){
        //Get the opcode.
        ParentWalker::AccessMethod method = determineAccess(op->getOpcode());
        if (method != ParentWalker::AccessMethod::NONE) {
            usageMap[generateID(dyn_cast<DeclRefExpr>(inner)->getDecl())] = method;
        }
    } else if (isa<MemberExpr>(inner)) {
        //Get the opcode.
        ParentWalker::AccessMethod method = determineAccess(op->getOpcode());
        if (method != ParentWalker::AccessMethod::NONE) {
            usageMap[generateID(dyn_cast<MemberExpr>(inner)->getMemberDecl())] = method;
        }
    } else {
        //Get the opcode.
        ParentWalker::AccessMethod method = determineAccess(op->getOpcode());

        //We just return the base usage map.
        usageMap = buildAccessMap(method, inner);
    }

    return usageMap;
}

/**
 * Helper method for the two walkers that handles statements. This is done
 * to avoid repeats in both walkers.
 * @param statement The statement to analyze.
 * @return The type of operation.
 */
ParentWalker::ROSType ParentWalker::handleMinimalStmt(Stmt *statement) {
    ROSType rtype = ROSType::ROS_NONE;

    //Looks for publish, subscribe, and time.
    if (CXXOperatorCallExpr* overload = dyn_cast<CXXOperatorCallExpr>(statement)){
        string type = overload->getType().getAsString();
        if (type.compare("class " + PUBLISHER_CLASS) == 0){
            const NamedDecl* assignee = getAssignee(overload);
            const MemberExpr* assignStmt = getAssignStmt(overload);
            if (!assignee || !assignStmt) return ROSType::ROS_NONE;

            recordAssociations(assignee, assignStmt, type);
            rtype = ROSType::PUB;
        } else if (type.compare("class " + SUBSCRIBER_CLASS) == 0) {
            const NamedDecl *assignee = getAssignee(overload);
            const MemberExpr *assignStmt = getAssignStmt(overload);
            if (!assignee || !assignStmt) return ROSType::ROS_NONE;

            recordAssociations(assignee, assignStmt, type);
            rtype = ROSType::SUB;
        } else if (type.compare("class " + TIMER_CLASS) == 0) {
            const NamedDecl *assignee = getAssignee(overload);
            const MemberExpr *assignStmt = getAssignStmt(overload);
            if (!assignee || !assignStmt) return ROSType::ROS_NONE;

            recordAssociations(assignee, assignStmt, type);
            rtype = ROSType::TIMER;
        }
    } else if (CXXConstructExpr* cxxExpr = dyn_cast<CXXConstructExpr>(statement)) {
        if (isSubscriberObj(cxxExpr)) {
            recordParentSubscribe(cxxExpr);
            rtype = ROSType::SUB;
        } else if (isPublisherObj(cxxExpr)) {
            recordParentPublish(cxxExpr);
            rtype = ROSType::PUB;
        }
    }

    //Handles the internals of each action.
    if (CallExpr* callExpr = dyn_cast<CallExpr>(statement)){
        //Deal with the expression.
        if (isPublish(callExpr)) {
            recordPublish(callExpr);
            rtype = ROSType::PUB;
        } else if (isSubscribe(callExpr)) {
            recordSubscribe(callExpr);
            rtype = ROSType::SUB;
        } else if (isAdvertise(callExpr)) {
            recordAdvertise(callExpr);
        } else if (isTimer(callExpr)) {
            recordTimer(callExpr);
            rtype = ROSType::TIMER;
        }
    }

    return rtype;
}

/**
 * Helper method for the two walkers that handles variable declarations. This is done
 * to avoid repeats in both walkers.
 * @param decl The variable decl to analyze.
 * @param pubEdge Whether we publish the edge.
 * @return The type of operation.
 */
bool ParentWalker::handleMinimalVarDecl(VarDecl *decl, bool pubEdge) {
    //Add ROS specific nodes and fields.
    CXXRecordDecl* parent = decl->getType()->getAsCXXRecordDecl();
    if (!parent) return false;

    //Checks if we have a publisher or such.
    string parentName = parent->getQualifiedNameAsString();
    if (parentName.compare(PUBLISHER_CLASS) == 0 ||
        parentName.compare(SUBSCRIBER_CLASS) == 0 ||
        parentName.compare(NODE_HANDLE_CLASS) == 0 ||
        parentName.compare(TIMER_CLASS) == 0){
        recordROSActionMinimal(decl, parentName, pubEdge);
        return true;
    }

    return false;
}

/**
 * Helper method for the two walkers that handles field declarations. This is done
 * to avoid repeats in both walkers.
 * @param decl The field declaration to analyze.
 * @param pubEdge Whether we publish edges or not.
 * @return The type of operation.
 */
bool ParentWalker::handleMinimalFieldDecl(FieldDecl *decl, bool pubEdge) {
    //Add ROS specific nodes and fields.
    CXXRecordDecl* parent = decl->getType()->getAsCXXRecordDecl();
    if (!parent) return false;

    //Checks if we have a publisher or such.
    string parentName = parent->getQualifiedNameAsString();
    if (parentName.compare(PUBLISHER_CLASS) == 0 ||
        parentName.compare(SUBSCRIBER_CLASS) == 0 ||
        parentName.compare(NODE_HANDLE_CLASS) == 0 ||
        parentName.compare(TIMER_CLASS) == 0){
        recordROSActionMinimal(decl, parentName, pubEdge);
        return true;
    }

    return false;
}

/**
 * Checks if we're constructing a node handle.
 * @param ctor The CXX constructor.
 * @return Whether its a node handle.
 */
bool ParentWalker::isNodeHandlerObj(const CXXConstructExpr* ctor){
    return isClass(ctor, NODE_HANDLE_CLASS);
}

/**
 * Checks if we're constructing a subscriber.
 * @param ctor The CXX constructor.
 * @return Whether its a subscriber.
 */
bool ParentWalker::isSubscriberObj(const CXXConstructExpr* ctor){
    return isClass(ctor, SUBSCRIBER_CLASS);
}

/**
 * Checks if we're constructing a publisher.
 * @param ctor The CXX constructor.
 * @return Whether its a publisher.
 */
bool ParentWalker::isPublisherObj(const CXXConstructExpr* ctor){
    return isClass(ctor, PUBLISHER_CLASS);
}

/**
 * Checks if we're making a publish call.
 * @param ctor The CXX constructor.
 * @return Whether its a publish call.
 */
bool ParentWalker::isPublish(const CallExpr *expr) {
    return isFunction(expr, PUBLISH_FUNCTION);
}

/**
 * Checks if we're making a subscribe call.
 * @param ctor The CXX constructor.
 * @return Whether its a subscribe call.
 */
bool ParentWalker::isSubscribe(const CallExpr *expr) {
    return isFunction(expr, SUBSCRIBE_FUNCTION);
}

/**
 * Checks if we're making an advertise call.
 * @param ctor The CXX constructor.
 * @return Whether its an advertise call.
 */
bool ParentWalker::isAdvertise(const CallExpr* expr){
    return isFunction(expr, ADVERTISE_FUNCTION);
}

bool ParentWalker::isTimer(const CallExpr* expr){
    return isFunction(expr, TIMER_FUNCTION);
}

/**
 * Records any associations that might exist between entities.
 * @param assignee The assignee.
 * @param assign The assign element.
 * @param type The type.
 */
void ParentWalker::recordAssociations(const NamedDecl* assignee, const MemberExpr* assign, string type) {
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
        currentPublisherOutdated = assigneeNode;
    } else if (type.compare("class " + SUBSCRIBER_CLASS) == 0) {
        currentSubscriber = assigneeNode;
    } else {
        currentTimer = assigneeNode;
    }
}

/**
 * Records a minimal ROS action.
 * @param decl The decl to add.
 * @param type The type of node to add.
 * @param pubEdge Whether we publish an edge too.
 */
void ParentWalker::recordROSActionMinimal(const NamedDecl* decl, string type, bool pubEdge){
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
        string ID = generateID(parentFinal);
        string name = generateName(parentFinal);

        //Checks if we add the node.
        if (!graph->doesNodeExist(ID)) {
            classNode = new RexNode(ID, name, RexNode::NodeType::CLASS);
            graph->addNode(classNode);
        } else {
            classNode = graph->findNode(ID);
        }

        string filename = generateFileName(parentFinal);
        classNode->addMultiAttribute(FILENAME_ATTR, filename);
    }

    //Next, we make our Rex variable object.
    string varNodeID = generateID(decl);
    string varNodeName = generateName(decl);
    RexNode::NodeType nType;
    if (type.compare(PUBLISHER_CLASS) == 0){
        nType = RexNode::PUBLISHER;
    } else if (type.compare(SUBSCRIBER_CLASS) == 0){
        nType = RexNode::SUBSCRIBER;
    } else if (type.compare(TIMER_CLASS) == 0){
        nType = RexNode::TIMER;
    } else {
        nType = RexNode::NODE_HANDLE;
    }

    RexNode* rexVarNode = new RexNode(varNodeID, varNodeName, nType);
    graph->addNode(rexVarNode);

    if (classNode && pubEdge) {
        RexEdge *edge = new RexEdge(classNode, rexVarNode, RexEdge::EdgeType::CONTAINS);
        graph->addEdge(edge);
    }
}

/**
 * Gets the item that was assigned based on an operator call.
 * @param parent The parent object.
 * @return The decl that is the assignee.
 */
const NamedDecl* ParentWalker::getAssignee(const CXXOperatorCallExpr* parent){
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

/**
 * Gets the assignment statement.
 * @param parent The parent object.
 * @return The assign statement part of this operator.
 */
const MemberExpr* ParentWalker::getAssignStmt(const CXXOperatorCallExpr* parent){
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

/**
 * Gets the parent class of a decl.
 * @param decl The decl to find the parent class.
 * @return The class.
 */
const CXXRecordDecl* ParentWalker::getParentClass(const NamedDecl* decl){
    bool getParent = true;

    //Get the parent.
    auto parent = Context->getParents(*decl);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current decl as named.
        auto currentDecl = parent[0].get<clang::NamedDecl>();
        if (currentDecl && isa<clang::CXXRecordDecl>(currentDecl)){
            return dyn_cast<CXXRecordDecl>(currentDecl);
        }

        parent = Context->getParents(parent[0]);
    }

    return nullptr;
}

/**
 * Checks whether an entity is a class.
 * @param ctor The class.
 * @param className The name of the class.
 * @return Whether the two items compare.
 */
bool ParentWalker::isClass(const CXXConstructExpr* ctor, string className){
    //Get the underlying class.
    QualType type = ctor->getBestDynamicClassTypeExpr()->getType();
    if (type->isArrayType()) return false;

    auto record = ctor->getBestDynamicClassType();
    if (record == nullptr) return false;

    //Check the qualified name.
    if (record->getQualifiedNameAsString().compare(className)) return false;
    return true;
}

/**
 * Checks whether an entity is a function..
 * @param expr The call expression.
 * @param functionName The name of the function.
 * @return Whether the two items compare.
 */
bool ParentWalker::isFunction(const CallExpr *expr, string functionName){
    //Gets the underlying callee.
    if (expr->getCalleeDecl() == nullptr) return false;
    auto callee = expr->getCalleeDecl()->getAsFunction();
    if (callee == nullptr) return false;

    //Checks the value of the callee.
    if (callee->getQualifiedNameAsString().compare(functionName)) return false;
    return true;
}

/**
 * Gets the decl assigned to a class.
 * @param expr The constructor expression.
 * @return The decl that was assigned.
 */
const NamedDecl* ParentWalker::getParentAssign(const CXXConstructExpr* expr){
    bool getParent = true;

    //Get the parent.
    auto parent = Context->getParents(*expr);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current decl as named.
        auto currentDecl = parent[0].get<clang::NamedDecl>();
        if (currentDecl && (isa<clang::VarDecl>(currentDecl) || isa<clang::FieldDecl>(currentDecl)
                            || isa<clang::ParmVarDecl>(currentDecl))){
            return currentDecl;
        } else if (currentDecl && isa<clang::FunctionDecl>(currentDecl)){
            getParent = false;
            continue;
        }

        parent = Context->getParents(parent[0]);
    }

    return nullptr;
}

/**
 * Gets the callback function based on a string.
 * @param callbackQualified The name of the callback function.
 * @return The node of the callback function.
 */
RexNode* ParentWalker::findCallbackFunction(std::string callbackQualified){
    //First, check to see if the string starts with &.
    if (callbackQualified.at(0) == '&'){
        callbackQualified = callbackQualified.erase(0, 1);
    }

    //Find a node by name.
    return graph->findNodeByEndName(callbackQualified);
}

/**
 * Gets arguments of a call expression.
 * @param expr The expression to get arguments.
 * @return The vector of arguments.
 */
vector<string> ParentWalker::getArgs(const CallExpr* expr){
    string sBuffer;

    //Creates a vector.
    vector<string> args;

    //Get the arguments for the CallExpr.
    int numArgs = expr->getNumArgs();
    auto argList = expr->getArgs();
    for (int i = 0; i < numArgs; i++){
        sBuffer = "";
        auto curArg = argList[i];

        //Pushes the argument string into a vector.
        llvm::raw_string_ostream strStream(sBuffer);
        curArg->printPretty(strStream, nullptr, Context->getPrintingPolicy());
        sBuffer = strStream.str();
        args.push_back(sBuffer);
    }

    return args;
}

/**
 * Gets the type of publisher.
 * @param expr The expression of the publisher.
 * @return The type of publisher.
 */
string ParentWalker::getPublisherType(const CallExpr* expr) {
    //Get the string representation of the expression.
    string sBuffer = "";
    llvm::raw_string_ostream strStream(sBuffer);
    expr->printPretty(strStream, nullptr, Context->getPrintingPolicy());
    sBuffer = strStream.str();

    //Loop through the string.
    string type = "";
    bool inBetween = false;
    for (int i = 0; i < sBuffer.size(); i++){
        if (sBuffer.at(i) == '<'){
            inBetween = true;
        } else if (inBetween && sBuffer.at(i) == '>'){
            break;
        } else if (inBetween) {
            type += sBuffer.at(i);
        }
    }

    if (type.compare("") == 0) return "none";
    return type;
}

/**
 * Gets the parent variable of a call expression.
 * @param callExpr The call expression.
 * @return The parent variable.
 */
NamedDecl* ParentWalker::getParentVariable(const Expr *callExpr) {
    //Get the CXX Member Call.
    auto call = dyn_cast<CXXMemberCallExpr>(callExpr);
    if (call == nullptr) return nullptr;

    //Next, get the implicit object.
    auto parentVar = call->getImplicitObjectArgument();
    if (parentVar == nullptr) return nullptr;
    auto refCalleeDecl = parentVar->getReferencedDeclOfCallee();
    if (refCalleeDecl == nullptr) return nullptr;

    return dyn_cast<NamedDecl>(refCalleeDecl);
}

/**
 * Determines variable access.
 * @param lhs Whether we're dealing with a left hand side.
 * @param opcode The type of operation.
 * @return The access type.
 */
ParentWalker::AccessMethod ParentWalker::determineAccess(bool lhs, BinaryOperator::Opcode opcode){
    if (!lhs) return ParentWalker::AccessMethod::READ;

    switch(opcode){
        case BinaryOperator::Opcode::BO_AddAssign:
        case BinaryOperator::Opcode::BO_DivAssign:
        case BinaryOperator::Opcode::BO_MulAssign:
        case BinaryOperator::Opcode::BO_SubAssign:
            return ParentWalker::AccessMethod::BOTH;
        case BinaryOperator::Opcode::BO_Assign:
        case BinaryOperator::Opcode::BO_AndAssign:
        case BinaryOperator::Opcode::BO_OrAssign:
        case BinaryOperator::Opcode::BO_RemAssign:
        case BinaryOperator::Opcode::BO_ShlAssign:
        case BinaryOperator::Opcode::BO_ShrAssign:
        case BinaryOperator::Opcode::BO_XorAssign:
            return ParentWalker::AccessMethod::WRITE;
        case BinaryOperator::Opcode::BO_PtrMemD:
        case BinaryOperator::Opcode::BO_PtrMemI:
            return ParentWalker::AccessMethod::NONE;
        default:
            return ParentWalker::AccessMethod::READ;
    }
}

/**
 * Determines access based on a unary opcode.
 * @param opcode The opcode.
 * @return The access type.
 */
ParentWalker::AccessMethod ParentWalker::determineAccess(UnaryOperator::Opcode opcode){
    switch(opcode){
        case UnaryOperator::Opcode::UO_PostDec:
        case UnaryOperator::Opcode::UO_PostInc:
        case UnaryOperator::Opcode::UO_PreDec:
        case UnaryOperator::Opcode::UO_PreInc:
            //Add write.
            return ParentWalker::AccessMethod::WRITE;
        case UnaryOperator::Opcode::UO_Plus:
        case UnaryOperator::Opcode::UO_Minus:
            //Add read.
            return ParentWalker::AccessMethod::READ;
        default:
            return ParentWalker::AccessMethod::NONE;
    }
}

/**
 * Builds an access map based on previous accesses.
 * @param prevAccess The upper/parent access type.
 * @param curExpr The current expression.
 * @return The updated access map.
 */
map<string, ParentWalker::AccessMethod> ParentWalker::buildAccessMap(ParentWalker::AccessMethod prevAccess,
                                                                     const Expr* curExpr){
    const Expr* prevExpr;

    //Rip out any unneeded items.
    do {
        prevExpr = curExpr;
        curExpr = curExpr->IgnoreParenCasts();
        curExpr = curExpr->IgnoreImpCasts();
        curExpr = curExpr->IgnoreImplicit();
        curExpr = curExpr->IgnoreCasts();
        curExpr = curExpr->IgnoreParens();
        curExpr = curExpr->IgnoreConversionOperator();
        curExpr = curExpr->IgnoreParenImpCasts();
        curExpr = curExpr->IgnoreParenLValueCasts();
    } while (prevExpr != curExpr);

    //Next, we determine the type of expression.
    if (isa<BinaryOperator>(curExpr)){
        return getAccessType(dyn_cast<BinaryOperator>(curExpr));
    } else if (isa<UnaryOperator>(curExpr))  {
        return getAccessType(dyn_cast<UnaryOperator>(curExpr));
    } else if (isa<DeclRefExpr>(curExpr)) {
        map<string, ParentWalker::AccessMethod> singleMap;
        singleMap[generateID(dyn_cast<DeclRefExpr>(curExpr)->getDecl())] = prevAccess;
        return singleMap;
    } else if (isa<MemberExpr>(curExpr)){
        map<string, ParentWalker::AccessMethod> singleMap;
        singleMap[generateID(dyn_cast<MemberExpr>(curExpr)->getMemberDecl())] = prevAccess;
        return singleMap;
    }

    //Check if we're dealing with other, unimportant expressions.
    if (isa<IntegerLiteral>(curExpr) || isa<CharacterLiteral>(curExpr) || isa<FloatingLiteral>(curExpr)
        || isa<ImaginaryLiteral>(curExpr) || isa<UserDefinedLiteral>(curExpr) || isa<GNUNullExpr>(curExpr)){
        return map<string, ParentWalker::AccessMethod>();
    }

#ifdef EXPR_DEBUG
    cerr << "Error: Expression cannot be detected!" << endl;
    cerr << "Expression on line " << Context->getSourceManager().getSpellingLineNumber(curExpr->getLocStart())
         << " in file " << Context->getSourceManager().getFilename(curExpr->getLocStart()).str() << endl;
    curExpr->dump();
#endif

    return map<string, ParentWalker::AccessMethod>();
}

/**
 * Generates which variables write to others.
 * @param lhsMap The left hand side.
 * @param rhsMap The right hand side.
 */
void ParentWalker::generateVarLinkage(map<string, ParentWalker::AccessMethod> lhsMap,
                                      map<string, ParentWalker::AccessMethod> rhsMap){
    //Next, we process the relations between variables.
    for (auto lhsItem : lhsMap){
        for (auto rhsItem : rhsMap){
            if ((lhsItem.second == ParentWalker::BOTH || lhsItem.second == ParentWalker::WRITE) &&
                (rhsItem.second == ParentWalker::BOTH || rhsItem.second == ParentWalker::READ)){
                //Get the variables.
                if (graph->doesEdgeExist(rhsItem.first, lhsItem.first, RexEdge::VAR_WRITES)) continue;

                //Add the edge.
                RexEdge* edge = new RexEdge(rhsItem.first, lhsItem.first, RexEdge::VAR_WRITES);
                graph->addEdge(edge);
            }
        }
    }
}

/**
 * Checks whether a statement is in the system header.
 * @param statement The statement.
 * @return Whether its in the system header.
 */
bool ParentWalker::isInSystemHeader(const Stmt *statement) {
    if (statement == nullptr) return false;

    //Get the system header.
    bool isIn;
    try {
        //Get the source manager.
        auto &manager = Context->getSourceManager();

        //Check if in header.
        isIn = isInSystemHeader(manager, statement->getBeginLoc());
    } catch (...) {
        return false;
    }

    return isIn;
}

/**
 * Checks whether a decl is in the system header.
 * @param decl The declaration.
 * @return Whether its in the system header.
 */
bool ParentWalker::isInSystemHeader(const Decl *decl){
    if (decl == nullptr) return false;

    //Get the system header.
    bool isIn;
    try {
        //Get the source manager.
        auto &manager = Context->getSourceManager();

        //Check if in header.
        isIn = isInSystemHeader(manager, decl->getBeginLoc());
    } catch (...) {
        return false;
    }

    return isIn;
}

/**
 * Records the parent subscriber.
 * @param expr The expression to add.
 */
void ParentWalker::recordParentSubscribe(const CXXConstructExpr* expr){
    //First, see if we have parents.
    const NamedDecl* parentVar = getParentAssign(expr);
    if (!parentVar) {
        parentVar = generateROSNode(expr, RexNode::SUBSCRIBER);
    }

    recordParentGeneric(generateID(parentVar), RexNode::SUBSCRIBER);
}

/**
 * Records the parent publisher.
 * @param expr The expression to add.
 */
void ParentWalker::recordParentPublish(const CXXConstructExpr* expr){
    //First, see if we have parents.
    const NamedDecl* parentVar = getParentAssign(expr);
    if (!parentVar) {
        parentVar = generateROSNode(expr, RexNode::PUBLISHER);
    }

    recordParentGeneric(generateID(parentVar), RexNode::PUBLISHER);
}

/**
 * Records a generic parent ROS item.
 * @param parentID The ID to add.
 * @param parentName The name to add.
 * @param type The type to add.
 */
void ParentWalker::recordParentGeneric(string parentID, RexNode::NodeType type){
    //Next, we record the relationship between the two items.
    RexNode* rosNode = graph->findNode(parentID);

    //Keeps track of the node being published/subscribed.
    if (type == RexNode::PUBLISHER) {
        currentPublisher = rosNode;
        currentPublisherOutdated = rosNode;
    } else {
        currentSubscriber = rosNode;
    }

}

/**
 * Records a subscriber object.
 * @param expr The expression with the subscriber.
 */
void ParentWalker::recordSubscribe(const CallExpr* expr){
    if (currentSubscriber == nullptr) return;

    //Now, adds information to the destination node.
    int numArgs = expr->getNumArgs();
    auto subscriberArgs = getArgs(expr);

    //Get the name of the topic and record it if not present.
    string topicName = validateStringArg(subscriberArgs[0]);
    recordTopic(topicName);
    RexNode* topic = graph->findNode(TOPIC_PREFIX + topicName);
    RexEdge* topEdge = new RexEdge(topic, currentSubscriber, RexEdge::SUBSCRIBE);
    graph->addEdge(topEdge);

    //Record attributes.
    currentSubscriber->addSingleAttribute(ROS_TOPIC_BUF_SIZE, subscriberArgs[1]);
    currentSubscriber->addSingleAttribute(ROS_NUM_ATTRIBUTES, to_string(numArgs));
    currentSubscriber->addSingleAttribute(ROS_CALLBACK, subscriberArgs[2]);

    //Gets the callback information.
    RexNode* callback = findCallbackFunction(subscriberArgs[2]);

    //Finally, adds in the callback function.
    RexEdge* callbackEdge;
    if (callback){
        callback->addSingleAttribute(CALLBACK_FLAG, "1");
        callbackEdge = new RexEdge(currentSubscriber, callback, RexEdge::CALLS);
    } else {
        //Remove the & at the beginning.
        string callback = subscriberArgs[2];
        if (callback.at(0) == '&'){
             callback = callback.erase(0, 1);
        }

        callbackEdge = new RexEdge(currentSubscriber, callback, RexEdge::CALLS);
    }
    graph->addEdge(callbackEdge);

    currentSubscriber = nullptr;
}

/**
 * Records a publisher object.
 * @param expr The publisher object.
 */
void ParentWalker::recordPublish(const CallExpr* expr){
    //Get the publisher object.
    auto parent = getParentVariable(expr);
    if (!parent) return;
    RexNode* parentVar = graph->findNodeByName(generateID(parent));
    if (!parentVar) return;

    //Next, gets the topic it publishes to.
    vector<RexEdge*> destinations = graph->findEdgesBySrc(parentVar->getID(), false);
    RexNode* topic = nullptr;
    for (int i = 0; i < destinations.size(); i++){
        auto item = destinations.at(i);

        //Narrows the item.
        if (item->getType() != RexEdge::ADVERTISE) continue;
        topic = item->getDestination();
    }
    if (!topic) return;

    //Now, generates the edge.
    RexEdge* edge = new RexEdge(parentVar, topic, RexEdge::PUBLISH);

    //Gets the publish data.
    auto args = getArgs(expr);
    string data = validateStringArg(args.at(0));
    if (data.size() > PUB_MAX) data = data.substr(0, PUB_MAX);
    edge->addSingleAttribute(ROS_PUB_DATA, data);

    //Adds the result to the graph.
    graph->addEdge(edge);
}

/**
 * Records an advertise call.
 * @param expr The call with the advertise.
 */
void ParentWalker::recordAdvertise(const CallExpr* expr) {
    if (currentPublisher == nullptr) return;

    //First, get the arguments.
    int numArgs = expr->getNumArgs();
    auto publisherArgs = getArgs(expr);

    //Get the name of the topic and record it if not present.
    string topicName = validateStringArg(publisherArgs[0]);
    recordTopic(topicName);
    RexNode* topic = graph->findNode(TOPIC_PREFIX + topicName);
    RexEdge* topEdge = new RexEdge(currentPublisher, topic, RexEdge::ADVERTISE);
    graph->addEdge(topEdge);

    //Record specific attributes.
    currentPublisher->addSingleAttribute(ROS_TOPIC_BUF_SIZE, publisherArgs[1]);
    currentPublisher->addSingleAttribute(ROS_NUM_ATTRIBUTES, to_string(numArgs));
    currentPublisher->addSingleAttribute(ROS_PUB_TYPE, getPublisherType(expr));

    currentPublisher = nullptr;
}

/**
 * Records a ROS timer from a call expression.
 * @param expr The call expression to record.
 */
void ParentWalker::recordTimer(const CallExpr* expr){
    if (currentTimer == nullptr) return;

    //First, get the arguments.
    int numArgs = expr->getNumArgs();
    auto timerArgs = getArgs(expr);

    //Gets the duration.
    string timerFreq = timerArgs[0];
    if (boost::starts_with(timerFreq, TIMER_PREFIX_1)){
        boost::replace_all(timerFreq, TIMER_PREFIX_1, "");
        boost::replace_all(timerFreq, ")", "");
    } else if (boost::starts_with(timerFreq, TIMER_PREFIX_2)){
        boost::replace_all(timerFreq, TIMER_PREFIX_2, "");
        boost::replace_all(timerFreq, ")", "");
    }

    //Checks if we have a one shot timer.
    string oneshot = timerArgs[3];
    if (oneshot == "" || oneshot == "false"){
        oneshot = "0";
    } else {
        oneshot = "1";
    }

    //Adds specific attributes.
    currentTimer->addSingleAttribute(TIMER_DURATION, timerFreq);
    currentTimer->addSingleAttribute(TIMER_ONESHOT, oneshot);

    //Get the callback function.
    RexNode* callback = findCallbackFunction(timerArgs[1]);

    //Finally, adds in the callback function.
    RexEdge* callbackEdge;
    if (callback){
        callbackEdge = new RexEdge(currentTimer, callback, RexEdge::SET_TIME);
    } else {
        //Remove the & at the beginning.
        string callback = timerArgs[1];
        if (callback.at(0) == '&'){
            callback = callback.erase(0, 1);
        }

        callbackEdge = new RexEdge(currentTimer, callback, RexEdge::SET_TIME);
    }
    graph->addEdge(callbackEdge);

    currentTimer = nullptr;
}

/**
 * Records a topic.
 * @param name The name of the topic.
 */
void ParentWalker::recordTopic(string name){
    string ID = TOPIC_PREFIX + name;
    if (graph->doesNodeExist(ID)) return;

    //Create the node.
    RexNode* node = new RexNode(ID, name, RexNode::TOPIC);
    graph->addNode(node);
}

/**
 * Generates a ROS node.
 * @param expr The expression.
 * @param type The type of node.
 * @return The decl generated.
 */
const NamedDecl* ParentWalker::generateROSNode(const CXXConstructExpr* expr, RexNode::NodeType type){
    //First, get the parent NamedDecl.
    bool getParent = true;
    auto parent = Context->getParents(*expr);
    const NamedDecl* result;
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current decl as named.
        result = parent[0].get<clang::NamedDecl>();
        if (result) {
            getParent = false;
            continue;
        }

        parent = Context->getParents(parent[0]);
    }

    //Next, create a node based on this.
    bool notAdded = true;
    int num = 0;
    while (notAdded){
        string ID = generateID(result) + "::" + to_string(num);
        if (!graph->doesNodeExist(ID)){
            string name = generateName(result) + "\'s " + ((type == RexNode::PUBLISHER) ? "Publisher" : "Subscriber");
            RexNode* dst = new RexNode(ID, name, type);
            graph->addNode(dst);

            //Add a contains relation.
            graph->addEdge(new RexEdge(graph->findNode(generateID(result)), dst, RexEdge::CONTAINS));
            notAdded = false;
            continue;
        }

        num++;
    }

    return result;
}

/**
 * Generates a unique ID based on a decl.
 * @param decl The decl to generate the ID.
 * @return A string of the ID.
 */
string ParentWalker::generateID(const NamedDecl* decl){
    //Gets the canonical decl.
    decl = dyn_cast<NamedDecl>(decl->getCanonicalDecl());
    string name = "";

    //Generates a special name for function overloading.
    if (isa<FunctionDecl>(decl) || isa<CXXMethodDecl>(decl)){
        const FunctionDecl* cur = decl->getAsFunction();
        name = cur->getReturnType().getAsString() + "-" + decl->getNameAsString();
        for (int i = 0; i < cur->getNumParams(); i++){
            name += "-" + cur->parameters().data()[i]->getType().getAsString();
        }
    } else {
        name = decl->getNameAsString();
    }


    bool getParent = true;
    bool recurse = false;
    const NamedDecl* originalDecl = decl;

    //Get the parent.
    auto parent = Context->getParents(*decl);
    while(getParent){
        //Check if it's empty.
        if (parent.empty()){
            getParent = false;
            continue;
        }

        //Get the current decl as named.
        decl = parent[0].get<clang::NamedDecl>();
        if (decl) {
            name = generateID(decl) + "::" + name;
            recurse = true;
            getParent = false;
            continue;
        }

        parent = Context->getParents(parent[0]);
    }

    //Sees if no true qualified name was used.
    Decl::Kind kind = originalDecl->getKind();
    if (!recurse) {
        if (kind != Decl::Function && kind == Decl::CXXMethod){
            //We need to get the parent function.
            const DeclContext *parentContext = originalDecl->getParentFunctionOrMethod();

            //If we have nullptr, get the parent function.
            if (parentContext != nullptr) {
                string parentQualName = generateID(static_cast<const FunctionDecl *>(parentContext));
                name = parentQualName + "::" + originalDecl->getNameAsString();
            }
        }
    }

    //Finally, check if we have a main method.
    if (name.compare("int-main-int-char **") == 0 || name.compare("int-main") == 0){
        name = Context->getSourceManager().getFilename(originalDecl->getBeginLoc()).str() + "--" + name;
    }

    return name;
}

/**
 * Generates a name based on a decl.
 * @param decl The declaration.
 * @return The string of the decl.
 */
string ParentWalker::generateName(const NamedDecl* decl){
    string name = decl->getQualifiedNameAsString();

    //Check if we have a main method.
    if (name.compare("main") == 0){
        name = Context->getSourceManager().getFilename(decl->getBeginLoc()).str() + "\'s " + name;
    }

    return name;
}

/**
 * Gets the filename of the decl.
 * @param decl The declaration.
 * @return The filename the declaration is in.
 */
string ParentWalker::generateFileName(const NamedDecl* decl){
    //Gets the file name.
    SourceManager& SrcMgr = Context->getSourceManager();
    auto fullLoc = Context->getFullLoc(decl->getBeginLoc());
    if (!fullLoc.isValid()) return string();

    string fileName = SrcMgr.getFilename(fullLoc).str();

    //Use boost to get the absolute path.
    boost::filesystem::path fN = boost::filesystem::path(fileName);
    string newPath = canonical(fN.normalize()).string();
    return newPath;
}

/**
 * Gets the location of the function decl.
 * @param decl The decl to add.
 */
void ParentWalker::recordParentClassLoc(const FunctionDecl* decl){
    if (!decl) return;
    auto funcDec = decl->getDefinition();
    if (!funcDec) return;

    //Gets the parent class.
    const CXXRecordDecl* classDec = getParentClass(decl);
    if (!classDec) return;
    RexNode* parentClass = graph->findNode(generateID(classDec));
    if (!parentClass) return;

    //Generates the filename.
    string baseFN = generateFileName(funcDec);
    parentClass->addMultiAttribute(FILENAME_ATTR, baseFN);
}

/**
 * Validates a string argument.
 * @param name The argument to validate.
 * @return The cleaned up string.
 */
string ParentWalker::validateStringArg(string name){
    //Checks the topic name.
    std::string prefix("\"");
    if (!name.compare(0, prefix.size(), prefix)){
        name = name.substr(1);
    }
    if (!name.compare(name.size() - 1, prefix.size(), prefix)){
        name = name.substr(0, name.size() - 1);
    }

    return name;
}

/**
 * Generates a TA model.
 * @param graph The graph to generate.
 * @param fileName The filename to save.
 * @return Status code.
 */
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

/**
 * Checks whether an item is in a system header file.
 * @param manager The source manager.
 * @param loc The source location.
 * @return Whether its in a system header file.
 */
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
        for (string curLibrary : ignoreLibraries){
            if (loc.string().find(curLibrary) != string::npos) return true;
        }
    }

    return false;
}
