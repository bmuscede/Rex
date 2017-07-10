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

bool ParentWalker::isNodeHandlerObj(const CXXConstructExpr* ctor){
    return isClass(ctor, NODE_HANDLE_CLASS);
}

bool ParentWalker::isSubscriberObj(const CXXConstructExpr* ctor){
    return isClass(ctor, SUBSCRIBER_CLASS);
}

bool ParentWalker::isPublisherObj(const CXXConstructExpr* ctor){
    return isClass(ctor, PUBLISHER_CLASS);
}

bool ParentWalker::isPublish(const CallExpr *expr) {
    return isFunction(expr, PUBLISH_FUNCTION);
}

bool ParentWalker::isSubscribe(const CallExpr *expr) {
    return isFunction(expr, SUBSCRIBE_FUNCTION);
}

bool ParentWalker::isAdvertise(const CallExpr* expr){
    return isFunction(expr, ADVERTISE_FUNCTION);
}

bool ParentWalker::isClass(const CXXConstructExpr* ctor, string className){
    //Get the underlying class.
    QualType type = ctor->getType(); //Different clang version QualType type = ctor->getBestDynamicClassTypeExpr()->getType();
    if (type->isArrayType()) return false; //TODO: Bandaid fix! Not sure why this works.

    auto record = ctor->getBestDynamicClassType();
    if (record == nullptr) return false;

    //Check the qualified name.
    if (record->getQualifiedNameAsString().compare(className)) return false;
    return true;
}

bool ParentWalker::isFunction(const CallExpr *expr, string functionName){
    //Gets the underlying callee.
    if (expr->getCalleeDecl() == nullptr) return false;
    auto callee = expr->getCalleeDecl()->getAsFunction();
    if (callee == nullptr) return false;

    //Checks the value of the callee.
    if (callee->getQualifiedNameAsString().compare(functionName)) return false;
    return true;
}

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

RexNode* ParentWalker::findCallbackFunction(std::string callbackQualified){
    //First, check to see if the string starts with &.
    if (callbackQualified.at(0) == '&'){
        callbackQualified = callbackQualified.erase(0, 1);
    }

    //Find a node by name.
    return graph->findNodeByName(callbackQualified);
}

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

string ParentWalker::getParentVariable(const Expr *callExpr) {
    //Get the CXX Member Call.
    auto call = dyn_cast<CXXMemberCallExpr>(callExpr);
    if (call == nullptr) return string();

    //Next, get the implicit object.
    auto parentVar = call->getImplicitObjectArgument();
    if (parentVar == nullptr) return string();

    //Finally, works on a print pretty system.
    string sBuffer = "";
    llvm::raw_string_ostream strStream(sBuffer);
    parentVar->printPretty(strStream, nullptr, Context->getPrintingPolicy());
    return strStream.str();
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

void ParentWalker::recordParentSubscribe(const CXXConstructExpr* expr){
    //First, see if we have parents.
    const NamedDecl* parentVar = getParentAssign(expr);
    if (!parentVar) return;

    recordParentGeneric(generateID(parentVar), generateName(parentVar), RexNode::SUBSCRIBER);
}

void ParentWalker::recordParentPublish(const CXXConstructExpr* expr){
    //First, see if we have parents.
    const NamedDecl* parentVar = getParentAssign(expr);
    if (!parentVar) return;

    recordParentGeneric(generateID(parentVar), generateName(parentVar), RexNode::PUBLISHER);
}

void ParentWalker::recordParentGeneric(string parentID, string parentName, RexNode::NodeType type){
    //Next, we record the relationship between the two items.
    RexNode* src = graph->findNode(parentID);
    RexNode* dst;
    if (type == RexNode::PUBLISHER) {
        src->addSingleAttribute(ROS_PUB_VAR_FLAG, "1");
        dst = graph->generatePublisherNode(parentID, parentName);
    } else {
        src->addSingleAttribute(ROS_SUB_VAR_FLAG, "1");
        dst = graph->generateSubscriberNode(parentID, parentName);
    }

    //Creates the edge.
    RexEdge* edge = new RexEdge(src, dst, ((type == RexNode::PUBLISHER) ? RexEdge::PUBLISH : RexEdge::SUBSCRIBE));
    graph->addEdge(edge);

    //Keeps track of the node being published/subscribed.
    if (type == RexNode::PUBLISHER) {
        currentPublisher = dst;
    } else {
        currentSubscriber = dst;
    }

}

void ParentWalker::recordParentNodeHandle(const CXXConstructExpr* expr){
    //TODO
}

void ParentWalker::recordNodeHandle(const CXXConstructExpr* expr){
    //TODO
}


void ParentWalker::recordSubscribe(const CallExpr* expr){
    if (currentSubscriber == nullptr) return;

    //Now, adds information to the destination node.
    int numArgs = expr->getNumArgs();
    auto subscriberArgs = getArgs(expr);

    //Get the name of the topic and record it if not present.
    string topicName = validateStringArg(subscriberArgs[0]);
    recordTopic(topicName);
    RexNode* topic = graph->findNode(TOPIC_PREFIX + topicName);
    RexEdge* topEdge = new RexEdge(currentSubscriber, topic, RexEdge::SUBSCRIBE);
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
        callbackEdge = new RexEdge(currentSubscriber, callback, RexEdge::CALLS);
    } else {
        callbackEdge = new RexEdge(currentSubscriber, subscriberArgs[2], RexEdge::CALLS);
    }
    graph->addEdge(callbackEdge);

    currentSubscriber = nullptr;
}

//TODO: This won't work since the qualified name isn't right.
void ParentWalker::recordPublish(const CallExpr* expr){
    //Get the publisher object.
    auto parent = getParentVariable(expr);
    if (parent.compare(string()) == 0) return;
    RexNode* parentVar = graph->findNodeByName(parent);
    if (!parentVar) return;

    //Next, gets the publisher that it's pointing to.
    auto destinations = graph->findEdgesBySrc(parentVar->getID(), false);
    RexNode* pubItem = nullptr;
    for (int i = 0; i < destinations.size(); i++){
        auto item = destinations.at(i);

        //Narrows down the item.
        if (item->getType() != RexEdge::SUBSCRIBE) continue;
        auto dest = item->getDestination();
        if (dest->getType() != RexNode::PUBLISHER) continue;

        if (pubItem == nullptr){
            pubItem = dest;
        } else if (atoi(pubItem->getSingleAttribute(ROS_NUMBER).c_str()) < atoi(item->getSingleAttribute(ROS_NUMBER).c_str())){
            pubItem = dest;
        }
    }
    if (!pubItem) return;

    //Next, gets the topic it publishes to.
    destinations = graph->findEdgesBySrc(pubItem->getID(), false);
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

void ParentWalker::recordTopic(string name){
    string ID = TOPIC_PREFIX + name;
    if (graph->doesNodeExist(ID)) return;

    //Create the node.
    RexNode* node = new RexNode(ID, name, RexNode::TOPIC);
    graph->addNode(node);
}

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

    return name;
}

string ParentWalker::generateName(const NamedDecl* decl){
    return decl->getQualifiedNameAsString();
}

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