/////////////////////////////////////////////////////////////////////////////////////////////////////////
// ParentWalker.h
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

#ifndef REX_PARENTWALKER_H
#define REX_PARENTWALKER_H

#include <map>
#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include "llvm/Support/CommandLine.h"
#include "../Graph/TAGraph.h"

class ROSWalker;
class MinimalROSWalker;

using namespace llvm;
using namespace clang;
using namespace clang::tooling;

class ParentWalker {
public:
    //Constructor/Destructor
    explicit ParentWalker(ASTContext *Context);
    virtual ~ParentWalker();

    //Graph Operations
    static void deleteTAGraphs();
    static void deleteTAGraph(int num);
    static int getNumGraphs();
    static int endCurrentGraph();
    static int generateCurrentTAModel(std::string fileName);
    static int generateTAModel(int num, std::string fileName);
    static int generateAllTAModels(std::vector<std::string> fileName);
    static void setCurrentGraphMinMode(bool minMode);
    static bool resolveAllTAModels(std::map<std::string, std::vector<std::string>> databaseMap);

    //Processing Operations
    void addLibrariesToIgnore(std::vector<std::string> libraries);

    static std::string CALLBACK_FLAG;

protected:
    const std::string FILENAME_ATTR = "filename";

    enum ROSType {ROS_NONE, PUB, SUB, TIMER};
    static TAGraph* graph;
    static std::vector<TAGraph*> graphList;
    ASTContext *Context;

    RexNode* currentSubscriber = nullptr;
    RexNode* currentPublisherOutdated = nullptr;
    RexNode* currentPublisher = nullptr;
    RexNode* currentTimer = nullptr;

    //ROS Names
    const std::string PUBLISHER_CLASS = "ros::Publisher";
    const std::string SUBSCRIBER_CLASS = "ros::Subscriber";
    const std::string NODE_HANDLE_CLASS = "ros::NodeHandle";
    const std::string TIMER_CLASS = "ros::Timer";

    //Minimal Handlers
    ParentWalker::ROSType handleMinimalStmt(Stmt* statement);
    bool handleMinimalVarDecl(VarDecl* decl, bool pubEdge = true);
    bool handleMinimalFieldDecl(FieldDecl* decl, bool pubEdge = true);

    //ROS Handlers
    bool isPublish(const CallExpr* expr);
    bool isSubscribe(const CallExpr* expr);
    bool isAdvertise(const CallExpr* expr);
    bool isTimer(const CallExpr* expr);

    //System Headers
    bool isInSystemHeader(const Stmt* statement);
    bool isInSystemHeader(const Decl* decl);

    //ROS Recorders
    void recordSubscribe(const CallExpr* expr);
    void recordPublish(const CallExpr* expr);
    void recordAdvertise(const CallExpr* expr);
    void recordTimer(const CallExpr* expr);
    void recordTopic(std::string name);

    //Name Helper Functions
    std::string generateID(const NamedDecl* decl);
    std::string generateName(const NamedDecl* decl);
    std::string validateStringArg(std::string name);
    std::string generateFileName(const NamedDecl* decl);
    void recordParentClassLoc(const FunctionDecl* decl);

    //Variable Access Methods
    enum AccessMethod {NONE, BOTH, READ, WRITE};
    std::map<std::string, ParentWalker::AccessMethod> getAccessType(const DeclStmt* op);
    std::map<std::string, ParentWalker::AccessMethod> getAccessType(const BinaryOperator* op);
    std::map<std::string, ParentWalker::AccessMethod> getAccessType(const UnaryOperator* op);

private:
    //ROS Topic Name
    const std::string TOPIC_PREFIX = "ros--topic--";

    //ROS Names
    const std::string PUBLISH_FUNCTION = "ros::Publisher::publish";
    const std::string SUBSCRIBE_FUNCTION = "ros::NodeHandle::subscribe";
    const std::string ADVERTISE_FUNCTION = "ros::NodeHandle::advertise";
    const std::string TIMER_FUNCTION = "ros::NodeHandle::createTimer";
    const std::string TIMER_PREFIX_1 = "ros::Duration(";
    const std::string TIMER_PREFIX_2 = "Duration(";

    //ROS Attributes
    const std::string ROS_TOPIC_BUF_SIZE = "bufferSize";
    const std::string ROS_NUM_ATTRIBUTES = "numAttributes";
    const std::string ROS_CALLBACK = "callbackFunc";
    const std::string ROS_PUB_TYPE = "publisherType";
    const std::string ROS_PUB_DATA = "pubData";
    const std::string TIMER_DURATION = "timerDuration";
    const std::string TIMER_ONESHOT = "isOneshot";
    const int PUB_MAX = 30;

    //Header Libraries
    const std::string STANDARD_IGNORE = "/ros/";
    std::vector<std::string> ignoreLibraries;

    //Graph Operations - Helpers
    static int generateTAModel(TAGraph* graph, std::string fileName);

    //System Headers - Helpers
    bool isInSystemHeader(const SourceManager& manager, SourceLocation loc);

    //Minimal Variables - Helpers
    void recordAssociations(const NamedDecl* assignee, const MemberExpr* assign, std::string type);
    void recordROSActionMinimal(const NamedDecl* decl, std::string type, bool pubEdge = true);
    const NamedDecl* getAssignee(const CXXOperatorCallExpr* parent);
    const MemberExpr* getAssignStmt(const CXXOperatorCallExpr* parent);
    const CXXRecordDecl* getParentClass(const NamedDecl* decl);

    //ROSHandlers - Helpers
    bool isClass(const CXXConstructExpr* ctor, std::string className);
    bool isFunction(const CallExpr* expr, std::string functionName);
    const NamedDecl* getParentAssign(const CXXConstructExpr* expr);
    RexNode* findCallbackFunction(std::string callbackQualified);
    std::vector<std::string> getArgs(const CallExpr* expr);
    std::string getPublisherType(const CallExpr* expr);
    NamedDecl* getParentVariable(const Expr* callExpr);

    //Variable Access - Helpers
    ParentWalker::AccessMethod determineAccess(bool lhs, BinaryOperator::Opcode opcode);
    ParentWalker::AccessMethod determineAccess(UnaryOperator::Opcode opcode);
    std::map<std::string, ParentWalker::AccessMethod> buildAccessMap(ParentWalker::AccessMethod prevAccess,
                                                                     const Expr* curExpr);
    void generateVarLinkage(std::map<std::string, ParentWalker::AccessMethod> lhs,
                            std::map<std::string, ParentWalker::AccessMethod> rhs);
};


#endif //REX_PARENTWALKER_H
