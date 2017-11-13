//
// Created by bmuscede on 06/07/17.
//

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

    enum ROSType {ROS_NONE, PUB, SUB};
    static TAGraph* graph;
    static std::vector<TAGraph*> graphList;
    ASTContext *Context;

    RexNode* currentSubscriber = nullptr;
    RexNode* currentPublisherOutdated = nullptr;
    RexNode* currentPublisher = nullptr;

    //ROS Names
    const std::string PUBLISHER_CLASS = "ros::Publisher";
    const std::string SUBSCRIBER_CLASS = "ros::Subscriber";
    const std::string NODE_HANDLE_CLASS = "ros::NodeHandle";

    //Minimal Handlers
    ParentWalker::ROSType handleMinimalStmt(Stmt* statement);
    bool handleMinimalVarDecl(VarDecl* decl, bool pubEdge = true);
    bool handleMinimalFieldDecl(FieldDecl* decl, bool pubEdge = true);

    //ROS Handlers
    bool isNodeHandlerObj(const CXXConstructExpr* ctor);
    bool isSubscriberObj(const CXXConstructExpr* ctor);
    bool isPublisherObj(const CXXConstructExpr* ctor);
    bool isPublish(const CallExpr* expr);
    bool isSubscribe(const CallExpr* expr);
    bool isAdvertise(const CallExpr* expr);

    //System Headers
    bool isInSystemHeader(const Stmt* statement);
    bool isInSystemHeader(const Decl* decl);

    //ROS Recorders
    void recordParentSubscribe(const CXXConstructExpr* expr);
    void recordParentPublish(const CXXConstructExpr* expr);
    void recordParentGeneric(std::string parentID, std::string parentName, RexNode::NodeType type);
    void recordParentNodeHandle(const CXXConstructExpr* expr);
    void recordNodeHandle(const CXXConstructExpr* expr);
    void recordSubscribe(const CallExpr* expr);
    void recordPublish(const CallExpr* expr);
    void recordAdvertise(const CallExpr* expr);
    void recordTopic(std::string name);

    //Name Helper Functions
    std::string generateID(const NamedDecl* decl);
    std::string generateName(const NamedDecl* decl);
    std::string validateStringArg(std::string name);
    std::string generateFileName(const NamedDecl* decl);
    void recordParentClassLoc(const FunctionDecl* decl);

    //Variable Access Methods
    enum AccessMethod {NONE, BOTH, READ, WRITE};
    std::map<std::string, ParentWalker::AccessMethod> getAccessType(const BinaryOperator* op);
    std::map<std::string, ParentWalker::AccessMethod> getAccessType(const UnaryOperator* op);

private:
    //ROS Topic Name
    const std::string TOPIC_PREFIX = "ros--topic--";

    //ROS Names
    const std::string PUBLISH_FUNCTION = "ros::Publisher::publish";
    const std::string SUBSCRIBE_FUNCTION = "ros::NodeHandle::subscribe";
    const std::string ADVERTISE_FUNCTION = "ros::NodeHandle::advertise";

    //ROS Attributes
    const std::string ROS_SUB_VAR_FLAG = "isSubscriber";
    const std::string ROS_PUB_VAR_FLAG = "isPublisher";
    const std::string ROS_TOPIC_BUF_SIZE = "bufferSize";
    const std::string ROS_NUM_ATTRIBUTES = "numAttributes";
    const std::string ROS_CALLBACK = "callbackFunc";
    const std::string ROS_PUB_TYPE = "publisherType";
    const std::string ROS_NUMBER = "rosNumber";
    const std::string ROS_PUB_DATA = "pubData";
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
};


#endif //REX_PARENTWALKER_H
