//
// Created by bmuscede on 07/04/17.
//

#ifndef REX_ROSWALKER_H
#define REX_ROSWALKER_H

#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include "llvm/Support/CommandLine.h"
#include "../Graph/TAGraph.h"

using namespace llvm;
using namespace clang;
using namespace clang::tooling;

class ROSWalker : public RecursiveASTVisitor<ROSWalker> {
public:
    explicit ROSWalker(ASTContext *Context);
    ~ROSWalker();

    //TA Generator
    static void deleteTAGraphs();
    static void deleteTAGraph(int num);
    static int getNumGraphs();
    static int endCurrentGraph();
    static int generateCurrentTAModel(std::string fileName);
    static int generateTAModel(int num, std::string fileName);
    static int generateAllTAModels(std::vector<std::string> fileName);

    //ASTWalker Functions
    bool VisitStmt(Stmt* statement);
    bool VisitFunctionDecl(FunctionDecl* decl);
    bool VisitCXXRecordDecl(CXXRecordDecl* decl);
    bool VisitVarDecl(VarDecl* decl);
    bool VisitFieldDecl(FieldDecl* decl);

protected:
    static int generateTAModel(TAGraph* graph, std::string fileName);

private:
    ASTContext *Context;
    static TAGraph* graph;
    static std::vector<TAGraph*> graphList;

    RexNode* currentSubscriber = nullptr;
    RexNode* currentPublisher = nullptr;

    const std::string TOPIC_PREFIX = "ros--topic--";

    const std::string PUBLISH_FUNCTION = "ros::Publisher::publish";
    const std::string SUBSCRIBE_FUNCTION = "ros::NodeHandle::subscribe";
    const std::string ADVERTISE_FUNCTION = "ros::NodeHandle::advertise";
    const std::string PUBLISHER_CLASS = "ros::Publisher";
    const std::string SUBSCRIBER_CLASS = "ros::Subscriber";
    const std::string NODE_HANDLE_CLASS = "ros::NodeHandle";

    //ROS Attribute Names.
    const std::string ROS_TOPIC_BUF_SIZE = "bufferSize";
    const std::string ROS_NUM_ATTRIBUTES = "numAttributes";
    const std::string ROS_SUB_VAR_FLAG = "isSubscriber";
    const std::string ROS_PUB_VAR_FLAG = "isPublisher";
    const std::string ROS_CALLBACK = "callbackFunc";
    const std::string ROS_PUB_TYPE = "publisherType";
    const std::string ROS_NUMBER = "rosNumber";
    const std::string ROS_PUB_DATA = "pubData";
    const int PUB_MAX = 30;

    //C++ Detectors
    void recordFunctionDecl(const FunctionDecl* decl);
    void recordClassDecl(const CXXRecordDecl* decl);
    void recordVarDecl(const VarDecl* decl);
    void recordFieldDecl(const FieldDecl* decl);

    //Expr Recorders
    void recordCallExpr(const CallExpr* expr);
    void recordVarUsage(const DeclRefExpr* expr);

    //ROS Detectors
    bool isNodeHandlerObj(const CXXConstructExpr* ctor);
    bool isSubscriberObj(const CXXConstructExpr* ctor);
    bool isPublisherObj(const CXXConstructExpr* ctor);
    bool isPublish(const CallExpr* expr);
    bool isSubscribe(const CallExpr* expr);
    bool isAdvertise(const CallExpr* expr);
    bool isFunction(const CallExpr* expr, std::string functionName);
    bool isClass(const CXXConstructExpr* ctor, std::string className);

    //ROS Recorders
    void recordParentSubscribe(const CXXConstructExpr* expr);
    void recordParentPublish(const CXXConstructExpr* expr);
    void recordParentNodeHandle(const CXXConstructExpr* expr);
    void recordTopic(std::string name);
    void recordNodeHandle(const CXXConstructExpr* expr);
    void recordSubscribe(const CallExpr* expr);
    void recordPublish(const CallExpr* expr);
    void recordAdvertise(const CallExpr* expr);

    //Helpers for ROS
    RexNode* findCallbackFunction(std::string callbackQualified);
    std::vector<std::string> getArgs(const CallExpr* expr);
    std::string getPublisherType(const CallExpr* expr);

    //Helper Functions
    bool isInSystemHeader(const Stmt* statement);
    bool isInSystemHeader(const Decl* decl);
    bool isInSystemHeader(const SourceManager& manager, SourceLocation loc);

    //Secondary Helper Functions
    void addParentRelationship(const NamedDecl* baseDecl, std::string baseID);
    const FunctionDecl* getParentFunction(const Expr* callExpr);
    std::string getParentVariable(const Expr* callExpr);
    const NamedDecl* getParentAssign(const CXXConstructExpr* expr);

    //Name Helper Functions
    std::string generateID(const FunctionDecl* decl);
    std::string generateID(const NamedDecl* decl);
    std::string generateName(const NamedDecl* decl);
    std::string validateStringArg(std::string name);
};

class ROSConsumer : public ASTConsumer {
public:
    explicit ROSConsumer(ASTContext *Context);
    virtual void HandleTranslationUnit(ASTContext &Context);

private:
    ROSWalker Visitor;
};

class ROSAction : public ASTFrontendAction {
public:
    virtual std::unique_ptr<ASTConsumer> CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile);
};

#endif //REX_ROSWALKER_H
