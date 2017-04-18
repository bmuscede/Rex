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
#include "TAGraph.h"

using namespace llvm;
using namespace clang;
using namespace clang::tooling;

class ROSWalker : public RecursiveASTVisitor<ROSWalker> {
public:
    explicit ROSWalker(ASTContext *Context);
    ~ROSWalker();

    //ASTWalker Functions
    bool VisitStmt(Stmt* statement);
    bool VisitFunctionDecl(FunctionDecl* decl);
    bool VisitCXXRecordDecl(CXXRecordDecl* decl);
    bool VisitVarDecl(VarDecl* decl);
    bool VisitFieldDecl(FieldDecl* decl);

    //TA Generator
    static void deleteTAGraph();
    static void flushTAGraph();
    static int generateTAModel(std::string fileName);

private:
    ASTContext *Context;
    static TAGraph* graph;

    const std::string PUBLISH_FUNCTION = "ros::Publisher::publish";
    const std::string SUBSCRIBE_FUNCTION = "ros::Subscriber::subscribe";
    const std::string PUBLISHER_CLASS = "ros::Publisher";
    const std::string SUBSCRIBER_CLASS = "ros::Subscriber";
    const std::string NODE_HANDLE_CLASS = "ros:NodeHandle";

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
    bool isFunction(const CallExpr* expr, std::string functionName);
    bool isClass(const CXXConstructExpr* ctor, std::string className);
    void recordPublish(const CallExpr* expr);
    void recordSubscribe(const CallExpr* expr);

    //Helper for Publishers
    std::vector<std::string> getArgs(const CallExpr* expr);

    //Helper Functions
    bool isInSystemHeader(const Stmt* statement);
    bool isInSystemHeader(const Decl* decl);
    bool isInSystemHeader(const SourceManager& manager, SourceLocation loc);

    //Secondary Helper Functions
    void addParentRelationship(const NamedDecl* baseDecl, std::string baseID);
    const FunctionDecl* getParentFunction(const Expr* callExpr);

    //Name Helper Functions
    std::string generateID(const FunctionDecl* decl);
    std::string generateID(const NamedDecl* decl);
    std::string generateName(const NamedDecl* decl);
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
