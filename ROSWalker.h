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

using namespace llvm;
using namespace clang;
using namespace clang::tooling;

class ROSWalker : public RecursiveASTVisitor<ROSWalker> {
public:
    explicit ROSWalker(ASTContext *Context);

    //ASTWalker Functions
    bool VisitStmt(Stmt* statement);
    bool VisitFunctionDecl(FunctionDecl* decl);
    bool VisitCXXRecordDecl(CXXRecordDecl *Declaration);

private:
    ASTContext *Context;

    const std::string PUBLISH_FUNCTION = "ros::Publisher::publish";
    const std::string SUBSCRIBE_FUNCTION = "ros::Subscriber::subscribe";

    //ROS Detectors
    bool isPublish(const CallExpr* expr);
    bool isSubscribe(const CallExpr* expr);
    bool isFunction(const CallExpr* expr, std::string functionName);
    void recordPublish(const CallExpr* expr);
    void recordSubscribe(const CallExpr* expr);

    //Helper for Publishers
    std::vector<std::string> getArgs(const CallExpr* expr);

    //Helper Functions
    bool isInSystemHeader(const Stmt* statement);
    bool isInSystemHeader(const Decl* decl);
    bool isInSystemHeader(const SourceManager& manager, SourceLocation loc);
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
