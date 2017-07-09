//
// Created by bmuscede on 06/07/17.
//

#ifndef REX_PARENTWALKER_H
#define REX_PARENTWALKER_H

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

    static void deleteTAGraphs();
    static void deleteTAGraph(int num);
    static int getNumGraphs();
    static int endCurrentGraph();
    static int generateCurrentTAModel(std::string fileName);
    static int generateTAModel(int num, std::string fileName);
    static int generateAllTAModels(std::vector<std::string> fileName);

protected:
    static TAGraph* graph;
    static std::vector<TAGraph*> graphList;
    ASTContext *Context;

    //Helper Functions
    bool isInSystemHeader(const Stmt* statement);
    bool isInSystemHeader(const Decl* decl);

private:
    static int generateTAModel(TAGraph* graph, std::string fileName);

    bool isInSystemHeader(const SourceManager& manager, SourceLocation loc);
};


#endif //REX_PARENTWALKER_H
