#include <iostream>
#include "clang/Frontend/FrontendActions.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include "clang/Tooling/Tooling.h"
#include "clang/ASTMatchers/ASTMatchers.h"
#include "clang/ASTMatchers/ASTMatchFinder.h"
#include "ROSWalker.h"

using namespace std;
using namespace clang;
using namespace clang::tooling;
using namespace clang::ast_matchers;
using namespace llvm;

static llvm::cl::OptionCategory RexCategory("Rex Options");

int main(int argc, const char** argv) {
    //Generates a ClangTool that will run and perform the extraction.
    CommonOptionsParser OptionsParser(argc, argv, RexCategory);
    ClangTool Tool(OptionsParser.getCompilations(), OptionsParser.getSourcePathList());

    //Creates a ROSWalker.
    ROSWalker* walker = new ROSWalker();

    //Generates the matchers.
    MatchFinder finder;
    walker->setMatchers(&finder);
    return Tool.run(newFrontendActionFactory(&finder).get());
}