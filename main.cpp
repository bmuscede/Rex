#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include "llvm/Support/CommandLine.h"
#include "ROSWalker.h"

using namespace clang::tooling;

static llvm::cl::OptionCategory RexCategory("Rex Options");

int main(int argc, const char **argv) {
    CommonOptionsParser OptionsParser(argc, argv, RexCategory);
    ClangTool Tool(OptionsParser.getCompilations(),
                   OptionsParser.getSourcePathList());
    return Tool.run(newFrontendActionFactory<ROSAction>().get());
}