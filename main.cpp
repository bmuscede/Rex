#include <iostream>
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include "llvm/Support/CommandLine.h"
#include "ROSWalker.h"

using namespace std;
using namespace clang::tooling;

static llvm::cl::OptionCategory RexCategory("Rex Options");

const string DEFAULT_FILENAME = "a.out";

int main(int argc, const char **argv) {
    //Runs the processor.
    CommonOptionsParser OptionsParser(argc, argv, RexCategory);
    ClangTool Tool(OptionsParser.getCompilations(),
                   OptionsParser.getSourcePathList());
    int code = Tool.run(newFrontendActionFactory<ROSAction>().get());

    //Gets the code and checks for warnings.
    if (code != 0) cerr << "Warning: Compilation errors were detected." << endl;

    //Generates the TA file.
    int ret = ROSWalker::generateTAModel(DEFAULT_FILENAME);

    //Removes the TA graph to restore memory.
    ROSWalker::deleteTAGraph();
    return ret;
}