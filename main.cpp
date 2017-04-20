#include <iostream>
#include <boost/filesystem.hpp>
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include "llvm/Support/CommandLine.h"
#include "ROSWalker.h"

using namespace std;
using namespace clang::tooling;

static llvm::cl::OptionCategory RexCategory("Rex Options");

const string DEFAULT_FILENAME = "a.out";

static const string INCLUDE_DIR = "./include";
static const string INCLUDE_DIR_LOC = "--extra-arg=-I" + INCLUDE_DIR;

const char** prepareUpdatedArgs(int *argc, const char** argv){
    //Checks if the folder exists.
    boost::filesystem::path includePath(INCLUDE_DIR);
    if (!boost::filesystem::is_directory(includePath)){
        cerr << "Error: The include folders do not exist." << endl << "Clang cannot function without them." << endl;
        _exit(1);
    }

    //Increments argc.
    (*argc)++;

    //Copies over the first argument.
    char** updated = new char*[*argc];
    updated[0] = new char[strlen(argv[0])];
    strcpy(updated[0], argv[0]);

    //Adds in the include argument.
    updated[1] = new char[INCLUDE_DIR_LOC.size()];
    strcpy(updated[1], INCLUDE_DIR_LOC.c_str());

    //Next, loops through and copies.
    for (int i = 1; i < (*argc - 1); i++){
        updated[i + 1] = new char[strlen(argv[i])];
        strcpy(updated[i + 1], argv[i]);
    }

    return (const char**) updated;
}

int main(int argc, const char **argv) {
    //Processes the arguments.
    const char** updatedArgv = prepareUpdatedArgs(&argc, argv);

    //Runs the processor.
    CommonOptionsParser OptionsParser(argc, updatedArgv, RexCategory);
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