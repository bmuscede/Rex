//
// Created by bmuscede on 15/05/17.
//

#ifndef REX_REXHANDLER_H
#define REX_REXHANDLER_H

#include <string>
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include <llvm/Support/CommandLine.h>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;

class RexHandler {
public:
    RexHandler();
    ~RexHandler();

    int getNumGraphs();
    int getNumFiles();

    std::vector<std::string> getFiles();

    bool processClangToolCode(int argc, const char** argv);
    bool processAllFiles();
    bool processROSProject(std::string projName);

    bool outputIndividualModel(int modelNum, std::string fileName = std::string());
    bool outputAllModels(std::string baseFileName);

    int addByPath(path curPath);
    int removeByPath(path curPath);

private:
    const std::string DEFAULT_EXT = ".ta";
    const std::string DEFAULT_FILENAME = "out";
    const std::string DEFAULT_START = "./Rex";
    const std::string INCLUDE_DIR = "./include";
    const std::string INCLUDE_DIR_LOC = "--extra-arg=-I" + INCLUDE_DIR;

    const std::string C_FILE_EXT = ".c";
    const std::string CPLUS_FILE_EXT = ".cc";
    const std::string CPLUSPLUS_FILE_EXT = ".cpp";

    const int BASE_LEN = 2;

    std::vector<path> files;
    std::vector<std::string> ext;

    const char** prepareUpdatedArgs(int *argc, const char** argv);
    char** prepareArgs(int *argc);

    int addFile(path file);
    int addDirectory(path directory);

    int removeFile(path file);
    int removeDirectory(path directory);

};


#endif //REX_REXHANDLER_H
