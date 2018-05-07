/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RexHandler.h
//
// Created By: Bryan J Muscedere
// Date: 15/05/17.
//
// Driver that connects to all the backend functions
// and coordinates them based on a user's commands
// from the master driver.
//
// Copyright (C) 2017, Bryan J. Muscedere
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef REX_REXHANDLER_H
#define REX_REXHANDLER_H

#include <string>
#include <vector>
#include "clang/Frontend/FrontendAction.h"
#include "clang/Tooling/Tooling.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include <llvm/Support/CommandLine.h>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;

class RexHandler {
public:
    /** Constructors/Destructors */
    RexHandler();
    ~RexHandler();

    /** Getters */
    int getNumGraphs();
    int getNumFiles();
    std::vector<std::string> getFiles();

    /** Processing Systems */
    bool processClangToolCode(int argc, const char** argv);
    bool processAllFiles(bool minimalWalk, bool lowMemory, int startNum = 0, std::string loadLoc = DEFAULT_LOAD);

    /** Recovery Systems */
    bool recoverCompact(std::string startDir);
    bool recoverFull(std::string startDir);

    /** Output Helpers */
    bool outputIndividualModel(int modelNum, std::string fileName = std::string());
    bool outputAllModels(std::string baseFileName);

    /** TA Helpers */
    bool resolveComponents(std::vector<path> databasePaths);
    bool processScenarioInformation(path scnPath, path rosPath);

    /** Add and Remove Functions */
    int addByPath(path curPath);
    int removeByPath(path curPath);
    int removeByRegex(std::string regex);

    /** Low Memory System */
    bool changeLowMemoryLoc(path curDir);

private:
    /** Default Arguments */
    const std::string DEFAULT_EXT = ".ta";
    const std::string DEFAULT_FILENAME = "out";
    const std::string DEFAULT_START = "./Rex";
    const std::string INCLUDE_DIR = "./include";
    const std::string INCLUDE_DIR_LOC = "--extra-arg=-I" + INCLUDE_DIR;
    static const std::string DEFAULT_LOAD;
    const int BASE_LEN = 2;
    const std::string COMPILATION_DB_NAME = "compile_commands.json";

    /** C/C++ Extensions */
    const std::string C_FILE_EXT = ".c";
    const std::string CPLUS_FILE_EXT = ".cc";
    const std::string CPLUSPLUS_FILE_EXT = ".cpp";

    /** Member Variables */
    std::vector<path> files;
    std::vector<std::string> ext;
    llvm::cl::OptionCategory RexCategory;
    path lowMemoryPath = "";
    bool recoveryMode = false;

    /** Arg Helper Methods */
    const char** prepareUpdatedArgs(int *argc, const char** argv);
    char** prepareArgs(int *argc);
    const std::vector<std::string> getFileList();

    /** Add/Remove Helper Methods */
    int addFile(path file);
    int addDirectory(path directory);
    int removeFile(path file);
    int removeDirectory(path directory);

    /** Recovery Helper */
    std::vector<int> getLMGraphs(std::string startDir);
    bool readSettings(std::string file, std::vector<std::string>* files, bool* minMode);
    int readStartNum(std::string file);

    /** Resolve Helper Methods */
    std::map<std::string, std::string> addDirectory(path directory, std::map<std::string, std::string> databases);
    std::map<std::string, std::vector<std::string>> resolveJSON(std::map<std::string, std::string> databases);

    /** Loading Helper Methods */
    std::vector<std::string> loadLibraries(std::string libs);

    /** Low Memory System */
    int extractIntegerWords(std::string str);
    std::vector<std::string> splitList(std::string list);
};

#endif //REX_REXHANDLER_H
