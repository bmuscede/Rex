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
    bool processAllFiles();

    /** Output Helpers */
    bool outputIndividualModel(int modelNum, std::string fileName = std::string());
    bool outputAllModels(std::string baseFileName);

    /** Add and Remove Functions */
    int addByPath(path curPath);
    int removeByPath(path curPath);
    int removeByRegex(std::string regex);

private:
    /** Default Arguments */
    const std::string DEFAULT_EXT = ".ta";
    const std::string DEFAULT_FILENAME = "out";
    const std::string DEFAULT_START = "./Rex";
    const std::string INCLUDE_DIR = "./include";
    const std::string INCLUDE_DIR_LOC = "--extra-arg=-I" + INCLUDE_DIR;
    const int BASE_LEN = 2;

    /** C/C++ Extensions */
    const std::string C_FILE_EXT = ".c";
    const std::string CPLUS_FILE_EXT = ".cc";
    const std::string CPLUSPLUS_FILE_EXT = ".cpp";

    /** Member Variables */
    std::vector<path> files;
    std::vector<std::string> ext;

    /** Arg Helper Methods */
    const char** prepareUpdatedArgs(int *argc, const char** argv);
    char** prepareArgs(int *argc);

    /** Add/Remove Helper Methods */
    int addFile(path file);
    int addDirectory(path directory);
    int removeFile(path file);
    int removeDirectory(path directory);

};

#endif //REX_REXHANDLER_H
