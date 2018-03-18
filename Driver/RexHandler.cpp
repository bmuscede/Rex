/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RexHandler.cpp
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

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <regex>
#include <fstream>
#include "RexHandler.h"
#include "../Walker/ROSConsumer.h"
#include "../JSON/json.h"
#include "../Configuration/ScenarioWalker.h"

using namespace std;
using namespace clang::tooling;

const string RexHandler::DEFAULT_LOAD = "REX_IGNORE.db";

/**
 * Constructor that prepares the RexHandler.
 */
RexHandler::RexHandler() : RexCategory("Rex Options"){
    //Sets the C, C++ extensions.
    ext.push_back(C_FILE_EXT);
    ext.push_back(CPLUS_FILE_EXT);
    ext.push_back(CPLUSPLUS_FILE_EXT);
}

/** Deletes all the TA graphs */
RexHandler::~RexHandler(){
    ParentWalker::deleteTAGraphs();
}

/**
 * Gets the number of already generated graphs.
 * @return Number of generated graphs.
 */
int RexHandler::getNumGraphs(){
    return ParentWalker::getNumGraphs();
}

/**
 * Number of files currently in the queue.
 * @return The size of the queue.
 */
int RexHandler::getNumFiles(){
    return (int) files.size();
}

/**
 * Gets a list of all the files.
 * @return A vector of all the files.
 */
vector<string> RexHandler::getFiles(){
    vector<string> strFiles;
    for (path file : files) strFiles.push_back(canonical(file).string());

    return strFiles;
}

/**
 * General method that processes the Clangtool code.
 * @param argc Argc command.
 * @param argv Argv command.
 * @return Returns a boolean indicating success.
 */
bool RexHandler::processClangToolCode(int argc, const char** argv){
    bool success = true;

    //Processes the arguments.
    const char** updatedArgv = prepareUpdatedArgs(&argc, argv);

    //Runs the processor.
    CommonOptionsParser OptionsParser(argc, updatedArgv, RexCategory);
    ClangTool Tool(OptionsParser.getCompilations(),
                   OptionsParser.getSourcePathList());
    int code = Tool.run(newFrontendActionFactory<ROSAction>().get());

    //Gets the code and checks for warnings.
    if (code != 0) {
        cerr << "Warning: Compilation errors were detected." << endl;
        success = false;
    }

    //Shifts the graphs.
    ParentWalker::endCurrentGraph();

    //Returns the success code.
    return success;
}

/**
 * Runs through all files in the queue and generates a graph.
 * @param minimalWalk Boolean that indicates what ROSWalker to use.
 * @param loadLoc The location to load the libraries to ignore (OPTIONAL).
 * @return Boolean indicating success.
 */
bool RexHandler::processAllFiles(bool minimalWalk, string loadLoc){
    bool success = true;
    //Creates the command line arguments.
    int argc = 0;
    char** argv = prepareArgs(&argc);

    //Gets the list of files.
    const vector<string> fileList = getFileList();

    //Sets up the processor.
    CommonOptionsParser OptionsParser(argc, (const char**) argv, RexCategory);
    ClangTool Tool(OptionsParser.getCompilations(), fileList);

    //Runs the processor.
    if (minimalWalk){
        ROSConsumer::setMode(ROSConsumer::MINIMAL);
    } else {
        ROSConsumer::setMode(ROSConsumer::FULL);

        //Loads in the Rex libraries to ignore.
        vector<string> rexLibraries = loadLibraries(loadLoc);
        ROSConsumer::setLibrariesToIgnore(rexLibraries);
    }

    int code = Tool.run(newFrontendActionFactory<ROSAction>().get());

    //Gets the code and checks for warnings.
    if (code != 0) {
        cerr << "Warning: Compilation errors were detected." << endl;
        success = false;
    }

    //Shifts the graphs.
    ParentWalker::endCurrentGraph();

    //Clears the graph.
    files.clear();

    //Returns the success code.
    return success;
}

/**
 * Outputs an individual TA model to TA format.
 * @param modelNum The number of the model to output.
 * @param fileName The filename to output as.
 * @return Boolean indicating success.
 */
bool RexHandler::outputIndividualModel(int modelNum, std::string fileName){
    if (fileName.compare(string()) == 0) fileName = DEFAULT_FILENAME;

    //First, check if the number if valid.
    if (modelNum < 0 || modelNum > getNumGraphs() - 1) return false;

    int succ = ParentWalker::generateTAModel(modelNum, fileName + DEFAULT_EXT);
    if (succ == 0) {
        cerr << "Error writing to " << fileName << "!" << endl
                                                             << "Check the file and retry!" << endl;
        return false;
    }

    ParentWalker::deleteTAGraph(modelNum);
    return true;
}

/**
 * Outputs all models generated based on a file name.
 * @param baseFileName The base file name to output on.
 * @return A boolean indicating success.
 */
bool RexHandler::outputAllModels(std::string baseFileName){
    bool succ = true;

    //Simply goes through and outputs.
    for (int i = 0; i < getNumGraphs(); i++){
        bool temp = outputIndividualModel(i, baseFileName + to_string(i));
        if (!temp) succ = false;
    }

    return succ;
}

/**
 * Resolves the components in the model.
 * @param databasePaths The paths to potential compilation databases.
 * @return A boolean indicating success.
 */
bool RexHandler::resolveComponents(std::vector<path> databasePaths){
    map<string, string> databaseMap;

    //First, from the path we resolve all compilation databases.
    for (path curPath : databasePaths){
        databaseMap = addDirectory(curPath, databaseMap);
    }

    //Count the number of keys.
    cout << databaseMap.size() << " compilation databases were detected!" << endl;
    if (databaseMap.size() == 0) return false;
    cout << "Now resolving..." << endl;

    //Go through each JSON file.
    map<string, vector<string>> results = resolveJSON(databaseMap);

    //Goes through each of the graphs.
    return ParentWalker::resolveAllTAModels(results);
}

bool RexHandler::processScenarioInformation(path scnPath, path rosPath){
    //Create a scenario walker instance.
    ScenarioWalker* walker = new ScenarioWalker(scnPath, rosPath);

    bool res = walker->readFiles();
    if (!res) return false;
    res = walker->processScenario();
    if (!res) return false;

    vector<string> activePackages = walker->getActivePackages();
    cout << "The following packages are active in this scenario:" << endl;
    for (string pkg : activePackages) {
        cout << " - " << pkg << endl;
    }

    //Next, resolve graph information.
    ParentWalker::onlyKeepFeatures(activePackages);
    return true;
}

/**
 * Adds a file/directory by path.
 * @param curPath The path to add.
 * @return Integer indicating the number of files added.
 */
int RexHandler::addByPath(path curPath){
    int num = 0;

    //Determines what the path is.
    if (is_directory(curPath)){
        num = addDirectory(curPath);
    } else {
        num = addFile(curPath);
    }

    return num;
}

/**
 * Removes a file/directory from the queue.
 * @param curPath The path to remove.
 * @return The number of files removed.
 */
int RexHandler::removeByPath(path curPath){
    int num = 0;

    //Determines what the path is.
    if (is_directory(curPath)){
        num = removeDirectory(curPath);
    } else {
        num = removeFile(curPath);
    }

    return num;
}

/**
 * Removes files and directories from the queue by regular expression.
 * @param regex The regular expression string to apply.
 * @return The number of files removed.
 */
int RexHandler::removeByRegex(string regex) {
    int num = 0;
    std::regex rregex(regex);

    //Iterates through the file.
    vector<path>::iterator it;
    for (it = files.begin(); it != files.end();){
        string path = it->string();

        //Checks the regex.
        if (regex_match(path, rregex)){
            it = files.erase(it);
            num++;
        } else {
            it++;
        }
    }

    return num;
}

/**
 * Generates arguments with the include directory.
 * @param argc The argc to update.
 * @param argv The previous argv.
 * @return The new argv.
 */
const char** RexHandler::prepareUpdatedArgs(int *argc, const char** argv){
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

/**
 * Generates an argv array based on the files in the queue and Clang's input format.
 * @param argc The number of tokens.
 * @return The new argv command.
 */
char** RexHandler::prepareArgs(int *argc){
    int size = BASE_LEN + (int) files.size();

    //Sets argc.
    *argc = size;

    //Next, argv.
    char** argv = new char*[size];

    //Copies the base start.
    argv[0] = new char[DEFAULT_START.size() + 1];
    argv[1] = new char[INCLUDE_DIR_LOC.size() + 1];

    //Next, moves them over.
    strcpy(argv[0], DEFAULT_START.c_str());
    strcpy(argv[1], INCLUDE_DIR_LOC.c_str());

    //Next, loops through the files and copies.
    for (int i = 0; i < files.size(); i++){
        string curFile = canonical(files.at(i)).string();
        argv[i + BASE_LEN] = new char[curFile.size() + 1];
        strcpy(argv[i + BASE_LEN], curFile.c_str());
    }

    return argv;
}

/**
 * Simply converts the path vector into a string
 * vector.
 * @return A string vector of files.
 */
const vector<string> RexHandler::getFileList(){
    vector<string> results;
    for (path curItem : files){
        results.push_back(canonical(curItem).string());
    }

    return results;
}

/**
 * Adds a file to the queue.
 * @param file The file to add.
 * @return Returns 1.
 */
int RexHandler::addFile(path file){
    //Gets the string that is added.
    files.push_back(file);
    return 1;
}

/**
 * Recursively adds a directory to the queue.
 * @param directory The directory to add.
 * @return The number of files added.
 */
int RexHandler::addDirectory(path directory){
    int numAdded = 0;
    vector<path> interiorDir = vector<path>();
    directory_iterator endIter;

    //Start by iterating through and inspecting each file.
    for (directory_iterator iter(directory); iter != endIter; iter++){
        //Check what the current file is.
        if (is_regular_file(iter->path())){
            //Check the extension.
            string extFile = extension(iter->path());

            //Iterates through the extension vector.
            for (int i = 0; i < ext.size(); i++){
                //Checks the file.
                if (extFile.compare(ext.at(i)) == 0){
                    numAdded += addFile(iter->path());
                }
            }
        } else if (is_directory(iter->path())){
            //Add the directory to the search system.
            interiorDir.push_back(iter->path());
        }
    }

    //Next, goes to all the internal directories.
    for (path cur : interiorDir){
        numAdded += addDirectory(cur);
    }

    return numAdded;
}

/**
 * Removes a file from the queue.
 * @param file The file to remove.
 * @return 1 if the file is removed, 0 if it wasn't found.
 */
int RexHandler::removeFile(path file){
    file = canonical(file);

    //Check if the path exists.
    int i = 0;
    for (path curFile : files){
        curFile = canonical(curFile);
        if (curFile.compare(file.string()) == 0){
            //Remove from vector.
            files.erase(files.begin() + i);
            return 1;
        }
        i++;
    }

    return 0;
}

/**
 * Removes a directory from the queue.
 * @param directory The directory to remove.
 * @return The number removed.
 */
int RexHandler::removeDirectory(path directory){
    int numRemoved = 0;
    vector<path> interiorDir = vector<path>();
    directory_iterator endIter;

    //Start by iterating through and inspecting each file.
    for (directory_iterator iter(directory); iter != endIter; iter++){
        //Check what the current file is.
        if (is_regular_file(iter->path())){
            //Check the extension.
            string extFile = extension(iter->path());

            //Iterates through the extension vector.
            for (int i = 0; i < ext.size(); i++){
                //Checks the file.
                if (extFile.compare(ext.at(i)) == 0){
                    numRemoved += removeFile(iter->path());
                }
            }
        } else if (is_directory(iter->path())){
            //Add the directory to the search system.
            interiorDir.push_back(iter->path());
        }
    }

    //Next, goes to all the internal directories.
    for (path cur : interiorDir){
        numRemoved += removeDirectory(cur);
    }

    return numRemoved;
}

/**
 * Adds a directory to the queue.
 * @param directory The directory to add.
 * @param databases A collection of databases.
 * @return A map of databases per file.
 */
map<string, string> RexHandler::addDirectory(path directory, map<string, string> databases){
    vector<path> interiorDir = vector<path>();
    directory_iterator endIter;

    //Start by iterating through and inspecting each file.
    for (directory_iterator iter(directory); iter != endIter; iter++){
        //Check what the current file is.
        if (is_regular_file(iter->path())){
            //Check the file name.
            string fn = iter->path().filename().string();

            //Adds if its a compilation database.
            if (fn.compare(COMPILATION_DB_NAME) == 0){
                databases[directory.string()] = canonical(iter->path()).string();
            }
        } else if (is_directory(iter->path())){
            //Add the directory to the search system.
            interiorDir.push_back(iter->path());
        }
    }

    //Next, goes to all the internal directories.
    for (path cur : interiorDir){
        databases = addDirectory(cur, databases);
    }

    return databases;
}

/**
 * Generates a collection of file to components.
 * @param databases The list of JSON databases.
 * @return A mapping from file to component name.
 */
map<string, vector<string>> RexHandler::resolveJSON(map<string, string> databases){
    map<string, vector<string>> resultMap;

    //Loop through the list of databases.
    for (auto entry : databases){
        //Gets the JSON objects.
        Json::Value root;

        //Load the JSON file.
        std::ifstream jsonFile(entry.second, std::ifstream::binary);
        jsonFile >> root;
        if (root.isNull()) continue;

        //Now, we loop through each entry.
        bool loop = true;
        int i = 0;
        while (loop){
            Json::Value node = root[i];
            if (node.isNull()){
                loop = false;
                continue;
            }

            //Get the file.
            string filename = canonical(path(node["file"].asString())).string();
            resultMap[filename].push_back(entry.first);

            i++;
        }
        jsonFile.close();
    }

    return resultMap;
};

/**
 * Adds libraries from the current supplied file and tells
 * the system to ignore them.
 * @param libs The file of the ignore file.
 * @return A vector of library paths to ignore.
 */
vector<string> RexHandler::loadLibraries(string libs){
    vector<string> output;
    string line;

    std::ifstream inputFile(libs);
    while(getline(inputFile, line)){
        boost::trim(line);
        if (line[0] == '#' || line.length() == 0) continue;

        output.push_back(line);
    }

    return output;
}
