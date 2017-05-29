//
// Created by bmuscede on 15/05/17.
//

#include <iostream>
#include <boost/filesystem.hpp>
#include "RexHandler.h"
#include "ROSWalker.h"

using namespace std;
using namespace clang::tooling;

RexHandler::RexHandler(){
    //Sets the C, C++ extensions.
    ext.push_back(C_FILE_EXT);
    ext.push_back(CPLUS_FILE_EXT);
    ext.push_back(CPLUSPLUS_FILE_EXT);
}

RexHandler::~RexHandler(){
    ROSWalker::deleteTAGraphs();
}

int RexHandler::getNumGraphs(){
    return ROSWalker::getNumGraphs();
}

int RexHandler::getNumFiles(){
    return (int) files.size();
}

vector<string> RexHandler::getFiles(){
    vector<string> strFiles;
    for (path file : files) strFiles.push_back(file.string());

    return strFiles;
}

bool RexHandler::processClangToolCode(int argc, const char** argv){
    bool success = true;
    llvm::cl::OptionCategory RexCategory("Rex Options");

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
    ROSWalker::endCurrentGraph();

    //Returns the success code.
    return success;
}

bool RexHandler::processAllFiles(){
    bool success = true;
    llvm::cl::OptionCategory RexCategory("Rex Options");

    //Creates the command line arguments.
    int argc = 0;
    char** argv = prepareArgs(&argc);

    //Runs the processor.
    CommonOptionsParser OptionsParser(argc, (const char**) argv, RexCategory);
    ClangTool Tool(OptionsParser.getCompilations(),
                   OptionsParser.getSourcePathList());
    int code = Tool.run(newFrontendActionFactory<ROSAction>().get());

    //Gets the code and checks for warnings.
    if (code != 0) {
        cerr << "Warning: Compilation errors were detected." << endl;
        success = false;
    }

    //Shifts the graphs.
    ROSWalker::endCurrentGraph();

    //Clears the graph.
    files.clear();

    //Returns the success code.
    return success;
}

bool RexHandler::processROSProject(std::string projName){
    return false;
}

bool RexHandler::outputIndividualModel(int modelNum, std::string fileName){
    if (fileName.compare(string()) == 0) fileName = DEFAULT_FILENAME;

    //First, check if the number if valid.
    if (modelNum < 0 || modelNum > getNumGraphs() - 1) return false;

    int succ = ROSWalker::generateTAModel(modelNum, fileName + DEFAULT_EXT);
    if (succ == 0) {
        cerr << "Error writing model to " << fileName << "!" << endl
                                                             << "Check the file and retry!" << endl;
        return false;
    }

    ROSWalker::deleteTAGraph(modelNum);
    return true;
}

bool RexHandler::outputAllModels(std::string baseFileName){
    bool succ = true;

    //Simply goes through and outputs.
    for (int i = 0; i < getNumGraphs(); i++){
        bool temp = outputIndividualModel(i, baseFileName + to_string(i));
        if (!temp) succ = false;
    }

    return succ;
}

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

char** RexHandler::prepareArgs(int *argc){
    //Sets argc.
    *argc = BASE_LEN + (int) files.size();

    //Next, argv.
    char** argv = new char*[*argc];

    //Copies the base start.
    argv[0] = new char[DEFAULT_START.size()];
    argv[1] = new char[INCLUDE_DIR_LOC.size()];

    //Next, moves them over.
    strcpy(argv[0], DEFAULT_START.c_str());
    strcpy(argv[1], INCLUDE_DIR_LOC.c_str());

    //Next, loops through the files and copies.
    for (int i = 0; i < files.size(); i++){
        argv[i + BASE_LEN] = new char[files.at(i).size()];
        strcpy(argv[i + BASE_LEN], files.at(i).c_str());
    }

    return argv;
}

int RexHandler::addFile(path file){
    //Gets the string that is added.
    files.push_back(canonical(file));
    return 1;
}

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

int RexHandler::removeFile(path file){
    file = canonical(file);

    //Check if the path exists.
    int i = 0;
    for (path curFile : files){
        if (curFile.compare(file.string()) == 0){
            //Remove from vector.
            files.erase(files.begin() + i);
            return 1;
        }
        i++;
    }

    return 0;
}

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