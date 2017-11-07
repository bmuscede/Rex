/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RexMaster.cpp
//
// Created By: Bryan J Muscedere
// Date: 13/05/17.
//
// Driver source code for the Rex program. Handles
// commands and passes it off to the RexHandler to
// translate it into a Rex action. Also handles
// errors gracefully.
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

#include <fstream>
#include <iostream>
#include <pwd.h>
#include <zconf.h>
#include <vector>
#include <boost/algorithm/string/regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/make_shared.hpp>
#include "RexHandler.h"

using namespace std;
using namespace boost::filesystem;
namespace po = boost::program_options;

/** Commands */
const static string HELP_ARG = "help";
const static string ABOUT_ARG = "about";
const static string EXIT_ARG = "quit";
const static string GEN_ARG = "generate";
const static string OUT_ARG = "output";
const static string ADD_ARG = "add";
const static string REMOVE_ARG = "remove";
const static string LIST_ARG = "list";
const static string SCRIPT_ARG = "script";

/** Rex Command Handler */
static RexHandler* masterHandle;

struct RexHelp {
    RexHelp(const std::string& name = {}, const po::options_description& desc = {})
            : name(name), desc(boost::make_shared<po::options_description>(desc)) {}

    RexHelp& operator=(const RexHelp& other) = default;

    boost::shared_ptr<po::options_description> desc;

private:
    std::string name;
};

/** Forward Declarations */
bool processCommand(string line);

/** Static Variables */
static map<string, RexHelp> helpInfo;
static map<string ,string> helpStrings;

/**
 * Takes in a line and tokenizes it to
 * a vector by spaces.
 * @param line The line to tokenize.
 * @return A vector of words.
 */
vector<string> tokenizeBySpace(string line){
    vector<string> result;
    istringstream iss(line);
    for(std::string s; iss >> s;)
        result.push_back(s);

    return result;
}

/**
 * From a collection of tokens, creates
 * a char** array of the tokens to mimic
 * a typical argv array.
 * @param tokens The tokens to convert to argv.
 * @return A char** array that mimics an argv array.
 */
char** createArgv(vector<string> tokens){
    //First, create the array.
    char** tokenC = new char*[(int) tokens.size()];
    for (int i = 0; i < tokens.size(); i++){
        tokenC[i] = new char[(int) tokens.at(i).size() + 1];
        strcpy(tokenC[i], tokens.at(i).c_str());
    }

    return tokenC;
}

/**
 * Helper method that generates a Y/N command line prompt
 * to ask a user if they want to do something.
 * @param promptText The text to display to ask the user.
 * @return Whether the user said yes (true) or no (false).
 */
bool promptForAction(string promptText){
    string result;

    bool loop = true;
    while (loop) {
        cout << promptText;
        getline(cin, result);

        //Checks the value.
        if (result.compare("Y") == 0 || result.compare("y") == 0){
            loop = false;
        } else if (result.compare("N") == 0 || result.compare("n") == 0){
            return false;
        } else {
            cout << "Invalid entry. Please type \'Y\' or \'N\'!" << endl;
        }
    }

    return true;
}

/**
 * Generates all the help messages for the system.
 * @param helpMap Map of all the program options.
 * @param helpString Map of all the help strings.
 */
void generateHelp(map<string, RexHelp>* helpMap, map<string, string>* helpString){
    //First, generate the help information for about.
    (*helpString)[ABOUT_ARG] = string("About Help\nUsage: " + ABOUT_ARG + "\n" "Prints information about the program including\n"
            "license information and program details.\n");

    //Next, generate the help information for exit.
    (*helpString)[EXIT_ARG] = string("Quit Help\nUsage: " + EXIT_ARG + "(!)\n" "Quits the program and returns back"
            " to the terminal.\nTyping " + EXIT_ARG + "will only quit if there are no items left to be processed.\n"
            "Typing " + EXIT_ARG + "! will quit the program automatically without any warning.\n");

    //Generate the help for generate.
    (*helpMap)[GEN_ARG] = RexHelp(GEN_ARG, po::options_description("Options"));
    helpMap->at(GEN_ARG).desc->add_options()
            ("help,h", "Print help message for generate.")
            ("minimal,m", "Runs a minimal analysis of the ROS files. Looks for ROS information.");
    stringstream ss;
    ss << *helpMap->at(GEN_ARG).desc;
    (*helpString)[GEN_ARG] = string("Generate Help\nUsage: " + GEN_ARG + "\nGenerates a graph based on the supplied"
            " C/C++ source files.\nYou must have at least 1 source file in the queue for the graph to be generated.\n"
            "Additionally, in the root directory, there must a \"compile_commands.json\" file.\n\n" + ss.str());

    //Generate the help for output.
    (*helpMap)[OUT_ARG] = RexHelp(OUT_ARG, po::options_description("Options"));
    helpMap->at(OUT_ARG).desc->add_options()
            ("help,h", "Print help message for output.")
            ("select,s", po::value<string>(), "Outputs graphs by graph number. Can separate values by comma.")
            ("outputFile", po::value<std::vector<std::string>>(), "The base file name to save.");
    ss.str(string());
    ss << *helpMap->at(OUT_ARG).desc;
    (*helpString)[OUT_ARG] = string("Output Help\nUsage: " + OUT_ARG + " [options] OutputName\nOutputs the generated"
            " graphs to a tuple-attribute (TA) file based on a specific\nRex schema. These models can then be used"
            " by other programs.\n\n" + ss.str());

    //Generate the help for add.
    (*helpMap)[ADD_ARG] = RexHelp(ADD_ARG, po::options_description("Options"));
    helpMap->at(ADD_ARG).desc->add_options()
            ("help,h", "Print help message for add.")
            ("source,s", po::value<std::vector<std::string>>(), "A file or directory to add to the current graph.");
    ss.str(string());
    ss << *helpMap->at(ADD_ARG).desc;
    (*helpString)[ADD_ARG] = string("Add Help\nUsage: " + ADD_ARG + " source\nAdds files or directories to process."
            "By adding directories, Rex will recursively\nsearch for source files starting from the root. For"
            " files\nyou can specify any file you want and Rex will add it to the project.\n\n" + ss.str());

    //Generate the help for remove.
    (*helpMap)[REMOVE_ARG] = RexHelp(REMOVE_ARG, po::options_description("Options"));
    helpMap->at(REMOVE_ARG).desc->add_options()
            ("help,h", "Print help message for add.")
            ("regex,r", po::value<string>(), "Regular expression to process.")
            ("source,s", po::value<std::vector<std::string>>(), "A file or directory to remove from the current graph.");
    ss.str(string());
    ss << *helpMap->at(REMOVE_ARG).desc;
    (*helpString)[REMOVE_ARG] = string("Remove Help\nUsage: " + REMOVE_ARG + " source\nRemoves files or directories "
            "from the processing queue. By removing directories, Rex will recursively\nsearch for files in the queue"
            " that can be removed. Individual files can also be removed\ntoo. Only files that are in the queue to"
            " begin with can be removed.\n\n" + ss.str());

    //Generates the help for list.
    (*helpMap)[LIST_ARG] = RexHelp(LIST_ARG, po::options_description("Options"));
    helpMap->at(LIST_ARG).desc->add_options()
            ("help,h", "Print help message for list.")
            ("num-graphs,g", "Prints the number of graphs already generated.")
            ("files,f", "Lists all the files for the current input.");
    ss.str(string());
    ss << *helpMap->at(LIST_ARG).desc;
    (*helpString)[LIST_ARG] = string("List Help\nUsage: " + LIST_ARG + " [options]\nLists information about the "
            "current state of Rex. This includes the number\nof graphs currently generated and all the files\nbeing"
            " processed for the next graph.\nOnly files that are in the queue are listed.\n\n" + ss.str());

    //Generates the help for script.
    (*helpMap)[SCRIPT_ARG] = RexHelp(SCRIPT_ARG, po::options_description("Options"));
    helpMap->at(SCRIPT_ARG).desc->add_options()
            ("help,h", "Print help message for list.")
            ("script,s", po::value<std::string>(), "The script file to run.");
    ss.str(string());
    ss << *helpMap->at(SCRIPT_ARG).desc;
    (*helpString)[SCRIPT_ARG] = string("Script Help\nUsage: " + SCRIPT_ARG + " script\nRuns a script that can "
            "handle any of the commands in this program automatically.\nThe script will terminate when it reaches"
            " the end of the script or hits quit.\n\n" + ss.str());
}

/**
 * Prints a simple help message.
 * Also lets users print information about commands.
 * @param line The line to process.
 * @param messages The help messages.
 */
void handleHelp(string line, map<string, string> messages) {
    string helpString = "Commands that can be used:\n"
        "help           : Prints help information.\n"
        "about          : Prints about information.\n"
        "quit(!)        : Quits the program.\n"
        "generate       : Generates a graph based on the current source files.\n"
        "output         : Outputs a graph to tuple-attribute format.\n"
        "add            : Adds a file/directory to be processed.\n"
        "remove         : Removes a file/directory from the queue.\n"
        "list           : Lists the current state of Rex.\n"
        "script         : Runs a script that handles program commands.\n\n"
        "For more help type \"help [argument name]\" for more details.";

    auto tokens = tokenizeBySpace(line);

    //Check if the line is empty.
    if (tokens.size() == 1){
        cout << helpString << endl;
    } else if (tokens.size() == 2){
        //Check if the key exists.
        if (messages.find(tokens.at(1)) == messages.end()){
            cerr << "The token \"" + tokens.at(1) + "\" is not a valid command!" << endl << endl
                    << helpString << endl;
        } else {
            cout << messages.at(tokens.at(1));
        }
    } else {
        cerr << "Error: The help command must be either typed as \"help\" or as \"help [argument name]\"" << endl;
    }
}

/**
 * Prints a simple about message.
 */
void handleAbout() {
    cout << "Rex - The ROS Extractor" << endl
         << "University of Waterloo, Copyright 2017" << endl
         << "Licensed under GNU Public License v3" << endl << endl
         << "This program is distributed in the hope that it will be useful," << endl
         << "but WITHOUT ANY WARRANTY; without even the implied warranty of" << endl
         << "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the" << endl
         << "GNU General Public License for more details." << endl << endl
         << "----------------------------------------------------" << endl << endl
         << "Rex is a ROS-based fact extractor designed for extracting program" << endl
         << "details from C/C++ source code. Rex will parse out C/C++ language" << endl
         << "features, ROS messages, and ROS core functions and store the" << endl
         << "information in a TA (tuple-attribute) file that can then be visualized" << endl
         << "or queried by additional software." << endl << endl
         << "To start Rex in \'simple\' mode, simply run Rex by specifying files and" << endl
         << "other Clang-based arguments on the command line. Otherwise, Rex can just" << endl
         << "be started with no command line arguments." << endl << endl
         << "For more information on available commands, type \'help\'." << endl;
}

/**
 * Handles the exit. Checks if theres still files or graphs to process.
 * @return Whether the user requested to exit.
 */
bool handleExit() {
    //Checks if there are items to be processed.
    if (masterHandle->getNumFiles() > 0 || masterHandle->getNumGraphs() > 0){
        bool result = promptForAction("There are still items to be processed. Are you sure you want to quit (Y/N): ");
        return result;
    }

    return true;
}

/**
 * Command to generate the current graph.
 * Allows users to specify the mode of processing. This can be minimal or regular.
 * @param line The line to perform command line argument parsing.
 * @param desc The program options information for this command.
 */
void generateGraph(string line, po::options_description desc) {
    //Generates the arguments.
    vector<string> tokens = tokenizeBySpace(line);
    char** argv = createArgv(tokens);
    int argc = (int) tokens.size();

    bool minMode = false;
    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, (const char *const *) argv, desc), vm);

        //Checks if help was enabled.
        if (vm.count("help")) {
            cout << "Usage: generate [options]" << endl << desc;
            return;
        }

        //Sets up what's getting listed.
        if (vm.count("minimal")){
            minMode = true;
        }

    } catch(po::error& e) {
        cerr << "Error: " << e.what() << endl;
        cerr << desc;
        return;
    }

    //Checks whether we can generate.
    int numFiles = masterHandle->getNumFiles();
    if (numFiles == 0) {
        cerr << "No files are in the queue to be processed. Add some before you continue." << endl;
        return;
    }

    //Next, tells Rex to generate them.
    cout << "Processing " << numFiles << " file(s)..." << endl << "This may take some time!" << endl << endl;
    bool success = masterHandle->processAllFiles(minMode);

    //Checks the success of the operation.
    if (success) cout << "ROS contribution graph was created successfully!" << endl
                << "Graph number is #" << masterHandle->getNumGraphs() - 1 << "." << endl;
}

/**
 * Driver method for the OUTPUT argument.
 * Allows users to specify what graphs to output.
 * Also performs basic sanity checking.
 * @param line The line to perform command line argument parsing.
 * @param desc The program options information for this command.
 */
void outputGraphs(string line, po::options_description desc){
    //Generates the arguments.
    vector<string> tokens = tokenizeBySpace(line);
    char** argv = createArgv(tokens);
    int argc = (int) tokens.size();

    string outputValues = string();;
    vector<int> outputIndex;

    //Processes the command line args.
    po::positional_options_description positionalOptions;
    positionalOptions.add("outputFile", 1);

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, (const char* const*) argv).options(desc)
                          .positional(positionalOptions).run(), vm);
        po::notify(vm);

        //Checks if help was enabled.
        if (vm.count("help")){
            cout << "Usage: output [options] OutputName" << endl << desc;
            return;
        }

        //Check the number of graphs.
        if (masterHandle->getNumGraphs() == 0){
            cerr << "Error: There are no graphs to output!" << endl;
            return;
        }

        //Checks if the user specified an output directory.
        if (!vm.count("outputFile")){
            throw po::error("You must specify an output base name!");
        }

        //Checks if selective output was enabled.
        if (vm.count("select")){
            outputValues = vm["select"].as<string>();

            //Now, parses the values.
            stringstream ss(outputValues);
            int i;
            while (ss >> i){
                outputIndex.push_back(i);
            }

            //Check for validity.
            if (outputIndex.size() == 0){
                throw po::error("Format the --select argument as 1,2,3,5.");
            }
        }

        po::notify(vm);
    } catch(po::error& e) {
        cerr << "Error: " << e.what() << endl;
        cerr << desc;
        return;
    }

    vector<string> outputVec = vm["outputFile"].as<vector<string>>();
    string output = outputVec.at(0);
    bool success = false;

    //Now, outputs the graphs.
    if (outputValues.compare(string()) == 0){
        //We output all the graphs.
        if (masterHandle->getNumGraphs() == 1){
            success = masterHandle->outputIndividualModel(0, output);
        } else {
            success = masterHandle->outputAllModels(output);
        }
    } else {
        //We selectively output the graphs.
        for (int indexNum : outputIndex){
            if (indexNum < 0 || indexNum >= masterHandle->getNumGraphs()){
                cerr << "Error: There are only " << masterHandle->getNumGraphs()
                     << " graphs! " << indexNum << " is out of bounds." << endl;
                return;
            }

            success = masterHandle->outputIndividualModel(indexNum, output);
        }
    }

    if (!success) {
        cerr << "There was an error outputting all graphs to TA models." << endl;
    } else {
        cout << "Contribution networks created successfully." << endl;
    }
}

/**
 * Driver method for the ADD command.
 * Allows users to specify files and folders to add.
 * @param line The line with all the command line arguments.
 */
void addFiles(string line){
    //Tokenize by space.
    vector<string> tokens = tokenizeBySpace(line);

    //Next, we check for errors.
    if (tokens.size() == 1) {
        cerr << "Error: You must include at least one file or directory to process." << endl;
        return;
    }

    //Next, we loop through to add these files.
    for (int i = 1; i < tokens.size(); i++){
        path curPath = tokens.at((unsigned int) i);

        //Check if the element exists.
        if (!exists(curPath)){
            bool result = promptForAction("Warning: File does not exist!\nDo you still want to add it (Y/N): ");
            if (!result) continue;
        }

        int numAdded = masterHandle->addByPath(curPath);

        //Checks whether the system is a directory.
        if (is_directory(curPath)){
           cout << numAdded << " source files were added from the directory " << curPath.filename() << "!" << endl;
        } else {
            cout << "Added C++ file " << curPath.filename() << "!" << endl;
        }
    }
}

/**
 * Driver method for the REMOVE command.
 * Allows users to specify files and folders to remove.
 * @param line The line with all the command line arguments.
 * @param desc The program options for the remove list.
 */
void removeFiles(string line, po::options_description desc){
    //Tokenize by space.
    vector<string> tokens = tokenizeBySpace(line);

    //Generates the arguments.
    char** argv = createArgv(tokens);
    int argc = (int) tokens.size();

    //Processes the command line args.
    po::positional_options_description positionalOptions;
    positionalOptions.add("source", -1);

    po::variables_map vm;
    bool regex = false;
    try {
        po::store(po::parse_command_line(argc, (const char *const *) argv, desc), vm);

        if (vm.count("regex")){
            regex = true;
        }

        //Check for processing errors.
        if (vm.count("source") && vm.count("regex")){
            throw po::error("The --source and --regex options cannot be used together!");
        }
        if (!vm.count("source") && !regex) {
            throw po::error("You must include at least one file or directory to remove from.");
        }
    } catch(po::error& e) {
        cerr << "Error: " << e.what() << endl;
        cerr << desc;
        return;
    }


    //Checks what operation to perform.
    if (!regex){
        vector<string> removeFiles = vm["source"].as<vector<string>>();

        //Next, we loop through to remove files.
        for (int i = 1; i < removeFiles.size(); i++){
            path curPath = removeFiles.at((unsigned int) i);

            int numRemoved = masterHandle->removeByPath(curPath);
            if (!is_directory(curPath) && numRemoved == 0){
                cerr << "The file " << curPath.filename() << " is not in the list." << endl;
            } else if (is_directory(curPath)){
                cout << numRemoved << " files have been removed for directory " << curPath.filename() << "!" << endl;
            } else {
                cout << "The file " << curPath.filename() << " has been removed!" << endl;
            }
        }
    } else {
        string regexString = vm["regex"].as<string>();

        //Simply remove by regex.
        int numRemoved = masterHandle->removeByRegex(regexString);
        if (!numRemoved){
            cerr << "The regular expression " << regexString << " did not match any files." << endl;
        } else {
            cout << "The regular expression " << regexString << " removed " << numRemoved << " file(s)." << endl;
        }
    }
}

/**
 * Driver method for the LIST command.
 * Prints the current state that the program is in.
 * @param line The line with all the command line arguments.
 */
void listState(string line, po::options_description desc){
    bool listAll = true;
    bool listGraphs = false;
    bool listFiles = false;

    //First we tokenize.
    vector<string> tokens = tokenizeBySpace(line);

    //Next, we split into arguments.
    char** argv = createArgv(tokens);
    int argc = (int) tokens.size();

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, (const char *const *) argv, desc), vm);

        //Checks if help was enabled.
        if (vm.count("help")) {
            cout << "Usage: list [options]" << endl << desc;
            return;
        }

        //Sets up what's getting listed.
        if (vm.count("num-graphs")){
            listAll = false;
            listGraphs = true;
        }
        if (vm.count("files")){
            listAll = false;
            listFiles = true;
        }
    } catch(po::error& e) {
        cerr << "Error: " << e.what() << endl;
        cerr << desc;
        return;
    }

    //Lists what was requested.
    if (listAll || listGraphs){
        cout << "Current number of graphs generated: " << masterHandle->getNumGraphs() << endl;
    }
    if (listAll || listFiles){
        cout << "Number of files: " << masterHandle->getNumFiles() << endl;
        if (masterHandle->getNumFiles() > 0){
            cout << "Files to be processed for the next graph: " << endl;

            vector<string> files = masterHandle->getFiles();

            //Prints the files only if they are available.
            for (string file : files) {
                cout << file << endl;
            }
        }
    }
}

/**
 * Runs a script from a file.
 * @param line The line that has the script filename.
 */
void runScript(string line, po::options_description desc){
    //First we tokenize.
    vector<string> tokens = tokenizeBySpace(line);

    //Generates the arguments.
    char** argv = createArgv(tokens);
    int argc = (int) tokens.size();

    //Processes the command line args.
    po::positional_options_description positionalOptions;
    positionalOptions.add("script", 1);

    po::variables_map vm;
    string filename = "";
    try {
        po::store(po::command_line_parser(argc, (const char* const*) argv).options(desc)
                          .positional(positionalOptions).run(), vm);
        po::notify(vm);

        if (vm.count("help")) {
            cout << "Usage: script [script]" << endl << desc;
            return;
        }

        if (!vm.count("script")) throw po::error("No script file was supplied!");
        filename = vm["script"].as<string>();
    } catch(po::error& e) {
        cerr << "Error: " << e.what() << endl;
        cerr << desc;
        return;
    }

    //Process the filename.
    std::ifstream scriptFile;
    scriptFile.open(filename);

    //Loop until we hit eof.
    string curLine;
    bool continueLoop = true;
    while(!scriptFile.eof() && continueLoop) {
        getline(scriptFile, curLine);
        continueLoop = !processCommand(curLine);
    }

    scriptFile.close();
    if (!continueLoop){
        _exit(1);
    }
}

/**
 * Simple method that prints the header for Rex
 * to display to users what program they're running.
 */
void printHeader(){
    cout << "                .\"\";._   _.---._   _.-\"\".\n"
            "               /_.'_  '-'      /`-` \\_   \\\n"
            "             .'   / `\\         \\   /` \\   '.\n"
            "           .'    /    ;    _ _  '-;    \\   ;'.\n"
            "        _.'     ;    /\\   / \\ \\    \\    ;  '._;._\n"
            "     .-'.--.    |   /  |  \\0|0/     \\   |        '-.\n"
            "    / /`    \\   |  /  .'             \\  | .---.     \\\n"
            "   | |       |  / /--'   .-\"\"\"-.      \\ \\/     \\     |\n"
            "   \\ \\      /  / /      (  , ,  )     /\\ \\      |    /\n"
            "    \\ '----' .' |        '-(_)-'     |  | '.   /    /\n"
            "     `'----'`   |                    '. |   `'----'`\n"
            "                \\                      `/\n"
            "                 '.         ,         .'\n"
            "                   `-.____.' '.____.-'\n"
            "                          \\   /\n"
            "                           '-'\n"
            "              __________               \n"
            "              \\______   \\ ____ ___  ___\n"
            "               |       _// __ \\\\  \\/  /\n"
            "               |    |   \\  ___/ >    < \n"
            "               |____|_  /\\___  >__/\\_ \\\n"
            "                      \\/     \\/      \\/\n"
            "                    Go get 'em boy!\n";
}

/**
 * Runs Rex in simple mode. Takes in the argc and
 * argv from the main method and uses it to run
 * a Clang tool.
 * @param argc Argc from main.
 * @param argv Argv from main.
 */
void parseSimpleMode(int argc, const char** argv) {
    //Prints the simple mode.
    cout << endl << "Running Rex in simple mode." << endl;

    //Runs Rex with basic arguments.
    RexHandler handleTool;
    bool success = handleTool.processClangToolCode(argc, argv);

    //Check success.
    bool outputModel = true;
    if (!success) outputModel = promptForAction("Do you want to output the model of this code (Y/N): ");

    if (outputModel) {
        bool succ = handleTool.outputIndividualModel(0);
        if (succ) cout << "Model saved!";
    } else {
        cout << "Model generation aborted!" <<  endl;
    }
}

bool processCommand(string line){
    if (line.compare("") == 0) return false;

    //Checks which option is used.
    if (!line.compare(0, HELP_ARG.size(), HELP_ARG) &&
            (line[HELP_ARG.size()] == ' ' || line.size() == HELP_ARG.size())) {
        handleHelp(line, helpStrings);
    } else if (!line.compare(0, ABOUT_ARG.size(), ABOUT_ARG) &&
            (line[ABOUT_ARG.size()] == ' ' || line.size() == ABOUT_ARG.size())) {
        handleAbout();
    } else if (!line.compare(0, EXIT_ARG.size(), EXIT_ARG) &&
            (line[EXIT_ARG.size()] == ' ' || line[EXIT_ARG.size()] == '!') || line.size() == EXIT_ARG.size()) {
        if (line.at(line.size() - 1) == '!') return false;
        return !handleExit();
    } else if (!line.compare(0, GEN_ARG.size(), GEN_ARG) &&
            (line[GEN_ARG.size()] == ' ' || line.size() == GEN_ARG.size())) {
        generateGraph(line, *(helpInfo.at(GEN_ARG).desc.get()));
    } else if (!line.compare(0, OUT_ARG.size(), OUT_ARG) &&
            (line[OUT_ARG.size()] == ' ' || line.size() == OUT_ARG.size())) {
        outputGraphs(line, *(helpInfo.at(OUT_ARG).desc.get()));
    } else if (!line.compare(0, ADD_ARG.size(), ADD_ARG) &&
            (line[ADD_ARG.size()] == ' ' || line.size() == ADD_ARG.size())) {
        addFiles(line);
    } else if (!line.compare(0, REMOVE_ARG.size(), REMOVE_ARG) &&
            (line[REMOVE_ARG.size()] == ' ' || line.size() == REMOVE_ARG.size())) {
        removeFiles(line, *(helpInfo.at(REMOVE_ARG).desc.get()));
    } else if (!line.compare(0, LIST_ARG.size(), LIST_ARG) &&
            (line[LIST_ARG.size()] == ' ' || line.size() == LIST_ARG.size())) {
        listState(line, *(helpInfo.at(LIST_ARG).desc.get()));
    } else if (!line.compare(0, SCRIPT_ARG.size(), SCRIPT_ARG) &&
            (line[SCRIPT_ARG.size()] == ' ' || line.size() == SCRIPT_ARG.size())) {
        runScript(line, *(helpInfo.at(SCRIPT_ARG).desc.get()));
    } else {
        cerr << "No such command: " << line << "\nType \'help\' for more information." << endl;
    }
}

/**
 * Parses the command line arguments in non-simple mode.
 * Loops until the user chooses to exit.
 */
void parseCommands() {
    masterHandle = new RexHandler();

    bool contLoop = true;
    string username;
    string line;

    //Gets the username.
    try {
        struct passwd *pass;
        pass = getpwuid(getuid());
        username = pass->pw_name;
    } catch(exception& e){
        //If the lookup fails.
        username = "user";
    }

    //Generates the descriptions for the help system.
    generateHelp(&helpInfo, &helpStrings);

    //Prints a help message.
    cout << endl << "Type \'help\' to see a list of available commands." << endl;
    while(contLoop) {
        cerr.flush();
        cout.flush();

        //Prompts the user.
        cout << username << " > ";
        getline(cin, line);

        contLoop = processCommand(line);
    }
}

/**
 * Main method that drives the program. Prints the
 * header and then determines what mode to be in.
 * @param argc The number of arguments
 * @param argv Char** array of arguments
 * @return Return code denoting success.
 */
int main(int argc, const char** argv) {
    bool simpleMode = argc != 1;

    //Print the header first.
    printHeader();

    //Check the option.
    if (simpleMode){
        parseSimpleMode(argc, argv);
    } else {
        parseCommands();
    }

    return 0;
}
