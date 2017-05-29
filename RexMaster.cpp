//
// Created by bmuscede on 13/05/17.
//

#include <iostream>
#include <pwd.h>
#include <zconf.h>
#include <vector>
#include <boost/algorithm/string/regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "RexHandler.h"

using namespace std;
using namespace boost::filesystem;
namespace po = boost::program_options;

const static string DEFAULT_FILENAME = "a.out";

const static string HELP_ARG = "help";
const static string ABOUT_ARG = "about";
const static string EXIT_ARG = "quit";
const static string GEN_ARG = "generate";
const static string OUT_ARG = "output";
const static string ADD_ARG = "add";
const static string REMOVE_ARG = "remove";
const static string LIST_ARG = "list";

static RexHandler* masterHandle;

vector<string> tokenizeBySpace(string line){
    vector<string> result;
    istringstream iss(line);
    for(std::string s; iss >> s; )
        result.push_back(s);

    return result;
}

char** createArgv(vector<string> tokens){
    char** tokenC;

    //First, create the array.
    tokenC = new char*[(int) tokens.size()];
    for (int i = 0; i < tokens.size(); i++){
        tokenC[i] = new char[(int) tokens.at(i).size()];
        strcpy(tokenC[i], tokens.at(i).c_str());
    }

    return tokenC;
}

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

//TODO
void handleHelp() {

}

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

    return;
}

bool handleExit() {
    //Checks if there are items to be processed.
    if (masterHandle->getNumFiles() > 0 || masterHandle->getNumGraphs() > 0){
        bool result = promptForAction("There are still items to be processed. Are you sure you want to quit (Y/N): ");
        return result;
    }

    return true;
}

void generateGraph() {
    //Checks whether we can generate.
    int numFiles = masterHandle->getNumFiles();
    if (numFiles == 0) {
        cerr << "No files are in the queue to be processed. Add some before you continue." << endl;
        return;
    }

    //Next, tells Rex to generate them.
    cout << "Processing " << numFiles << " file(s)..." << endl << "This may take some time!" << endl << endl;
    bool success = masterHandle->processAllFiles();

    //Checks the success of the operation.
    if (success) cout << "ROS contribution graph was created successfully!" << endl
                << "Graph number is #" << masterHandle->getNumGraphs() - 1 << "." << endl;
}

//TODO
void outputGraphs(string line){
    //Generates the arguments.
    vector<string> tokens = tokenizeBySpace(line);
    char** argv = createArgv(tokens);
    int argc = (int) tokens.size();

    string outputValues = string();;
    vector<int> outputIndex;

    //Processes the command line args.
    po::options_description desc("Options");
    po::positional_options_description positionalOptions;

    desc.add_options()
            ("help,h", "Print help message for output.")
            ("select,s", po::value<string>(), "Outputs graphs by graph number. Can separate values by comma.")
            ("outputFile", po::value<std::vector<std::string> >(), "The base file name to save.");
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

void removeFiles(string line){
    //Tokenize by space.
    vector<string> tokens = tokenizeBySpace(line);

    //Next, we check for errors.
    if (tokens.size() == 1) {
        cerr << "Error: You must include at least one file or directory to remove from." << endl;
        return;
    }

    //Next, we loop through to remove files.
    for (int i = 1; i < tokens.size(); i++){
        path curPath = tokens.at((unsigned int) i);

        int numRemoved = masterHandle->removeByPath(curPath);
        if (!is_directory(curPath) && numRemoved == 0){
            cerr << "The file " << curPath.filename() << " is not in the list." << endl;
        } else if (is_directory(curPath)){
            cout << numRemoved << " files have been removed for directory " << curPath.filename() << "!" << endl;
        } else {
            cout << "The file " << curPath.filename() << " has been removed!" << endl;
        }
    }
}

void listState(string line){
    bool listAll = true;
    bool listGraphs = false;
    bool listFiles = false;

    //First we tokenize.
    vector<string> tokens = tokenizeBySpace(line);

    //Next, we split into arguments.
    char** argv = createArgv(tokens);
    int argc = (int) tokens.size();

    //Sets up the program options.
    po::options_description desc("Options");
    desc.add_options()
            ("help,h", "Print help message for list.")
            ("num-graphs,g", "Prints the number of graphs already generated.")
            ("files,f", "Lists all the files for the current input.");

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
        bool succ = handleTool.outputIndividualModel(0, DEFAULT_FILENAME);
        if (succ) cout << "Model saved to " << DEFAULT_FILENAME << "!";
    } else {
        cout << "Model generation aborted!" <<  endl;
    }
}

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

    //Prints a help message.
    cout << endl << "Type \'help\' to see a list of available commands." << endl;
    while(contLoop) {
        cerr.flush();
        cout.flush();

        //Prompts the user.
        cout << username << " > ";
        getline(cin, line);

        if (line.compare("") == 0) continue;

        //Checks which option is used.
        if (!line.compare(0, HELP_ARG.size(), HELP_ARG)) {
            handleHelp();
        } else if (!line.compare(0, ABOUT_ARG.size(), ABOUT_ARG)){
            handleAbout();
        } else if (!line.compare(0, EXIT_ARG.size(), EXIT_ARG)) {
            if (line.at(line.size() - 1) == '!') break;
            contLoop = !handleExit();
        } else if (!line.compare(0, GEN_ARG.size(), GEN_ARG)) {
            generateGraph();
        } else if (!line.compare(0, OUT_ARG.size(), OUT_ARG)) {
            outputGraphs(line);
        } else if (!line.compare(0, ADD_ARG.size(), ADD_ARG)) {
            addFiles(line);
        } else if (!line.compare(0, REMOVE_ARG.size(), REMOVE_ARG)) {
            removeFiles(line);
        } else if (!line.compare(0, LIST_ARG.size(), LIST_ARG)) {
            listState(line);
        } else {
            cerr << "No such command: " << line << "\nType \'help\' for more information." << endl;
        }
    }
}

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