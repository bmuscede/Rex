#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include "json/json.h"

using namespace std;
using namespace boost::filesystem;

const string DEFAULT_NAME = "compile_commands.json";
const string COMMAND = "command";
const string INCLUDE_DIR = "-I";

const int INJECT_POS = 5;

string readFile(string fileName){
    //Creates an input filestream.
    std::ifstream t(fileName);

    string element;
    string final = "";
    while (getline(t, element)){
        final += element + "\n";
    }

    return final;
}

vector<string> tokenizeBySpace(string toTokenize){
    istringstream stream(toTokenize);
    return {istream_iterator<string>{stream}, istream_iterator<string>{}};
}

string inject(vector<string> tokens, string toInject, int pos){
    string result = "";

    //Iterate through the tokens.
    for (int i = 0; i < tokens.size(); i++){
        //Checks if we add the string to inject.
        if (i == pos) result += toInject + " ";

        result += tokens.at(i);

        //Adds a space if necessary.
        if (i < tokens.size() - 1) result += " ";
    }

    return result;
}

vector<string> findJSON(path curDirectory){
    vector<string> jsonFiles = vector<string>();
    vector<path> interiorDir = vector<path>();
    directory_iterator endIter;

    //Start by iterating through and inspecting each file.
    for (directory_iterator iter(curDirectory); iter != endIter; iter++){
        path currentPath = iter->path();

        //Check what the current file is.
        if (currentPath.filename().string().compare(DEFAULT_NAME) == 0){
            //We found the JSON file.
            jsonFiles.push_back(currentPath.string());
        } else if (is_directory(currentPath)){
            //Add the directory to the search system.
            interiorDir.push_back(currentPath);
        }
    }

    //Next, goes to all the internal directories.
    for (path cur : interiorDir){
        vector<string> results = findJSON(cur);
        jsonFiles.insert(jsonFiles.end(), results.begin(), results.end());
    }

    return jsonFiles;
}

string mergeJSON(vector<string> jsonFiles, path includeDir){
    Json::Value finalVal;
    int num = 0;

    //Iterates through the file.
    for (string file : jsonFiles){
        cout << "Parsing: " << file << "..." <<  endl;

        //First, gets the file contents.
        string contents = readFile(file);

        //Gets the JSON objects.
        Json::Value root;
        Json::Reader reader;
        bool succ = reader.parse(contents, root);

        //Next, checks whether parsing is successful.
        if (!succ){
            cout << "Error" << endl;
            cout << reader.getFormattedErrorMessages() << endl;
            continue;
        }

        //Prepares the merge.
        int i = 0;
        bool loop = true;
        while (loop){
            Json::Value curr = root[i];
            if (curr.isNull()){
                loop = false;
                continue;
            }

            //We want to inject the build command for Rex.
            string command = curr[COMMAND].asString();

            //Tokenizes the string by spaces and inserts the plus.
            vector<string> tokens = tokenizeBySpace(command);
            command = inject(tokens, INCLUDE_DIR + includeDir.string(), INJECT_POS);

            curr[COMMAND] = command;

            finalVal[num++] = curr;
            i++;
        }
    }

    Json::StyledWriter writer;
    string output = writer.write(finalVal);
    return output;
}

bool saveOutput(string outputName, string outputContents){
    std::ofstream outfile(outputName);
    if(!outfile.is_open()) {
        return false;
    }

    //Writes the string to the file.
    outfile << outputContents;
    outfile.flush();
    outfile.close();

    return true;
}

int main(int argc, char** argv) {
    //Starts by checking the command line arguments.
    if (argc != 4 && argc != 3){
        cerr << "Usage: " << argv[0] << " [input dir] [include dir] [output filename]" << endl;
        cerr << endl << "[input dir]        : Name of input directory with compile_commands.json files.";
	cerr << endl << "[include dir]      : Directory where include libraries are located.";
        cerr << endl << "[output filename]  : Output filename (OPTIONAL)." << endl;
        return 1;
    }

    //Next, gets the two files.
    path inputDir = path(argv[1]);
    path includeDir = path(argv[2]);
    string outputFile;
    if (argc == 4) outputFile = argv[3];
    else outputFile = DEFAULT_NAME;

    //Performs validity checking.
    if (!is_directory(inputDir)){
        cerr << "Directory " << inputDir << " is not a directory!" << endl;
        return 2;
    }
    if (!is_directory(includeDir)){
	cerr << "Directory " << includeDir << " is not a directory!" << endl;
	return 2;
    }
    if (is_regular_file(outputFile)){
        cerr << "File " << outputFile << " already exists!" << endl;
        return 2;
    }

    //Next, we look for all the JSON files.
    vector<string> jsonFiles = findJSON(inputDir);

    //From that, reads them in one by one and generates an output file.
    string output = mergeJSON(jsonFiles, includeDir);
    bool succ = saveOutput(outputFile, output);

    if (succ){
        cout << "Save to " << outputFile << " was successful!" << endl;
    } else {
        cerr << "Merged JSON file could not be saved to " << outputFile << "!" << endl;
        return 1;
    }
    return 0;
}
