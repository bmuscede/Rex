#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>

using namespace std;
using namespace boost::filesystem;

const string DEFAULT_NAME = "compile_commands.json";

vector<string> readFile(string fileName){
    //Creates an input filestream.
    ifstream t(fileName);

    string element;
    vector<string> output;
    while (getline(t, element)){
        output.push_back(element);
    }

    return output;
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

string mergeJSON(vector<string> jsonFiles){
    string output = "[\n";

    //Iterates through all the files.
    for (string currentFile : jsonFiles){
        //Reads in the current file.
        vector<string> contents = readFile(currentFile);

        //Iterates through and positions.
        for (int i = 1; i < contents.size() - 1; i++){
            output += contents.at(i) + ((i == contents.size() - 2) ? ",\n" : "\n");
        }
    }

    output += "]";
    return output;
}

bool saveOutput(string outputName, string outputContents){
    ofstream outfile(outputName);
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
    if (argc != 3 && argc != 2){
        cerr << "Usage: " << argv[0] << "[input dir] [output filename]" << endl;
        cerr << endl << "[input dir]        : Name of input directory with compile_commands.json files." << endl;
        cerr << endl << "[output filename]  : Output filename (OPTIONAL)." << endl;
        return 1;
    }

    //Next, gets the two files.
    path inputDir = path(argv[1]);
    string outputFile;
    if (argc == 3) outputFile = argv[2];
    else outputFile = DEFAULT_NAME;

    //Performs validity checking.
    if (!is_directory(inputDir)){
        cerr << "Directory " << inputDir << " is not a directory!" << endl;
        return 2;
    }
    if (is_regular_file(outputFile)){
        cerr << "File " << outputFile << " already exists!" << endl;
        return 2;
    }

    //Next, we look for all the JSON files.
    vector<string> jsonFiles = findJSON(inputDir);

    //From that, reads them in one by one and generates an output file.
    string output = mergeJSON(jsonFiles);
    bool succ = saveOutput(outputFile, output);

    if (succ){
        cout << "Save to " << outputFile << " was successful!" << endl;
    } else {
        cerr << "Merged JSON file could not be saved to " << outputFile << "!" << endl;
        return 1;
    }
    return 0;
}