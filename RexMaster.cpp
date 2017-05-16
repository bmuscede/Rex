//
// Created by bmuscede on 13/05/17.
//

#include <iostream>
#include <pwd.h>
#include <zconf.h>
#include <vector>
#include <boost/algorithm/string/regex.hpp>
#include <boost/regex.hpp>

using namespace std;

const static string HELP_ARG = "help";
const static string EXIT_ARG = "quit";

void handleHelp() {

}

bool handleExit() {
    return true;
}

int getArgc(string line){
    vector<std::string> result;
    boost::algorithm::split_regex(result, line, boost::regex("\\s+"));
    return (int) result.size();
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

void parseCommands() {
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
    while(contLoop){
        //Prompts the user.
        cout << username << " > ";
        getline(cin, line);

        //Now get argc.
        int linec = getArgc(line);

        //Checks which option is used.
        //TODO: Process arguments.
        if (!line.compare(0, HELP_ARG.size(), HELP_ARG)) {
            handleHelp();
        } else if (!line.compare(0, EXIT_ARG.size(), EXIT_ARG)) {
            contLoop = !handleExit();
        } else {
            cout << "No such command: " << line << "\n\tType \'help\' for more information." << endl;
        }
    }
}

int main(int argc, char** argv) {
    //Print the header first.
    printHeader();

    //Check the option.
    parseCommands();
    return 0;
}