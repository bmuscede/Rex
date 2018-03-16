//
// Created by bmuscede on 13/03/18.
//

#include "../XML/rapidxml_utils.hpp"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include "ScenarioWalker.h"

using namespace std;

ScenarioWalker::ScenarioWalker(path mainScenarioLoc, path rosPkgDir) {
    mainScenario = mainScenarioLoc;
    this->rosPkgDir = rosPkgDir;
}

ScenarioWalker::~ScenarioWalker() { }

bool ScenarioWalker::readFiles() {
    bool foundLaunch = false;
    bool foundJSON = false;

    //Check the path for the files.
    directory_iterator end_itr;
    for (directory_iterator itr(mainScenario); itr != end_itr; ++itr){
        path cur = *itr;

        //Check the file types.
        if (boost::algorithm::ends_with(cur.string(), LAUNCH_EXT) && !foundLaunch){
            foundLaunch = true;
            launchFile = cur;
        } else if (boost::algorithm::ends_with(cur.string(), JSON_EXT) && !foundJSON){
            foundJSON = true;
            jsonFile = cur;
        }
    }

    //Perform error checking.
    if (!foundJSON || !foundLaunch) return false;

    filesRead = true;
    return true;
}

bool ScenarioWalker::processScenario(){
    if (!filesRead) return false;

    readLaunchFile();
    return true;
}

vector<string> ScenarioWalker::getActivePackages(){
    return activePackages;
}

void ScenarioWalker::readLaunchFile(){
    //Attempt to load in the XML file.
    rapidxml::file<> xmlFile(launchFile.string().c_str());
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    //Get all include tags.
    vector<rapidxml::xml_document<>*> docs = expandIncludes(&doc);

    //Next, parse the documents for active nodes.
    vector<string> results;
    for (auto curDoc : docs){
        vector<string> cur = getActivePackages(curDoc);
        results.insert(results.end(), cur.begin(), cur.end());
    }
    activePackages = results;

    for (auto curDoc : docs) delete curDoc;
}

vector<rapidxml::xml_document<>*> ScenarioWalker::expandIncludes(rapidxml::xml_document<> *doc){
    vector<rapidxml::xml_document<>*> docs;
    docs.push_back(doc);

    //Iterate through the include tags in the document.
    for (auto node = doc->first_node(INCLUDE_NODE.c_str()); node; node = node->next_sibling(INCLUDE_NODE.c_str())){
        //Get the file attribute.
        for (auto attr = node->first_attribute(); attr; attr = attr->next_attribute()){
            if (attr->name() == FILE_ATTRIBUTE.c_str()){
                //Expand out the document.
                string loc  = attr->value();

                //Search for find.
                if (boost::regex_search(loc, FIND_REGEX)){
                    //Get the results.
                    boost::smatch match;
                    boost::regex_search(loc, match, FIND_REGEX);
                    string result = match.str(1);

                    //Remove characters from string and trim.
                    boost::erase_all(result, "$");
                    boost::erase_all(result, "(");
                    boost::erase_all(result, ")");
                    boost::erase_all(result, "find");
                    boost::trim(result);

                    //Look up the string.
                    string path = findPackage(result, rosPkgDir);
                    if (path != "") string replacePath = regex_replace(loc, FIND_REGEX, path);
                }

                //Next, search for the file.
                path nextFile = loc;
                if (!exists(nextFile)){
                    continue;
                }

                //Open it for parsing.
                rapidxml::file<> xmlFile(nextFile.string().c_str());
                auto nextDoc = new rapidxml::xml_document<>();
                nextDoc->parse<0>(xmlFile.data());

                //Generate the inner results.
                auto results = expandIncludes(nextDoc);
                docs.insert(docs.end(), results.begin(), results.end());
            }
        }
    }

    return docs;
}

string ScenarioWalker::findPackage(string pkgName, path startDir){
    //Find the first instance of the package name.
    for (directory_iterator itr(startDir); itr!=directory_iterator(); ++itr) {
        if (is_directory(itr->path())){
            string name = itr->path().filename().string();
            if (name == pkgName){
                return canonical(itr->path()).string();
            } else {
                string result = findPackage(pkgName, itr->path());
                if (result != "") return result;
            }
        }
    }

    return "";
}

vector<string> ScenarioWalker::getActivePackages(rapidxml::xml_document<>* doc){
    vector<string> results;

    for (auto node = doc->first_node(NODE_NODE.c_str()); node; node = node->next_sibling(NODE_NODE.c_str())){
        for (auto attr = node->first_attribute(); attr; attr = attr->next_attribute()){
            if (string(attr->name()) == PKG_ATTRIBUTE){
                results.push_back(string(attr->value()));
                continue;
            }
        }
    }

    return results;
}