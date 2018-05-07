////////////////////////////////////////////////////////////////////////////////////////////////////////
// ScenarioWalker.cpp
//
// Created By: Bryan J Muscedere
// Date: 13/03/18.
//
// Goes through a ROS scenario to find the ROS packages that
// are active or inactive at the time of the scenario running.
// This slims down the graph.
//
// Copyright (C) 2018, Bryan J. Muscedere
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

#include "../XML/rapidxml_utils.hpp"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include "ScenarioWalker.h"

using namespace std;

/**
 * Main constructor. Gets a scenario location and the root project directory.
 * @param mainScenarioLoc The directory with the scenario XML files.
 * @param rosPkgDir The ROS root project directory.
 */
ScenarioWalker::ScenarioWalker(path mainScenarioLoc, path rosPkgDir) {
    mainScenario = mainScenarioLoc;
    this->rosPkgDir = rosPkgDir;
}

/**
 * Stub Destructor.
 */
ScenarioWalker::~ScenarioWalker() { }

/**
 * Reads scenario files and denotes success.
 * @return Whether the operation was successful.
 */
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

/**
 * Processes a scenario file.
 * @return Whether the operation was successful.
 */
bool ScenarioWalker::processScenario(){
    if (!filesRead) return false;

    readLaunchFile();
    return true;
}

/**
 * Gets all active ROS packages that are a part of the scenario.
 * @return The active packages.
 */
vector<string> ScenarioWalker::getActivePackages(){
    return activePackages;
}

void ScenarioWalker::readLaunchFile(){
    //Attempt to load in the XML file.
    rapidxml::file<> xmlFile(launchFile.string().c_str());
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    //Get all include tags.
    vector<string> results = expandIncludes(&doc, doc.first_node());
    results = clearDuplicates(results);
    activePackages = results;
}

/**
 * Visits other scenarios based on their includes in a main scenario file.
 * @param curDoc The current document.
 * @param curNode The current node.
 * @return Expanded files to investigate.
 */
vector<string> ScenarioWalker::expandIncludes(rapidxml::xml_document<>* curDoc,
                                                                 rapidxml::xml_node<>* curNode){
    vector<string> results;

    //Iterate through the include tags in the document.
    for (auto node = curNode; node; node = node->next_sibling()){

        if (node->name() == INCLUDE_NODE){
            //Get the file attribute.
            for (auto attr = node->first_attribute(); attr; attr = attr->next_attribute()) {
                if (attr->name() == FILE_ATTRIBUTE) {
                    //Expand out the document.
                    string loc = attr->value();

                    //Search for find.
                    if (boost::regex_search(loc, FIND_REGEX)) {
                        //Get the results.
                        boost::smatch match;
                        boost::regex_search(loc, match, FIND_REGEX);
                        string result = match.str();

                        //Remove characters from string and trim.
                        boost::erase_all(result, "$");
                        boost::erase_all(result, "(");
                        boost::erase_all(result, ")");
                        boost::erase_all(result, "find");
                        boost::trim(result);

                        //Look up the string.
                        string path = findPackage(result, rosPkgDir);
                        if (path != "") {
                            string replacePath = regex_replace(loc, FIND_REGEX, path);
                            loc = replacePath;
                        }
                    }

                    //Next, search for the file.
                    path nextFile = loc;
                    if (!exists(nextFile)) {
                        continue;
                    }

                    //Open it for parsing.
                    rapidxml::file<> xmlFile(nextFile.string().c_str());
                    auto nextDoc = new rapidxml::xml_document<>();
                    nextDoc->parse<0>(xmlFile.data());

                    //Generate the inner results.
                    auto curRes = expandIncludes(nextDoc, nextDoc->first_node());
                    results.insert(results.end(), curRes.begin(), curRes.end());
                    results = clearDuplicates(results);
                }
            }
        } else {
            if (node->name() == NODE_NODE){
                for (auto attr = node->first_attribute(); attr; attr = attr->next_attribute()) {
                    if (attr->name() == PKG_ATTRIBUTE) {
                        string val = attr->value();
                        results.push_back(val);
                        break;
                    }
                }
            }

            //Traverse downwards.
            auto curRes = expandIncludes(curDoc, curNode->first_node());
            results.insert(results.end(), curRes.begin(), curRes.end());
            results = clearDuplicates(results);
        }
    }

    return results;
}

/**
 * Finds a ROS package by name recursively.
 * @param pkgName The name of the package.
 * @param startDir The start directory.
 * @return The location.
 */
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

/**
 * Removes duplicates from the results.
 * @param dupArr The initial vector.
 * @return A vector with no duplicates.
 */
vector<string> ScenarioWalker::clearDuplicates(vector<string> dupArr){
    vector<string> resArr;

    //Iterate through all the elements.
    for (string cur : dupArr){
        if (find(resArr.begin(), resArr.end(), cur) == resArr.end()){
            resArr.push_back(cur);
        }
    }

    return resArr;
}