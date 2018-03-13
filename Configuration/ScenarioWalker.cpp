//
// Created by bmuscede on 13/03/18.
//

#include "../XML/rapidxml_utils.hpp"
#include <boost/algorithm/string/predicate.hpp>
#include "ScenarioWalker.h"

ScenarioWalker::ScenarioWalker(path mainScenarioLoc) {
    mainScenario = mainScenarioLoc;
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

void ScenarioWalker::readLaunchFile(){
    //Attempt to load in the XML file.
    rapidxml::file<> xmlFile(launchFile.string().c_str());
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    //Get all include tags.

}
