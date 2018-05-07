////////////////////////////////////////////////////////////////////////////////////////////////////////
// ScenarioWalker.h
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

#ifndef REX_SCENARIOWALKER_H
#define REX_SCENARIOWALKER_H

#include "../XML/rapidxml.hpp"
#include "../XML/rapidxml_utils.hpp"
#include <boost/filesystem.hpp>
#include <string>
#include <boost/regex.hpp>
#include <iostream>

using namespace boost::filesystem;

class ScenarioWalker {
public:
    /** Constructor / Destructor */
    ScenarioWalker(path mainScenarioLoc, path rosPkgDir);
    ~ScenarioWalker();

    /** Scenario Manager */
    bool readFiles();
    bool processScenario();

    /** Package Manager */
    std::vector<std::string> getActivePackages();

private:
    /** Helper Variables */
    const std::string JSON_EXT = ".json";
    const std::string LAUNCH_EXT = ".launch";
    const std::string INCLUDE_NODE = "include";
    const std::string FILE_ATTRIBUTE = "file";
    const std::string NODE_NODE = "node";
    const std::string PKG_ATTRIBUTE = "pkg";
    const boost::regex FIND_REGEX = boost::regex("\\$\\( *find *.*.*\\)");

    /** Member Variables */
    bool filesRead = false;
    path rosPkgDir;
    path mainScenario;
    path launchFile;
    path jsonFile;
    std::vector<std::string> activePackages;

    /** Helper Methods */
    void readLaunchFile();
    std::vector<std::string> expandIncludes(rapidxml::xml_document<>* curDoc,
                                                          rapidxml::xml_node<>* curNode);
    std::string findPackage(std::string pkgName, path startDir);
    std::vector<std::string> clearDuplicates(std::vector<std::string> dupArr);
};


#endif //REX_SCENARIOWALKER_H
