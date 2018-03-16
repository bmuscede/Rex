//
// Created by bmuscede on 13/03/18.
//

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
    ScenarioWalker(path mainScenarioLoc, path rosPkgDir);
    ~ScenarioWalker();

    bool readFiles();
    bool processScenario();

    std::vector<std::string> getActivePackages();

private:
    const std::string JSON_EXT = ".json";
    const std::string LAUNCH_EXT = ".launch";

    const std::string INCLUDE_NODE = "include";
    const std::string FILE_ATTRIBUTE = "file";
    const std::string NODE_NODE = "node";
    const std::string PKG_ATTRIBUTE = "pkg";

    const boost::regex FIND_REGEX = boost::regex("\\$\\( *find *.*.*\\)");

    bool filesRead = false;
    path rosPkgDir;
    path mainScenario;
    path launchFile;
    path jsonFile;

    std::vector<std::string> activePackages;

    void readLaunchFile();

    std::vector<rapidxml::xml_document<>*> expandIncludes(rapidxml::xml_document<> *doc);
    std::string findPackage(std::string pkgName, path startDir);

    std::vector<std::string> getActivePackages(rapidxml::xml_document<>* doc);
};


#endif //REX_SCENARIOWALKER_H
