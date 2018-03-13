//
// Created by bmuscede on 13/03/18.
//

#ifndef REX_SCENARIOWALKER_H
#define REX_SCENARIOWALKER_H

#include <boost/filesystem.hpp>
#include <string>

using namespace boost::filesystem;

class ScenarioWalker {
public:
    ScenarioWalker(path mainScenarioLoc);
    ~ScenarioWalker();

    bool readFiles();
    bool processScenario();

private:
    const std::string JSON_EXT = ".json";
    const std::string LAUNCH_EXT = ".launch";

    bool filesRead = false;
    path mainScenario;
    path launchFile;
    path jsonFile;

    void readLaunchFile();
};


#endif //REX_SCENARIOWALKER_H
