//
// Created by bmuscede on 02/05/18.
//

#include <fstream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "LowMemoryTAGraph.h"

using namespace std;
namespace bs = boost::filesystem;

int LowMemoryTAGraph::currentNumber = 0;
const string LowMemoryTAGraph::CUR_FILE_LOC = "curFile.txt";
const string LowMemoryTAGraph::CUR_SETTING_LOC = "curSetting.txt";

LowMemoryTAGraph::LowMemoryTAGraph() : TAGraph() {
    purge = true;
    fileNumber = LowMemoryTAGraph::currentNumber;
    LowMemoryTAGraph::currentNumber++;

    instanceFN = bs::weakly_canonical(bs::path(to_string(fileNumber) + "-" + BASE_INSTANCE_FN)).string();
    relationFN = bs::weakly_canonical(bs::path(to_string(fileNumber) + "-" + BASE_RELATION_FN)).string();
    mvRelationFN = bs::weakly_canonical(bs::path(to_string(fileNumber) + "-" + BASE_MV_RELATION_FN)).string();
    attributeFN = bs::weakly_canonical(bs::path(to_string(fileNumber) + "-" + BASE_ATTRIBUTE_FN)).string();
    settingFN = bs::weakly_canonical(bs::path(to_string(fileNumber) + "-" + CUR_SETTING_LOC)).string();
    curFileFN = bs::weakly_canonical(bs::path(to_string(fileNumber) + "-" + CUR_FILE_LOC)).string();

    if (doesFileExist(instanceFN)) deleteFile(instanceFN);
    if (doesFileExist(relationFN)) deleteFile(relationFN);
    if (doesFileExist(attributeFN)) deleteFile(attributeFN);
    std::ofstream f = std::ofstream{ instanceFN };
    f.close();
    f = ofstream{ relationFN };
    f.close();
    f = ofstream{ attributeFN };
    f.close();
    f = ofstream{ settingFN };
    f.close();
    f = ofstream{ curFileFN };
    f.close();
}

LowMemoryTAGraph::LowMemoryTAGraph(string basePath) : TAGraph() {
    purge = true;
    fileNumber = LowMemoryTAGraph::currentNumber;
    LowMemoryTAGraph::currentNumber++;

    instanceFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + BASE_INSTANCE_FN)).string();
    relationFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + BASE_RELATION_FN)).string();
    mvRelationFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + BASE_MV_RELATION_FN)).string();
    attributeFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + BASE_ATTRIBUTE_FN)).string();
    settingFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + CUR_SETTING_LOC)).string();
    curFileFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + CUR_FILE_LOC)).string();

    if (doesFileExist(instanceFN)) deleteFile(instanceFN);
    if (doesFileExist(relationFN)) deleteFile(relationFN);
    if (doesFileExist(attributeFN)) deleteFile(attributeFN);
    std::ofstream f = std::ofstream{ instanceFN };
    f.close();
    f = ofstream{ relationFN };
    f.close();
    f = ofstream{ attributeFN };
    f.close();
    f = ofstream{ settingFN };
    f.close();
    f = ofstream{ curFileFN };
    f.close();
}

LowMemoryTAGraph::LowMemoryTAGraph(string basePath, int curNum) : TAGraph() {
    purge = true;
    fileNumber = curNum;

    instanceFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + BASE_INSTANCE_FN)).string();
    relationFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + BASE_RELATION_FN)).string();
    mvRelationFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + BASE_MV_RELATION_FN)).string();
    attributeFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + BASE_ATTRIBUTE_FN)).string();
    settingFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + CUR_SETTING_LOC)).string();
    curFileFN = bs::weakly_canonical(bs::path(basePath + "/" + to_string(fileNumber) + "-" + CUR_FILE_LOC)).string();
}

LowMemoryTAGraph::~LowMemoryTAGraph(){
    if (doesFileExist(instanceFN)) deleteFile(instanceFN);
    if (doesFileExist(relationFN)) deleteFile(relationFN);
    if (doesFileExist(attributeFN)) deleteFile(attributeFN);
    if (doesFileExist(settingFN)) deleteFile(settingFN);
    if (doesFileExist(curFileFN)) deleteFile(curFileFN);
}

void LowMemoryTAGraph::addNode(RexNode* node){
    //Check the number of entities.
    int amt = getNumberEntities();
    if (amt > PURGE_AMOUNT){
        purgeCurrentGraph();
    }

    //Add the graph.
    TAGraph::addNode(node);
}

void LowMemoryTAGraph::addEdge(RexEdge* edge){
    //Check the number of entities.
    int amt = getNumberEntities();
    if (amt > PURGE_AMOUNT){
        purgeCurrentGraph();
    }

    //Add the graph.
    return TAGraph::addEdge(edge);
}

string LowMemoryTAGraph::getTAModel() {
    string format = (minMode) ? MINI_TA_SCHEMA : FULL_TA_SCHEMA;
    string curLine;

    //Generate the instances.
    format += "FACT TUPLE :\n";
    ifstream instances(instanceFN);
    if (instances.is_open()) while(getline(instances, curLine)) format += curLine + "\n";
    instances.close();

    //Generate the relations.
    ifstream relations(relationFN);
    if (relations.is_open()) while(getline(relations, curLine)) format += curLine + "\n";
    relations.close();
    format += "\n";

    //Generate the attributes.
    format += "FACT ATTRIBUTE :\n";
    ifstream attributes(attributeFN);
    if (attributes.is_open()) while(getline(attributes, curLine)) format += curLine + "\n";
    attributes.close();

    return format;
}

void LowMemoryTAGraph::dumpCurrentFile(int fileNum, string file){
    //Opens the file list.
    std::ofstream curFile(curFileFN);
    if (!curFile.is_open()) return;

    //Writes the current file.
    curFile << fileNum << endl << file;
    curFile.close();
}

void LowMemoryTAGraph::dumpSettings(vector<bs::path> files, bool minMode){
    //Opens the file.
    std::ofstream curSettings(settingFN);
    if (!curSettings.is_open()) return;

    //First, dump the files.
    for (int i = 0; i < files.size(); i++){
        curSettings << files.at(i).string();
        if (i != files.size() - 1) curSettings << ",";
    }
    curSettings << endl;

    //Next, dump the min mode.
    curSettings << minMode;
    curSettings.close();
}

void LowMemoryTAGraph::purgeCurrentGraph(){
    if (!purge) return;

    //Start by writing everything to disk.
    ofstream instances(instanceFN, std::ios::out | std::ios::app);
    if (!instances.is_open()) return;
    instances << generateInstances();
    instances.close();

    ofstream relations(relationFN, std::ios::out | std::ios::app);
    if (!relations.is_open()) return;
    relations << generateRelations();
    relations.close();

    ofstream attributes(attributeFN, std::ios::out | std::ios::app);
    if (!attributes.is_open()) return;
    attributes << generateAttributes();
    attributes.close();

    //Clear the graph.
    clearGraph();
}

void LowMemoryTAGraph::purgeUnestablishedEdges(bool resolveFirst) {
    unordered_map<string, string> idMap;

    //Load in the tuples.
    string line;
    vector<string> strVec;
    ifstream instances(instanceFN);
    if (!instances.is_open()) return;

    //Create map of tuples.
    while(getline(instances, line)){
        boost::algorithm::split(strVec,line, boost::is_any_of(" "));
        if (strVec.size() != 3) continue;

        idMap[strVec.at(1)] = strVec.at(2);
    }
    instances.close();

    //Next, resolves the relations.
    bs::path org = relationFN;
    bs::path dst = mvRelationFN;
    rename(org, dst);

    ifstream original(mvRelationFN);
    ofstream destination(relationFN, std::ios_base::out);
    if (!original.is_open() || !destination.is_open()) return;
    while (getline(original, line)){
        boost::algorithm::split(strVec,line, boost::is_any_of(" "));
        if (strVec.size() != 3) return;

        //Check if the edge is established
        if (idMap.find(strVec.at(1)) != idMap.end() && idMap.find(strVec.at(2)) != idMap.end()) {
            //Write out to disk.
            destination << line << endl;
        }
    }
    original.close();
    destination.close();
    deleteFile(mvRelationFN);

    //Last, rewrite the original instances.
    ofstream outInt(instanceFN, std::ios_base::out);
    if (!outInt.is_open()) return;
    for (auto it = idMap.begin(); it != idMap.end(); ++it){
        outInt << "$INSTANCE " << it->first << " " << it->second << endl;
    }
    outInt.close();
}

int LowMemoryTAGraph::getNumberEntities(){
    //Get the number of keys in the graph.
    auto curAmnt = (int) idList.size();
    return curAmnt;
}

bool LowMemoryTAGraph::doesFileExist(string fN){
    struct stat buffer;
    return (stat (fN.c_str(), &buffer) == 0);
}

void LowMemoryTAGraph::deleteFile(string fN) {
    remove(fN.c_str());
}
