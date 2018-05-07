//
// Created by bmuscede on 02/05/18.
//

#ifndef REX_LOWMEMORYTAGRAPH_H
#define REX_LOWMEMORYTAGRAPH_H

#include "TAGraph.h"

class LowMemoryTAGraph : public TAGraph {
public:
    LowMemoryTAGraph();
    LowMemoryTAGraph(std::string basePath);
    LowMemoryTAGraph(std::string basePath, int curNum);
    ~LowMemoryTAGraph() override;

    /** Changes the Root */
    void changeRoot(std::string basePath);

    void addNode(RexNode* node) override;
    void addEdge(RexEdge* edge) override;

    std::string getTAModel() override;

    void dumpCurrentFile(int fileNum, std::string file);
    void dumpSettings(std::vector<boost::filesystem::path> files, bool minMode);

    void purgeCurrentGraph();

    bool resolveComponents(std::map<std::string, std::vector<std::string>> databaseMap) override;
    void purgeUnestablishedEdges(bool resolveFirst = true) override;

    static const std::string CUR_FILE_LOC;
    static const std::string CUR_SETTING_LOC;
    static const std::string BASE_INSTANCE_FN;
    static const std::string BASE_RELATION_FN;
    static const std::string BASE_MV_RELATION_FN;
    static const std::string BASE_ATTRIBUTE_FN;


private:
    const int PURGE_AMOUNT = 1000;

    std::string instanceFN;
    std::string relationFN;
    std::string mvRelationFN;
    std::string attributeFN;
    std::string settingFN;
    std::string curFileFN;

    static int currentNumber;
    int fileNumber;
    bool purge;

    int getNumberEntities();

    bool doesFileExist(std::string fN);
    void deleteFile(std::string fN);

    std::vector<std::string> tokenize(std::string);
    std::vector<std::pair<std::string, std::vector<std::string>>> generateStrAttributes(std::vector<std::string> line);
};


#endif //REX_LOWMEMORYTAGRAPH_H
