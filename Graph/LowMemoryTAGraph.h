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

    void addNode(RexNode* node) override;
    void addEdge(RexEdge* edge) override;

    std::string getTAModel() override;

    void dumpCurrentFile(int fileNum, std::string file);
    void dumpSettings(std::vector<boost::filesystem::path> files, bool minMode);

    void purgeCurrentGraph();

    std::string checkCorrectness() override;
    void purgeUnestablishedEdges(bool resolveFirst = true) override;

    static const std::string CUR_FILE_LOC;
    static const std::string CUR_SETTING_LOC;

private:
    const int PURGE_AMOUNT = 1000;
    const std::string BASE_INSTANCE_FN = "instances.ta";
    const std::string BASE_RELATION_FN = "relations.ta";
    const std::string BASE_MV_RELATION_FN = "old.relations.ta";
    const std::string BASE_ATTRIBUTE_FN = "attributes.ta";

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

};


#endif //REX_LOWMEMORYTAGRAPH_H
