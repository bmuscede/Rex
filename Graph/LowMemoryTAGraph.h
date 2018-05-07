/////////////////////////////////////////////////////////////////////////////////////////////////////////
// LowMemoryTAGraph.h
//
// Created By: Bryan J Muscedere
// Date: 02/05/18.
//
// Handles a run of the low-memory system by dumping items to disk.
// Adds items to the graph and manages resolution and disk dumps.
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

#ifndef REX_LOWMEMORYTAGRAPH_H
#define REX_LOWMEMORYTAGRAPH_H

#include "TAGraph.h"

class LowMemoryTAGraph : public TAGraph {
public:
    /** Constructor / Destructor */
    LowMemoryTAGraph();
    LowMemoryTAGraph(std::string basePath);
    LowMemoryTAGraph(std::string basePath, int curNum);
    ~LowMemoryTAGraph() override;

    /** Changes the Root */
    void changeRoot(std::string basePath);

    /** Adders */
    void addNode(RexNode* node) override;
    void addEdge(RexEdge* edge) override;

    /** TA Generation */
    std::string getTAModel() override;

    /** File Dumping */
    void dumpCurrentFile(int fileNum, std::string file);
    void dumpSettings(std::vector<boost::filesystem::path> files, bool minMode);

    /** Graph Reset */
    void purgeCurrentGraph();

    /** Component Resolution */
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

    /** Helper Methods */
    int getNumberEntities();
    bool doesFileExist(std::string fN);
    void deleteFile(std::string fN);
    std::vector<std::string> tokenize(std::string);
    std::vector<std::pair<std::string, std::vector<std::string>>> generateStrAttributes(std::vector<std::string> line);
};


#endif //REX_LOWMEMORYTAGRAPH_H
