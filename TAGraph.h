//
// Created by bmuscede on 10/04/17.
//

#ifndef REX_TAGRAPH_H
#define REX_TAGRAPH_H

#include <unordered_map>
#include <string>
#include <vector>
#include "RexNode.h"
#include "RexEdge.h"

class TAGraph {
public:
    TAGraph();
    ~TAGraph();

    void addNode(RexNode* node);
    void addEdge(RexEdge* edge);

    void removeNode(std::string nodeID);
    void removeEdge(std::string srcID, std::string dstID);

    RexNode* findNode(std::string nodeID);
    RexEdge* findEdge(std::string srcID, std::string dstID);
    std::vector<RexEdge*> findEdgesBySrc(std::string srcID);
    std::vector<RexEdge*> findEdgesByDst(std::string dstID);

    bool doesNodeExist(std::string nodeID);
    bool doesEdgeExist(std::string srcID, std::string dstID);

    void purgeUnestablishedEdges();

private:
    std::unordered_map<std::string, RexNode*> idList;
    std::unordered_map<std::string, std::vector<RexEdge*>> edgeSrcList;
    std::unordered_map<std::string, std::vector<RexEdge*>> edgeDstList;

};


#endif //REX_TAGRAPH_H
