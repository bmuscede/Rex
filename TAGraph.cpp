//
// Created by bmuscede on 10/04/17.
//

#include "TAGraph.h"

TAGraph::TAGraph(){
    idList = std::unordered_map<std::string, RexNode*>();
    edgeSrcList = std::unordered_map<std::string, std::vector<RexEdge*>>();
    edgeDstList = std::unordered_map<std::string, std::vector<RexEdge*>>();
}

TAGraph::~TAGraph(){
    for (auto &entry : idList){
        delete entry.second;
    }
    for (auto &entry : edgeSrcList){
        for (auto &vecEntry : entry.second){
            delete vecEntry;
        }
    }
}

void TAGraph::addNode(RexNode* node){
    idList[node->getID()] = node;
}

void TAGraph::addEdge(RexEdge* edge){
    edgeSrcList[edge->getSourceID()].push_back(edge);
    edgeDstList[edge->getDestinationID()].push_back(edge);
}

void TAGraph::removeNode(std::string nodeID){
    //Erase the node.
    idList.erase(nodeID);

    //Erase all pertinent edges.
    //TODO
}

void TAGraph::removeEdge(std::string srcID, std::string dstID){
    //TODO
}

RexNode* TAGraph::findNode(std::string nodeID){
    //Check to see if the node exists.
    if (!doesNodeExist(nodeID)) return nullptr;
    return idList[nodeID];
}

RexEdge* TAGraph::findEdge(std::string srcID, std::string dstID){
    //TODO
}

std::vector<RexEdge*> TAGraph::findEdgesBySrc(std::string srcID){
    return edgeSrcList[srcID];
}

std::vector<RexEdge*> TAGraph::findEdgesByDst(std::string dstID){
    return edgeDstList[dstID];
}

bool TAGraph::doesNodeExist(std::string nodeID){
    if (idList.find(nodeID) == idList.end()) return false;
    return true;
}

bool TAGraph::doesEdgeExist(std::string srcID, std::string dstID){
    //TODO
}

void TAGraph::purgeUnestablishedEdges(){
    //TODO
}