//
// Created by bmuscede on 10/04/17.
//

#include "TAGraph.h"

using namespace std;

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
    RexNode* node = idList[nodeID];

    //Erase the node.
    idList.erase(nodeID);
    delete node;

    //Erase all pertinent edges.
    vector<RexEdge*> srcs = findEdgesBySrc(nodeID);
    vector<RexEdge*> dsts = findEdgesByDst(nodeID);
    for (auto &item : srcs){
        removeEdge(item->getSourceID(), item->getDestinationID(), item->getType());
    }
    for (auto &item : dsts){
        removeEdge(item->getSourceID(), item->getDestinationID(), item->getType());
    }
}

void TAGraph::removeEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type){
    RexEdge* edgeToRemove = nullptr;

    //Starts by finding the node in the source list.
    vector<RexEdge*> srcEdges = edgeSrcList[srcID];
    for (int i = 0; srcEdges.size(); i++){
        RexEdge* edge = srcEdges.at(i);
        if (edge->getSourceID().compare(srcID) == 0 && edge->getDestinationID().compare(dstID) == 0 &&
                edge->getType() == type){
            edgeToRemove = edge;
            edgeSrcList[srcID].erase(edgeSrcList[srcID].begin() + i);
            break;
        }
    }

    //Removes the node in the destination list.
    vector<RexEdge*> dstEdges = edgeSrcList[srcID];
    for (int i = 0; dstEdges.size(); i++){
        RexEdge* edge = dstEdges.at(i);
        if (edge->getSourceID().compare(srcID) == 0 && edge->getDestinationID().compare(dstID) == 0 &&
            edge->getType() == type){
            edgeDstList[srcID].erase(edgeDstList[srcID].begin() + i);
            break;
        }
    }

    //Removes the edge.
    if (edgeToRemove) delete edgeToRemove;
}

RexNode* TAGraph::findNode(std::string nodeID){
    //Check to see if the node exists.
    if (!doesNodeExist(nodeID)) return nullptr;
    return idList[nodeID];
}

RexEdge* TAGraph::findEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type){
    //Gets the edges for a source.
    vector<RexEdge*> edges = edgeSrcList[srcID];
    for (auto &edge : edges){
        if (edge->getSourceID().compare(srcID) == 0 && edge->getDestinationID().compare(dstID) == 0 &&
                edge->getType() == type){
            return edge;
        }
    }

    return nullptr;
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

bool TAGraph::doesEdgeExist(std::string srcID, std::string dstID, RexEdge::EdgeType type){
    //Gets the edges for a source.
    vector<RexEdge*> edges = edgeSrcList[srcID];
    for (auto &edge : edges){
        if (edge->getSourceID().compare(srcID) == 0 && edge->getDestinationID().compare(dstID) == 0 &&
            edge->getType() == type){
            return true;
        }
    }

    return false;
}

void TAGraph::purgeUnestablishedEdges(){
    //We iterate through our edges.
    for(auto it = edgeSrcList.begin(); it != edgeSrcList.end(); it++) {
        vector<RexEdge*> edges = it->second;

        //Go through the list of subedges.
        for (int i = 0; i < edges.size(); i++){
            RexEdge* curEdge = edges.at(i);

            //Check if the edge is established.
            if (!curEdge->isEstablished()){
                removeEdge(curEdge->getSourceID(), curEdge->getDestinationID(), curEdge->getType());
            }
        }
    }
}