//
// Created by bmuscede on 10/04/17.
//

#include <iostream>
#include <openssl/md5.h>
#include <cstring>
#include <assert.h>
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
    //Convert the node to a hash version.
    string newID = getMD5(node->getID());
    node->setID(newID);

    //Add the node.
    if (idList.find(node->getID()) != idList.end())
        assert("Entry already exists.");
    idList[node->getID()] = node;
}

void TAGraph::addEdge(RexEdge* edge){
    //Convert the edge to a hash version.
    edge->setSourceID(getMD5(edge->getSourceID()));
    edge->setDestinationID(getMD5(edge->getDestinationID()));

    edgeSrcList[edge->getSourceID()].push_back(edge);
    edgeDstList[edge->getDestinationID()].push_back(edge);
}

void TAGraph::removeNode(std::string nodeID){
    nodeID = getMD5(nodeID);
    RexNode* node = idList[nodeID];

    //Erase the node.
    idList.erase(nodeID);
    delete node;

    //Erase all pertinent edges.
    vector<RexEdge*> srcs = findEdgesBySrc(nodeID);
    vector<RexEdge*> dsts = findEdgesByDst(nodeID);
    for (auto &item : srcs){
        removeEdge(item->getSourceID(), item->getDestinationID(), item->getType(), true);
    }
    for (auto &item : dsts){
        removeEdge(item->getSourceID(), item->getDestinationID(), item->getType(), true);
    }
}

void TAGraph::removeEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type, bool hashed){
    if (!hashed){
        srcID = getMD5(srcID);
        dstID = getMD5(dstID);
    }

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
    vector<RexEdge*> dstEdges = edgeDstList[dstID];
    for (int i = 0; dstEdges.size(); i++){
        RexEdge* edge = dstEdges.at(i);
        if (edge->getSourceID().compare(srcID) == 0 && edge->getDestinationID().compare(dstID) == 0 &&
            edge->getType() == type){
            edgeDstList[dstID].erase(edgeDstList[dstID].begin() + i);
            break;
        }
    }

    //Removes the edge.
    if (edgeToRemove) delete edgeToRemove;
}

RexNode* TAGraph::findNode(std::string nodeID){
    //Check to see if the node exists.
    if (!doesNodeExist(nodeID)) return nullptr;
    return idList[getMD5(nodeID)];
}

RexEdge* TAGraph::findEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type){
    //Gets the edges for a source.
    vector<RexEdge*> edges = edgeSrcList[getMD5(srcID)];
    for (auto &edge : edges){
        if (edge->getSourceID().compare(getMD5(srcID)) == 0 && edge->getDestinationID().compare(getMD5(dstID)) == 0 &&
                edge->getType() == type){
            return edge;
        }
    }

    return nullptr;
}

std::vector<RexEdge*> TAGraph::findEdgesBySrc(std::string srcID){
    return edgeSrcList[getMD5(srcID)];
}

std::vector<RexEdge*> TAGraph::findEdgesByDst(std::string dstID){
    return edgeDstList[getMD5(dstID)];
}

bool TAGraph::doesNodeExist(std::string nodeID){
    if (idList.find(getMD5(nodeID)) == idList.end()) return false;
    return true;
}

bool TAGraph::doesEdgeExist(std::string srcID, std::string dstID, RexEdge::EdgeType type){
    //Gets the edges for a source.
    vector<RexEdge*> edges = edgeSrcList[getMD5(srcID)];
    for (auto &edge : edges){
        if (edge->getSourceID().compare(getMD5(srcID)) == 0 && edge->getDestinationID().compare(getMD5(dstID)) == 0 &&
            edge->getType() == type){
            return true;
        }
    }

    return false;
}

void TAGraph::resolveUnestablishedEdges(){
    //We go through our edges.
    for(auto it = edgeSrcList.begin(); it != edgeSrcList.end(); it++) {
        vector<RexEdge*> edges = it->second;

        //Go through the list of subedges.
        for (int i = 0; i < edges.size(); i++){
            RexEdge* curEdge = edges.at(i);

            //Check if the edge is established.
            if (!curEdge->isEstablished()){
                resolveEdge(curEdge);
            }
        }
    }
}

void TAGraph::purgeUnestablishedEdges(bool resolveFirst){
    //We iterate through our edges.
    for(auto it = edgeSrcList.begin(); it != edgeSrcList.end(); it++) {
        vector<RexEdge*> edges = it->second;

        //Go through the list of subedges.
        for (int i = 0; i < edges.size(); i++){
            RexEdge* curEdge = edges.at(i);

            //Check if the edge is established.
            if (!curEdge->isEstablished()){
                bool remove = true;
                if (resolveFirst) remove = !resolveEdge(curEdge);
                if (remove) removeEdge(curEdge->getSourceID(), curEdge->getDestinationID(), curEdge->getType(), true);
            }
        }
    }
}

string TAGraph::getTAModel(){
    string model = TA_SCHEMA;

    //Writes the nodes.
    model += "FACT TUPLE :\n";
    for (auto &entry : idList){
        if (entry.second == nullptr) continue;
        model += entry.second->generateTANode() + "\n";
    }

    //Writes the edges.
    for (auto &entry : edgeSrcList){
        for (auto &vecEntry : entry.second){
            model += vecEntry->generateTAEdge() + "\n";
        }
    }

    //Writes the attributes.
    model += "\nFACT ATTRIBUTE :\n";
    for (auto &entry : idList){
        if (entry.second == nullptr) continue;
        if (entry.second->getNumAttributes() == 0) continue;
        model += entry.second->generateTAAttribute() + "\n";
    }
    for (auto &entry : edgeSrcList){
        for (auto &vecEntry : entry.second){
            if (vecEntry->getNumAttributes() == 0) continue;
            model += vecEntry->generateTAAttribute() + "\n";
        }
    }

    return model;
}

string TAGraph::getMD5(string ID){
    //Creates a digest buffer.
    unsigned char digest[MD5_DIGEST_LENGTH];
    const char* cText = ID.c_str();

    //Initializes the MD5 string.
    MD5_CTX ctx;
    MD5_Init(&ctx);
    MD5_Update(&ctx, cText, strlen(cText));
    MD5_Final(digest, &ctx);

    //Fills it with characters.
    char mdString[MD5_LENGTH];
    for (int i = 0; i < 16; i++)
        sprintf(&mdString[i*2], "%02x", (unsigned int)digest[i]);

    return string(mdString);
}

bool TAGraph::resolveEdge(RexEdge *edge) {
    if (edge->isEstablished()) return true;

    //Look for the source and destination.
    if (edge->getSource() == nullptr){
        //Resolves the source ID.
        string sourceID = edge->getSourceID();
        RexNode* srcNode = idList[sourceID];
        if (srcNode == nullptr) return false;

        edge->setSource(srcNode);
    }
    if (edge->getDestination() == nullptr){
        //Resolves the source ID.
        string destID = edge->getDestinationID();
        RexNode* destNode = idList[destID];
        if (destNode == nullptr) return false;

        edge->setDestination(destNode);
    }

    return true;
}