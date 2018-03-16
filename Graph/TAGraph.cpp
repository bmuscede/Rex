/////////////////////////////////////////////////////////////////////////////////////////////////////////
// TAGraph.cpp
//
// Created By: Bryan J Muscedere
// Date: 10/04/17.
//
// Master system for maintaining the in-memory
// digraph system. This is important to store
// C++ entities and then output to TA.
//
// Copyright (C) 2017, Bryan J. Muscedere
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

#include <iostream>
#include <openssl/md5.h>
#include <cstring>
#include <assert.h>
#include "../Walker/ROSWalker.h"
#include "TAGraph.h"

using namespace std;

/**
 * Constructor. Sets up all the member variables.
 * @param edgeType The edge type that forms the forest.
 */
TAGraph::TAGraph(RexEdge::EdgeType edgeType){
    forestEdgeType = edgeType;
    idList = std::unordered_map<std::string, RexNode*>();
    edgeSrcList = std::unordered_map<std::string, std::vector<RexEdge*>>();
    edgeDstList = std::unordered_map<std::string, std::vector<RexEdge*>>();
}

/**
 * Destructor. Deletes all nodes and edges.
 */
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

/**
 * Sets whether we are processing in minimal or full mode.
 * @param minMode Whether we're in minimal mode.
 */
void TAGraph::setMinMode(bool minMode){
    this->minMode = minMode;
}

/**
 * Adds a new node.
 * @param node The node to add.
 */
void TAGraph::addNode(RexNode* node){
    //Convert the node to a hash version.
    string newID = getMD5(node->getID());
    node->setID(newID);

    //Add the node.
    if (idList.find(node->getID()) != idList.end())
        assert("Entry already exists.");
    idList[node->getID()] = node;
}

/**
 * Adds a new edge.
 * @param edge  The edge to add.
 */
void TAGraph::addEdge(RexEdge* edge){
    //Convert the edge to a hash version.
    edge->setSourceID(getMD5(edge->getSourceID()));
    edge->setDestinationID(getMD5(edge->getDestinationID()));

    edgeSrcList[edge->getSourceID()].push_back(edge);
    edgeDstList[edge->getDestinationID()].push_back(edge);
}

void TAGraph::hierarchyRemove(RexNode* toRemove){
    auto edges = findEdgesBySrc(toRemove->getID());
    for (auto &item : edges){
        if (item == nullptr) continue;
        if (item->getType() == RexEdge::COMP_CONTAINS || item->getType() == RexEdge::CONTAINS){
            hierarchyRemove(item->getDestination());
        }
    }

    removeNode(toRemove->getID());
}

/**
 * Removes a node.
 * @param nodeID The ID of the node to remove.
 */
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

/**
 * Removes an edge.
 * @param srcID The source node ID
 * @param dstID The destination node ID
 * @param type The node type.
 * @param hashed Whether the IDs we're searching for are already hashed.
 */
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

/**
 * Finds a node by ID
 * @param nodeID The ID to check.
 * @return The pointer to the node.
 */
RexNode* TAGraph::findNode(std::string nodeID){
    //Check to see if the node exists.
    if (!doesNodeExist(nodeID)) return nullptr;
    return idList[getMD5(nodeID)];
}

/**
 * Finds node by name
 * @param nodeName The name of the node.
 * @param MD5Check Whether we want to hash by MD5
 * @return The pointer to the node.
 */
RexNode* TAGraph::findNodeByName(string nodeName, bool MD5Check) {
    //Loop through the map.
    for (auto entry : idList){
        string entryName = (MD5Check) ? getMD5(entry.second->getName()) : entry.second->getName();
        if (entryName.compare(nodeName) == 0)
            return entry.second;
    }

    return nullptr;
}

/**
 * Finds a node by the end of its name.
 * @param endName The end name string to search for.
 * @param MD5Check Whether we want to hash by MD5
 * @return The pointer to the node.
 */
RexNode* TAGraph::findNodeByEndName(string endName, bool MD5Check) {
    for (auto entry: idList){
        string entryName = (MD5Check) ? getMD5(entry.second->getName()) : entry.second->getName();
        if (hasEnding(entryName, endName)) {
            return entry.second;
        }
    }

    return nullptr;
}

vector<RexNode*> TAGraph::findNodesByType(RexNode::NodeType type){
    vector<RexNode*> nodes;

    for (auto entry : idList){
        if (entry.second->getType() == type) nodes.push_back(entry.second);
    }

    return nodes;
}

/**
 * Find edge bashed on its IDs.
 * @param srcID The source node ID.
 * @param dstID The destination node ID.
 * @param type The node type.
 * @return The edge that was found.
 */
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

/**
 * Finds edge by source ID.
 * @param srcID The source ID.
 * @param md5 Whether we want to hash the ID.
 * @return A vector of matched nodes.
 */
std::vector<RexEdge*> TAGraph::findEdgesBySrc(std::string srcID, bool md5){
    if (md5) return edgeSrcList[getMD5(srcID)];
    return edgeSrcList[srcID];
}

/**
 * Finds edge by destination ID.
 * @param srcID The destination ID.
 * @param md5 Whether we want to hash the ID.
 * @return A vector of matched nodes.
 */
std::vector<RexEdge*> TAGraph::findEdgesByDst(std::string dstID, bool md5){
    if (md5) return edgeDstList[getMD5(dstID)];
    return edgeDstList[dstID];
}

/**
 * Checks whether a node exists.
 * @param nodeID The ID of the node.
 * @return Whether the node exists.
 */
bool TAGraph::doesNodeExist(std::string nodeID){
    if (idList.find(getMD5(nodeID)) == idList.end()) return false;
    return true;
}

/**
 * Checks whether an edge exists.
 * @param srcID The source ID of the node.
 * @param dstID The destination ID of the node.
 * @type type The edge type.
 * @return Whether the edge exists.
 */
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

/**
 * Checks the TA graph for correctness to ensure it conforms to requirements.
 * @return A list of problems.
 */
string TAGraph::checkCorrectness(){
    string correctnessMsg = "";

    //Go through the destination edges.
    for(auto it = edgeDstList.begin(); it != edgeDstList.end(); it++) {
        vector<RexEdge*> edges = it->second;
        bool forestEncountered = false;
        RexEdge* prevForest = nullptr;

        //Go through the edges.
        for (int i = 0; i < edges.size(); i++){
            RexEdge* curEdge = edges.at(i);

            if (curEdge->getType() == forestEdgeType){
                if (forestEncountered){
                    correctnessMsg += "Error! Trying to add edge " + curEdge->getSource()->getName() + " -> " +
                            curEdge->getDestination()->getName() + "!\n";
                    correctnessMsg += "\tEdge " + prevForest->getSource()->getName() + " -> " +
                            prevForest->getDestination()->getName() + " already exists!\n";
                } else {
                    forestEncountered = true;
                    prevForest = curEdge;
                }
            }
        }
    }

    if (correctnessMsg.compare("") != 0){
        correctnessMsg = "MODEL ERRORS:\n-------------------------------------------------------\n" + correctnessMsg;
    }

    return correctnessMsg;
}

/**
 * Reslolves components based on a map of databases to files.
 * @param databaseMap The map of files to compile commands databases.
 * @return Whether the operation was successful.
 */
bool TAGraph::resolveComponents(map<string, vector<string>> databaseMap){
    //Go through the list of nodes.
    for (auto entry : idList){
        RexNode* curNode = entry.second;
        if (curNode->getType() != RexNode::CLASS) continue;

        //Get the filename.
        vector<string> filenames = curNode->getMultiAttribute(FILENAME_ATTR);
        for (string curFN : filenames){
            //Gets a list of components
            vector<string> components = databaseMap[curFN];

            for (string curComp : components){
                //Creates the node.
                RexNode* compNode;
                if (!doesNodeExist(curComp)){
                    compNode = new RexNode(curComp, curComp, RexNode::COMPONENT);
                    addNode(compNode);
                } else {
                    compNode = findNode(curComp);
                }

                //Adds the relationship.
                RexEdge* edge = new RexEdge(compNode, curNode, RexEdge::COMP_CONTAINS);
                addEdge(edge);
            }
        }
    }

    return true;
}

/**
 * Resolves edges that have not been established.
 */
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

/**
 * Removes unestablished edges.
 * @param resolveFirst Whether we resolve them first.
 */
void TAGraph::purgeUnestablishedEdges(bool resolveFirst){
    //We iterate through our edges.
    for(auto it = edgeSrcList.begin(); it != edgeSrcList.end(); it++) {
        vector<RexEdge*> edges = it->second;

        //Go through the list of subedges.
        for (int i = 0; i < edges.size(); i++){
            RexEdge* curEdge = edges.at(i);

            //Check if the edge is established
            if (!curEdge->isEstablished()){
                bool remove = true;
                if (resolveFirst) remove = !resolveEdge(curEdge);
                if (remove) removeEdge(curEdge->getSourceID(), curEdge->getDestinationID(), curEdge->getType(), true);
            }
        }
    }
}

bool TAGraph::keepFeatures(vector<string> features){
    //Find features by name.
    vector<RexNode*> featureNodes = findNodesByType(RexNode::COMPONENT);
    for (RexNode* curNode : featureNodes){
        if (find(features.begin(), features.end(), curNode->getName()) == features.end()){
            hierarchyRemove(curNode);
        }
    }
}

/**
 * Generates the TA model of the graph.
 * @return The string representation of the model.
 */
string TAGraph::getTAModel(){
    string model = (minMode) ? MINI_TA_SCHEMA : FULL_TA_SCHEMA;

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

/**
 * Hashes a string based on the MD5 hash.
 * @param ID The string to hash.
 * @return The hashed string.
 */
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

/**
 * Resolves an edge. If the nodes exist, the edge will be properly resolved.
 * @param edge The edge to resolve.
 * @return Whether it was resolved successfully.
 */
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

    //Checks for callback.
    if (edge->getType() == RexEdge::CALLS && edge->getSource()->getType() == RexNode::SUBSCRIBER &&
            edge->getDestination()->getType() == RexNode::FUNCTION){
        edge->getDestination()->addSingleAttribute(ROSWalker::CALLBACK_FLAG, "1");
    }

    return true;
}

/**
 * Resolves an edge based on the names of the nodes.
 * @param edge The edge to resolve.
 * @return Whether it was resolved successfully.
 */
bool TAGraph::resolveEdgeByName(RexEdge* edge){
    if (edge->isEstablished()) return true;

    //Look for the source and destination.
    if (edge->getSource() == nullptr){
        //Resolves the source ID.
        string sourceName = edge->getSourceID();
        RexNode* srcNode = findNodeByName(sourceName, true);
        if (srcNode == nullptr) return false;

        edge->setSource(srcNode);
    }
    if (edge->getDestination() == nullptr){
        //Resolves the source ID.
        string destName = edge->getDestinationID();
        RexNode* destNode = findNodeByName(destName, true);
        if (destNode == nullptr) return false;

        edge->setDestination(destNode);
    }

    return true;
}

/**
 * Checks if a string has an ending.
 * @param fullString The string to check.
 * @param ending The ending to apply.
 * @return Whether that string has that ending.
 */
bool TAGraph::hasEnding(std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}