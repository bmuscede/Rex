/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RexEdge.cpp
//
// Created By: Bryan J Muscedere
// Date: 10/04/17.
//
// Maintains an edge structure. Basic
// system for storing information about
// edge information and for printing to
// TA.
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
#include "RexEdge.h"

using namespace std;

/**
 * Converts an edge type to a string representation.
 * @param type The edge type.
 * @return The string representation.
 */
string RexEdge::typeToString(RexEdge::EdgeType type){
    switch(type){
        case CONTAINS:
            return "contain";

        case COMP_CONTAINS:
            return "compContain";

        case REFERENCES:
            return "reference";

        case CALLS:
            return "call";

        case READS:
            return "read";

        case WRITES:
            return "write";

        case ADVERTISE:
            return "advertise";

        case SUBSCRIBE:
            return "subscribe";

        case PUBLISH:
            return "publish";

        case VAR_WRITES:
            return "varWrite";

        case VAR_INFLUENCE:
            return "varInfluence";

        case VAR_INFLUENCE_FUNC:
            return "varInfFunc";

        case SET_TIME:
            return "time";
    }

    return "unknown";
}

/**
 * Creates an established edge based on two Rex nodes.
 * @param src The pointer to the source.
 * @param dst The pointer to the destination.
 * @param type The edge type.
 */
RexEdge::RexEdge(RexNode* src, RexNode* dst, RexEdge::EdgeType type){
    sourceNode = src;
    destNode = dst;
    sourceID = src->getID();
    destID = dst->getID();
    this->type = type;
}

/**
 * Creates an unestablished edge based on two Rex nodes.
 * @param src The string of the source ID.
 * @param dst The string of the destination ID.
 * @param type The edge type.
 */
RexEdge::RexEdge(string src, string dst, RexEdge::EdgeType type){
    sourceNode = nullptr;
    destNode = nullptr;
    sourceID = src;
    destID = dst;
    this->type = type;
}

/**
 * Creates an unestablished edge based on two Rex nodes.
 * @param src The pointer to the source.
 * @param dst The string of the destination ID.
 * @param type The edge type.
 */
RexEdge::RexEdge(RexNode* src, string dst, RexEdge::EdgeType type){
    sourceNode = src;
    destNode = nullptr;
    sourceID = src->getID();
    destID = dst;
    this->type = type;
}

/**
 * Creates an unestablished edge based on two Rex nodes.
 * @param src The string of the source ID.
 * @param dst The pointer to the destination.
 * @param type The edge type.
 */
RexEdge::RexEdge(string src, RexNode* dst, RexEdge::EdgeType type){
    sourceNode = nullptr;
    destNode = dst;
    sourceID = src;
    destID = dst->getID();
    this->type = type;
}

/**
 * Destructor
 */
RexEdge::~RexEdge(){ }

/**
 * Checks whether the edge is established.
 * @return Whether the edge is established.
 */
bool RexEdge::isEstablished(){
    //Check to see if the nodes are in place.
    if (sourceNode && destNode) return true;
    return false;
}

/**
 * Sets the source node by pointer.
 * @param src The source node.
 */
void RexEdge::setSource(RexNode* src){
    sourceNode = src;
    sourceID = src->getID();
}

/**
 * Sets the destination node by pointer.
 * @param dst The destination node.
 */
void RexEdge::setDestination(RexNode* dst){
    destNode = dst;
    destID = dst->getID();
}

/**
 * Sets the ID of the source.
 * @param ID The ID of the source.
 */
void RexEdge::setSourceID(string ID){
    if (!sourceNode){
        sourceID = ID;
    }
}

/**
 * Sets the ID of the destination.
 * @param ID The ID of the destination.
 */
void RexEdge::setDestinationID(string ID){
    if (!destNode){
        destID = ID;
    }
}

/**
 * Sets the edge type.
 * @param type The new edge type.
 */
void RexEdge::setType(RexEdge::EdgeType type){
    this->type = type;
}

/**
 * Gets the source node.
 * @return The node of the source.
 */
RexNode* RexEdge::getSource(){
    return sourceNode;
}

/**
 * Gets the destination node.
 * @return The node of the destination.
 */
RexNode* RexEdge::getDestination(){
    return destNode;
}

/**
 * Gets the edge type.
 * @return The edge type.
 */
RexEdge::EdgeType RexEdge::getType(){
    return type;
}

/**
 * Gets the source ID.
 * @return The source ID.
 */
string RexEdge::getSourceID(){
    if (sourceNode) {
        return sourceNode->getID();
    }
    return sourceID;
}

/**
 * Gets the destination ID.
 * @return The destination ID.
 */
string RexEdge::getDestinationID(){
    if (destNode) {
        return destNode->getID();
    }
    return destID;
}

/**
 * Gets the number of attributes.
 * @return The number of attributes.
 */
int RexEdge::getNumAttributes(){
return (int) (singleAttributes.size() + multiAttributes.size());
}

/**
 * Adds an attribute that only has one value.
 * @param key The key of the attribute.
 * @param value The value of the attribute.
 */
void RexEdge::addSingleAttribute(std::string key, std::string value){
    singleAttributes[key] = value;
}

/**
 * Adds an attribute that has multiple values.
 * @param key The key of the attribute.
 * @param value The value of the attribute.
 */
void RexEdge::addMultiAttribute(string key, string value){
    multiAttributes[key].push_back(value);
}

/**
 * Get a single attribute by key.
 * @param key The key of the attribute.
 * @return A string of the value.
 */
string RexEdge::getSingleAttribute(string key){
    //Check if the attribute exists.
    if (singleAttributes.find(key) == singleAttributes.end()) return string();
    return singleAttributes[key];
}

/**
 * Get a multi attribute by key.
 * @param key The key of the attribute.
 * @return A string of the value.
 */
vector<string> RexEdge::getMultiAttribute(string key){
    //Check if the attribute exists.
    if (multiAttributes.find(key) == multiAttributes.end()) return vector<string>();
    return multiAttributes[key];
}

/**
 * Generates an edge in TA format.
 * @return A string of the TA representation.
 */
string RexEdge::generateTAEdge(){
    return RexEdge::typeToString(type) + " " + getSourceID() + " " + getDestinationID();
}

/**
 * Generates all attributes in TA format.
 * @return A string of the TA representation.
 */
string RexEdge::generateTAAttribute(){
    string attributes = "(" + generateTAEdge() + ") { ";

    //Starts by generating all the single attributes.
    for (auto &entry : singleAttributes){
        attributes += entry.first + " = " + "\"" + entry.second + "\" ";
    }
    for (auto &entry : multiAttributes){
        attributes += entry.first + " = ( ";
        for (auto &vecEntry : entry.second){
            attributes += vecEntry + " ";
        }
        attributes += ") ";
    }
    attributes += "}";

    return attributes;
}