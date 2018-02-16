/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RexNode.cpp
//
// Created By: Bryan J Muscedere
// Date: 10/04/17.
//
// Maintains an node structure. Basic
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

#include <openssl/md5.h>
#include <cstring>
#include <algorithm>
#include "RexNode.h"

using namespace std;

/**
 * Converts an node type to a string representation.
 * @param type The node type.
 * @return The string representation.
 */
string RexNode::typeToString(RexNode::NodeType type){
    switch (type){
        case FUNCTION:
            return "cFunction";

        case VARIABLE:
            return "cVariable";

        case CLASS:
            return "cClass";

        case TOPIC:
            return "rosTopic";

        case PUBLISHER:
            return "rosPublisher";

        case SUBSCRIBER:
            return "rosSubscriber";

        case NODE_HANDLE:
            return "rosNodeHandle";

        case FILE:
            return "cFile";

        case COMPONENT:
            return "cComponent";
    }

    return "cRoot";
}

/**
 * Creates a Rex node with only an ID.
 * @param ID The ID.
 * @param type The node type.
 */
RexNode::RexNode(std::string ID, NodeType type){
    this->ID = ID;
    this->name = ID;
    this->type = type;
}

/**
 * Creates a Rex node with an ID and name.
 * @param ID The ID.
 * @param name The name.
 * @param type The node type.
 */
RexNode::RexNode(std::string ID, std::string name, NodeType type){
    this->ID = ID;
    this->name = name;
    this->type = type;

    //Add the label.
    addSingleAttribute(LABEL_FLAG, name);
}

/**
 * Destructor
 */
RexNode::~RexNode(){ }

/**
 * Gets the ID.
 * @return The node ID.
 */
std::string RexNode::getID(){
    return ID;
}

/**
 * Gets the name.
 * @return The node name.
 */
std::string RexNode::getName(){
    return name;
}

/**
 * Gets the type.
 * @return The node type.
 */
RexNode::NodeType RexNode::getType(){
    return type;
}

/**
 * Gets a single attribute by key.
 * @param key The key.
 * @return A string of the value.
 */
std::string RexNode::getSingleAttribute(std::string key){
    //Check if the item exists.
    if (singleAttributes.find(key) == singleAttributes.end()) return string();
    return singleAttributes[key];
}

/**
 * Gets a multi attribute by key.
 * @param key The key.
 * @return A vector of values.
 */
std::vector<std::string> RexNode::getMultiAttribute(std::string key){
    //Check if the item exists.
    if (multiAttributes.find(key) == multiAttributes.end()) return vector<string>();
    return multiAttributes[key];
}

/**
 * Gets the number of attributes.
 * @return The number of attributes.
 */
int RexNode::getNumAttributes(){
    return (int) (singleAttributes.size() + multiAttributes.size());
}

/**
 * Sets the ID.
 * @param newID The new ID to add.
 */
void RexNode::setID(std::string newID){
    ID = newID;
}

/**
 * Sets the name.
 * @param newName The new name to add.
 */
void RexNode::setName(std::string newName){
    name = newName;

    //Add the new label.
    addSingleAttribute(LABEL_FLAG, name);
}

/**
 * Sets the node type.
 * @param newType The new type.
 */
void RexNode::setType(NodeType newType){
    type = newType;
}

/**
 * Adds a single attribute.
 * @param key The key.
 * @param value The value.
 */
void RexNode::addSingleAttribute(std::string key, std::string value){
    //Add the KV pair in.
    singleAttributes[key] = value;
}

/**
 * Adds a multi attribute.
 * @param key The key.
 * @param value The value.
 */
void RexNode::addMultiAttribute(std::string key, std::string value){
    //Get the vector for the key.
    vector<string> kV = multiAttributes[key];
    if (std::find(kV.begin(), kV.end(), value) != kV.end()) return;

    //Add the pair to the list.
    multiAttributes[key].push_back(value);
}

/**
 * Generates the TA node string.
 * @return The string TA representation.
 */
string RexNode::generateTANode(){
    return INSTANCE_FLAG + " " + ID + " " + RexNode::typeToString(type);
}

/**
 * Generates the TA attribute string.
 * @return The string TA representation.
 */
string RexNode::generateTAAttribute(){
    string attributes = ID + " { ";
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