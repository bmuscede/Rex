/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RexNode.h
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

#ifndef REX_REXNODE_H
#define REX_REXNODE_H

#include <map>
#include <string>
#include <vector>

class RexNode {
public:
    //Node Type Information
    enum NodeType {FUNCTION, VARIABLE, CLASS, COMPONENT, TOPIC, PUBLISHER, SUBSCRIBER, NODE_HANDLE, FILE};
    static std::string typeToString(NodeType type);

    //Constructor/Destructor
    RexNode(std::string ID, NodeType type);
    RexNode(std::string ID, std::string name, NodeType type);
    ~RexNode();

    //Getters
    std::string getID();
    std::string getName();
    NodeType getType();
    std::string getSingleAttribute(std::string key);
    std::vector<std::string> getMultiAttribute(std::string key);
    int getNumAttributes();

    //Setters
    void setID(std::string newID);
    void setName(std::string newName);
    void setType(NodeType newType);

    //Attribute Managers
    void addSingleAttribute(std::string key, std::string value);
    void addMultiAttribute(std::string key, std::string value);

    //TA Generators
    std::string generateTANode();
    std::string generateTAAttribute();

private:
    std::string ID;
    std::string name;
    NodeType type;

    std::map<std::string, std::string> singleAttributes;
    std::map<std::string, std::vector<std::string>> multiAttributes;

    const std::string INSTANCE_FLAG = "$INSTANCE";
    const std::string LABEL_FLAG = "label";
};

#endif //REX_REXNODE_H
