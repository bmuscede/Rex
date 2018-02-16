/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RexEdge.h
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

#ifndef REX_REXEDGE_H
#define REX_REXEDGE_H

#include "RexNode.h"

class RexEdge {
public:
    //Node Type Information
    enum EdgeType {CONTAINS, COMP_CONTAINS, VAR_WRITES,  REFERENCES, CALLS, READS, WRITES,
        ADVERTISE, SUBSCRIBE, PUBLISH, VAR_INFLUENCE, VAR_INFLUENCE_FUNC};
    static std::string typeToString(EdgeType type);

    //Constructor/Destructor
    RexEdge(RexNode* src, RexNode* dst, EdgeType type);
    RexEdge(std::string src, std::string dst, EdgeType type);
    RexEdge(RexNode* src, std::string dst, EdgeType type);
    RexEdge(std::string src, RexNode* dst, EdgeType type);
    ~RexEdge();

    //Information Methods
    bool isEstablished();

    //Setters
    void setSource(RexNode* src);
    void setDestination(RexNode* dst);
    void setSourceID(std::string ID);
    void setDestinationID(std::string ID);
    void setType(EdgeType type);

    //Getters
    RexNode* getSource();
    RexNode* getDestination();
    EdgeType getType();
    std::string getSourceID();
    std::string getDestinationID();
    int getNumAttributes();

    //Attribute Manager
    void addSingleAttribute(std::string key, std::string value);
    void addMultiAttribute(std::string key, std::string value);
    std::string getSingleAttribute(std::string key);
    std::vector<std::string> getMultiAttribute(std::string key);

    //TA Generator
    std::string generateTAEdge();
    std::string generateTAAttribute();

private:
    RexNode* sourceNode;
    RexNode* destNode;

    std::string sourceID;
    std::string destID;

    EdgeType type;

    std::map<std::string, std::string> singleAttributes;
    std::map<std::string, std::vector<std::string>> multiAttributes;
};


#endif //REX_REXEDGE_H
