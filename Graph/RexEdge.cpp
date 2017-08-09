//
// Created by bmuscede on 10/04/17.
//

#include <iostream>
#include "RexEdge.h"

using namespace std;

string RexEdge::typeToString(RexEdge::EdgeType type){
    switch(type){
        case CONTAINS:
            return "contain";

        case REFERENCES:
            return "reference";

        case CALLS:
            return "call";

        case ADVERTISE:
            return "advertise";

        case SUBSCRIBE:
            return "subscribe";

        case PUBLISH:
            return "publish";
    }

    return "unknown";
}

RexEdge::RexEdge(RexNode* src, RexNode* dst, RexEdge::EdgeType type){
    sourceNode = src;
    destNode = dst;
    sourceID = src->getID();
    destID = dst->getID();
    this->type = type;
}

RexEdge::RexEdge(string src, string dst, RexEdge::EdgeType type){
    sourceNode = nullptr;
    destNode = nullptr;
    sourceID = src;
    destID = dst;
    this->type = type;
}

RexEdge::RexEdge(RexNode* src, string dst, RexEdge::EdgeType type){
    sourceNode = src;
    destNode = nullptr;
    sourceID = src->getID();
    destID = dst;
    this->type = type;
}

RexEdge::RexEdge(string src, RexNode* dst, RexEdge::EdgeType type){
    sourceNode = nullptr;
    destNode = dst;
    sourceID = src;
    destID = dst->getID();
    this->type = type;
}

RexEdge::~RexEdge(){ }

bool RexEdge::isEstablished(){
    //Check to see if the nodes are in place.
    if (sourceNode && destNode) return true;
    return false;
}

void RexEdge::setSource(RexNode* src){
    sourceNode = src;
    sourceID = src->getID();
}

void RexEdge::setDestination(RexNode* dst){
    destNode = dst;
    destID = dst->getID();
}

void RexEdge::setSourceID(string ID){
    if (!sourceNode){
        sourceID = ID;
    }
}

void RexEdge::setDestinationID(string ID){
    if (!destNode){
        destID = ID;
    }
}

void RexEdge::setType(RexEdge::EdgeType type){
    this->type = type;
}

RexNode* RexEdge::getSource(){
    return sourceNode;
}
RexNode* RexEdge::getDestination(){
    return destNode;
}

RexEdge::EdgeType RexEdge::getType(){
    return type;
}

string RexEdge::getSourceID(){
    if (sourceNode) {
        return sourceNode->getID();
    }
    return sourceID;
}

string RexEdge::getDestinationID(){
    if (destNode) {
        return destNode->getID();
    }
    return destID;
}

int RexEdge::getNumAttributes(){
return (int) (singleAttributes.size() + multiAttributes.size());
}

void RexEdge::addSingleAttribute(std::string key, std::string value){
    singleAttributes[key] = value;
}

void RexEdge::addMultiAttribute(string key, string value){
    multiAttributes[key].push_back(value);
}

string RexEdge::getSingleAttribute(string key){
    //Check if the attribute exists.
    if (singleAttributes.find(key) == singleAttributes.end()) return string();
    return singleAttributes[key];
}

vector<string> RexEdge::getMultiAttribute(string key){
    //Check if the attribute exists.
    if (multiAttributes.find(key) == multiAttributes.end()) return vector<string>();
    return multiAttributes[key];
}

string RexEdge::generateTAEdge(){
    return RexEdge::typeToString(type) + " " + getSourceID() + " " + getDestinationID();
}

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