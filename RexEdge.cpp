//
// Created by bmuscede on 10/04/17.
//

#include "RexEdge.h"

using namespace std;

RexEdge::RexEdge(RexNode* src, RexNode* dst){
    sourceNode = src;
    destNode = dst;
    sourceID = src->getID();
    destID = dst->getID();
}

RexEdge::RexEdge(string src, string dst){
    sourceNode = nullptr;
    destNode = nullptr;
    sourceID = src;
    destID = dst;
}

RexEdge::RexEdge(RexNode* src, string dst){
    sourceNode = src;
    destNode = nullptr;
    sourceID = src->getID();
    destID = dst;
}

RexEdge::RexEdge(string src, RexNode* dst){
    sourceNode = nullptr;
    destNode = dst;
    sourceID = src;
    destID = dst->getID();
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
    return sourceID;
}

string RexEdge::getDestinationID(){
    return destID;
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