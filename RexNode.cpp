//
// Created by bmuscede on 10/04/17.
//

#include "RexNode.h"

using namespace std;

RexNode::RexNode(std::string ID, NodeType type){
    this->ID = ID;
    this->type = type;
}

RexNode::RexNode(std::string ID, std::string name, NodeType type){
    this->ID = ID;
    this->name = name;
    this->type = type;
}

RexNode::~RexNode(){ }

std::string RexNode::getID(){
    return ID;
}

std::string RexNode::getName(){
    return name;
}

RexNode::NodeType RexNode::getType(){
    return type;
}

std::string RexNode::getSingleAttribute(std::string key){
    //Check if the item exists.
    if (singleAttributes.find(key) == singleAttributes.end()) return string();
    return singleAttributes[key];
}

std::vector<std::string> RexNode::getMultiAttribute(std::string key){
    //Check if the item exists.
    if (multiAttributes.find(key) == multiAttributes.end()) return vector<string>();
    return multiAttributes[key];
}

void RexNode::setID(std::string newID){
    ID = newID;
}

void RexNode::setName(std::string newName){
    name = newName;
}

void RexNode::setType(NodeType newType){
    type = newType;
}

void RexNode::addSingleAttribute(std::string key, std::string value){
    //Add the KV pair in.
    singleAttributes[key] = value;
}

void RexNode::addMultiAttribute(std::string key, std::string value){
    //Add the pair to the list.
    multiAttributes[key].push_back(value);
}