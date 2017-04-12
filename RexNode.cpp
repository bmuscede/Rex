//
// Created by bmuscede on 10/04/17.
//

#include "RexNode.h"

using boost::assign::map_list_of;
using namespace std;

boost::unordered_map<RexNode::NodeType, const char*> nTypeToString = map_list_of
        (RexNode::FUNCTION, "cFunction")
        (RexNode::VARIABLE, "cVariable")
        (RexNode::CLASS, "cClass")
        (RexNode::TOPIC, "rosTopic");

RexNode::RexNode(std::string ID, NodeType type){
    this->ID = ID;
    this->name = ID;
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

string RexNode::generateTANode(){
    return INSTANCE_FLAG + " " + ID + " " + nTypeToString.at(type);
}

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
