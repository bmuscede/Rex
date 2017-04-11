//
// Created by bmuscede on 10/04/17.
//

#ifndef REX_REXNODE_H
#define REX_REXNODE_H

#include <map>
#include <string>
#include <vector>

class RexNode {
public:
    enum NodeType {FUNCTION, VARIABLE, CLASS, TOPIC};

    RexNode(std::string ID, NodeType type);
    RexNode(std::string ID, std::string name, NodeType type);
    ~RexNode();

    std::string getID();
    std::string getName();
    NodeType getType();
    std::string getSingleAttribute(std::string key);
    std::vector<std::string> getMultiAttribute(std::string key);

    void setID(std::string newID);
    void setName(std::string newName);
    void setType(NodeType newType);

    void addSingleAttribute(std::string key, std::string value);
    void addMultiAttribute(std::string key, std::string value);

private:
    std::string ID;
    std::string name;
    NodeType type;

    std::map<std::string, std::string> singleAttributes;
    std::map<std::string, std::vector<std::string>> multiAttributes;
};


#endif //REX_REXNODE_H
