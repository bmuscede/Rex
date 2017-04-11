//
// Created by bmuscede on 10/04/17.
//

#ifndef REX_REXEDGE_H
#define REX_REXEDGE_H

#include "RexNode.h"

class RexEdge {
public:
    enum EdgeType {CONTAINS, REFERENCES};

    RexEdge(RexNode* src, RexNode* dst);
    RexEdge(std::string src, std::string dst);
    RexEdge(RexNode* src, std::string dst);
    RexEdge(std::string src, RexNode* dst);
    ~RexEdge();

    bool isEstablished();

    void setSource(RexNode* src);
    void setDestination(RexNode* dst);
    void setType(EdgeType type);

    RexNode* getSource();
    RexNode* getDestination();
    EdgeType getType();
    std::string getSourceID();
    std::string getDestinationID();

    void addSingleAttribute(std::string key, std::string value);
    void addMultiAttribute(std::string key, std::string value);
    std::string getSingleAttribute(std::string key);
    std::vector<std::string> getMultiAttribute(std::string key);

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
