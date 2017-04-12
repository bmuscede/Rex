//
// Created by bmuscede on 10/04/17.
//

#ifndef REX_REXEDGE_H
#define REX_REXEDGE_H

#include <boost/assign/list_of.hpp>
#include <boost/unordered_map.hpp>
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

    std::string generateTAEdge();
    std::string generateTAAttribute();

private:
    const boost::unordered_map<EdgeType, const char*> eTypeToString;

    RexNode* sourceNode;
    RexNode* destNode;

    std::string sourceID;
    std::string destID;

    EdgeType type;

    std::map<std::string, std::string> singleAttributes;
    std::map<std::string, std::vector<std::string>> multiAttributes;
};


#endif //REX_REXEDGE_H
